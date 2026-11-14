#!/usr/bin/env python3
"""Terrain sample maker.

Turns a declared analytic terrain recipe into contour assets that the
contour -> constrained-TIN kernel consumes directly:

    recipe (assets/source/<id>.json)
      -> contour level-set extraction (marching squares, boundary + hole clipped)
      -> kernel input     (assets/contours/<id>.json)
      -> constrained TIN  (assets/models/<id>.obj, <id>_pipeline.json)
      -> preview          (assets/previews/<id>.svg)
      -> verification     (assets/verification/<id>.json)

Every recipe value is required.  There are deliberately no defaults: a recipe
that omits a parameter fails loudly instead of silently producing an asset
that merely looks like terrain.  The only derived values (control elevations)
must opt in with `"elevation_from_surface": true`.

Usage:
    python3 scripts/make_terrain_assets.py                 # build every recipe
    python3 scripts/make_terrain_assets.py ridge_truncated # build one by id
    python3 scripts/make_terrain_assets.py --recipe path/to/recipe.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import terrain_lib as tl  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, "build", "terrain")
SOURCE_DIR = os.path.join(ROOT, "assets", "source")
HEIGHTFIELD_DIR = os.path.join(ROOT, "assets", "heightfields")
CONTOUR_DIR = os.path.join(ROOT, "assets", "contours")
MODEL_DIR = os.path.join(ROOT, "assets", "models")
PREVIEW_DIR = os.path.join(ROOT, "assets", "previews")
VERIFY_DIR = os.path.join(ROOT, "assets", "verification")
INDEX_PATH = os.path.join(ROOT, "assets", "index.json")
GENERATOR_VERSION = "terrain-maker/1.0"

REQUIRED_RECIPE_KEYS = (
    "id", "title", "description", "cases", "meta", "boundary", "levels_m",
    "grid_step_m", "surface", "controls",
)
REQUIRED_META_KEYS = ("xy_unit", "xy_meters_per_unit", "z_unit", "z_meters_per_unit", "z_datum")


class MakerError(RuntimeError):
    pass


def require(recipe, key, where):
    if key not in recipe:
        raise MakerError("%s: missing required key '%s'" % (where, key))
    return recipe[key]


def sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha256_file(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def validate_recipe(recipe, where):
    for key in REQUIRED_RECIPE_KEYS:
        require(recipe, key, where)
    for key in REQUIRED_META_KEYS:
        if key not in recipe["meta"]:
            raise MakerError("%s.meta: missing required key '%s'" % (where, key))
    if not isinstance(recipe["levels_m"], list) or len(recipe["levels_m"]) < 2:
        raise MakerError("%s: levels_m must list at least two levels" % where)
    seen = set()
    for level in recipe["levels_m"]:
        if isinstance(level, bool) or not isinstance(level, (int, float)):
            raise MakerError("%s: every level in levels_m must be a number" % where)
        if level in seen:
            raise MakerError("%s: duplicate level %r; each level must be unique" % (where, level))
        seen.add(level)
    if recipe["levels_m"] != sorted(recipe["levels_m"]):
        raise MakerError("%s: levels_m must be ascending" % where)
    step = require(recipe, "grid_step_m", where)
    if not isinstance(step, (int, float)) or step <= 0:
        raise MakerError("%s: grid_step_m must be a positive number" % where)
    holes = recipe.get("holes", [])
    if not isinstance(holes, list):
        raise MakerError("%s: holes must be an array of rings" % where)
    for index, hole in enumerate(holes):
        tl.boundary_bounds(hole, "%s.holes[%d]" % (where, index))
    tl.boundary_bounds(recipe["boundary"], where + ".boundary")
    for index, control in enumerate(recipe["controls"]):
        context = "%s.controls[%d]" % (where, index)
        if "name" not in control or "position" not in control:
            raise MakerError(context + ": needs 'name' and 'position'")
        if control.get("elevation_from_surface") is not True:
            raise MakerError(context + ": set 'elevation_from_surface': true; "
                             "a literal elevation would be silently detached from the terrain")
        if "elevation" in control:
            raise MakerError(context + ": elevation is derived; do not also supply a literal value")
    for index, fault in enumerate(recipe.get("faults", [])):
        context = "%s.faults[%d]" % (where, index)
        if "name" not in fault or "points" not in fault:
            raise MakerError(context + ": needs 'name' and 'points'")
        if len(fault["points"]) < 2:
            raise MakerError(context + ": needs at least two points")
    surface = tl.validate_surface(recipe["surface"], where + ".surface")
    return surface


def generate_asset(recipe, heightfield_path):
    where = recipe["id"]
    surface = validate_recipe(recipe, where)
    boundary = recipe["boundary"]
    holes = recipe.get("holes", [])
    bounds = tl.boundary_bounds(boundary, where + ".boundary")
    step = float(recipe["grid_step_m"])

    lo, hi = tl.surface_extrema(surface, bounds, step)
    for level in recipe["levels_m"]:
        if not (lo[0] < level < hi[0]):
            raise MakerError("%s: level %g is outside the terrain range %.3f .. %.3f; "
                             "the recipe would produce no contour for it" % (where, level, lo[0], hi[0]))

    # The raster is the terrain source of record: write it, then parse it back
    # and extract levels from the parsed grid.  A bug in the raster writer
    # therefore changes the contours, which the verifier reconciles.
    grid = tl.sample_heightfield(surface, bounds, step)
    tl.write_ascii_grid(heightfield_path, grid, recipe["meta"])
    grid = tl.read_ascii_grid(heightfield_path)
    heightfield_sha = sha256_file(heightfield_path)

    contours = []
    contour_id = 0
    level_stats = []
    for level in recipe["levels_m"]:
        pieces = tl.extract_level_from_grid(grid, boundary, level, holes)
        if not pieces:
            raise MakerError("%s: level %g produced no contour; adjust levels_m or the surface" % (where, level))
        for piece in pieces:
            contour_id += 1
            points = piece["points"]
            points_for_error = points[:-1] if (piece["closed"] and len(points) > 2
                                               and points[0] == points[-1]) else points
            # Error is measured against the parsed raster, not the analytic
            # function, because the raster is what drove the extraction.
            error = max(abs(tl.bilinear(grid, x, y) - level) for x, y in points_for_error)
            contours.append({
                "id": contour_id,
                "name": "z=%g" % level,
                "closed": piece["closed"],
                "elevation": level,
                "points": points,
                "attributes": {
                    "source": GENERATOR_VERSION,
                    "level_m": "%g" % level,
                },
            })
            level_stats.append({"level_m": level, "closed": piece["closed"],
                                "points": len(points), "marching_error_m": error})

    # Open contour ends land on the boundary or a hole ring.  The kernel
    # enforces constraints exactly, so add a ring vertex at every meeting point
    # and place the contour endpoint on that vertex.  Without this, a contour
    # stopping part-way along a ring segment is a T-junction the TIN cannot
    # represent and the constraint audit rejects the build.
    open_ends = []
    for contour in contours:
        if not contour["closed"]:
            open_ends.append(contour["points"][0])
            open_ends.append(contour["points"][-1])
    new_boundary, boundary_inserted = tl.split_ring_at_points(boundary, open_ends, tol=1e-6)
    new_holes = []
    hole_inserted = 0
    for hole in holes:
        updated, inserted = tl.split_ring_at_points(hole, open_ends, tol=1e-6)
        new_holes.append(updated)
        hole_inserted += inserted

    all_rings = [new_boundary] + new_holes

    def snap_to_ring(point):
        best = None
        for ring in all_rings:
            for vertex in ring:
                d = ((point[0] - vertex[0]) ** 2 + (point[1] - vertex[1]) ** 2) ** 0.5
                if d <= 1e-6 and (best is None or d < best[0]):
                    best = (d, vertex)
        return list(best[1]) if best else point

    for contour in contours:
        if contour["closed"]:
            continue
        contour["points"][0] = snap_to_ring(contour["points"][0])
        contour["points"][-1] = snap_to_ring(contour["points"][-1])

    controls = []
    for index, control in enumerate(recipe["controls"]):
        x, y = control["position"]
        controls.append({
            "id": index + 1,
            "name": control["name"],
            "position": [x, y],
            "elevation": round(tl.surface_elevation(surface, x, y), 4),
            "attributes": {"elevation_source": "recipe surface sampled at position"},
        })

    faults = []
    for index, fault in enumerate(recipe.get("faults", [])):
        faults.append({"id": index + 1, "name": fault["name"],
                       "points": [[float(x), float(y)] for x, y in fault["points"]]})

    asset = {
        "name": recipe["id"],
        "title": recipe["title"],
        "description": recipe["description"],
        "meta": recipe["meta"],
        "boundary": new_boundary,
        "holes": new_holes,
        "contours": contours,
        "controls": controls,
        "faults": faults,
        "provenance": {
            "generator": GENERATOR_VERSION,
            "recipe": "assets/source/%s.json" % recipe["id"],
            "recipe_schema": "terrain-recipe/1",
            "surface": surface,
            "heightfield": "assets/heightfields/%s.asc" % recipe["id"],
            "heightfield_sha256": heightfield_sha,
            "grid_step_m": step,
            "levels_m": recipe["levels_m"],
            "boundary_vertices_inserted": boundary_inserted,
            "hole_vertices_inserted": hole_inserted,
            "cases": recipe["cases"],
            "derivation": "terrain raster written from the analytic surface, re-parsed, "
                          "then marching-squares level sets clipped exactly to boundary and holes; "
                          "rings split at contour meeting points so constraints are representable",
        },
    }
    return asset, {"extrema": {"min": lo, "max": hi}, "levels": level_stats,
                   "boundary_inserted": boundary_inserted, "hole_inserted": hole_inserted}


def write_json(path, payload):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    text = json.dumps(payload, indent=2, sort_keys=False) + "\n"
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return text


def build_model(asset_path, asset_id):
    """Run the existing triangulation kernel on the generated contours."""
    if not os.path.exists(BIN):
        raise MakerError("missing %s; run `make` before generating assets" % BIN)
    prefix = os.path.join(MODEL_DIR, asset_id)
    os.makedirs(MODEL_DIR, exist_ok=True)
    proc = subprocess.run([BIN, "build", asset_path, "--out-prefix", prefix],
                          capture_output=True, text=True, cwd=ROOT)
    if proc.returncode != 0:
        raise MakerError("kernel build failed for %s:\n%s\n%s" % (asset_id, proc.stdout, proc.stderr))
    pipeline = prefix + "_pipeline.json"
    with open(pipeline, encoding="utf-8") as handle:
        doc = json.load(handle)
    if not doc.get("mesh", {}).get("ok"):
        raise MakerError("kernel reported a failed mesh for %s: %s"
                         % (asset_id, doc.get("mesh", {}).get("error")))
    # The CLI also writes <prefix>_mesh.json, which carries wall-clock timings.
    # Our reproducibility check compares every derived file byte for byte, so
    # keep only the deterministic export set: OBJ, stripped pipeline, model.
    for extra in (prefix + "_mesh.json", prefix + ".stl", prefix + "_vertices.csv",
                  prefix + "_triangles.csv"):
        try:
            os.remove(extra)
        except OSError:
            pass
    return prefix, doc, proc.stdout.strip()


def write_preview(asset, path):
    """A real vector preview: plan view, level labels, boundary and holes."""
    points = []
    for p in asset["boundary"]:
        points.append(p)
    for contour in asset["contours"]:
        points.extend(contour["points"])
    for hole in asset.get("holes", []):
        points.extend(hole)
    if not points:
        raise MakerError("%s: asset has no geometry to preview" % asset["name"])
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width, height = 960.0, 720.0
    pad = 28.0
    scale = min((width - 2 * pad) / max(max_x - min_x, 1e-9),
                (height - 2 * pad) / max(max_y - min_y, 1e-9))
    def tx(p):
        return pad + (p[0] - min_x) * scale, pad + (max_y - p[1]) * scale

    palette = ["#ffd166", "#ff9f4a", "#9be07a", "#6fd3e0", "#b79cff", "#ff7fb0", "#f2f2f2"]
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">'
        % (width, height, width, height),
        '<rect width="100%" height="100%" fill="#0a0f15"/>',
    ]
    boundary = asset["boundary"]
    boundary_path = "M " + " L ".join("%.2f %.2f" % tx(p) for p in boundary) + " Z"
    parts.append('<path d="%s" fill="#101a24" stroke="#5fa8ff" stroke-width="2"/>' % boundary_path)
    for hole in asset.get("holes", []):
        hole_path = "M " + " L ".join("%.2f %.2f" % tx(p) for p in hole) + " Z"
        parts.append('<path d="%s" fill="#0a0f15" stroke="#b07cff" stroke-width="2"/>' % hole_path)
    for index, contour in enumerate(asset["contours"]):
        color = palette[index % len(palette)]
        d = "M " + " L ".join("%.2f %.2f" % tx(p) for p in contour["points"])
        if contour["closed"]:
            d += " Z"
        parts.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.6"/>' % (d, color))
        label = contour["points"][len(contour["points"]) // 2]
        lx, ly = tx(label)
        parts.append('<text x="%.1f" y="%.1f" fill="%s" font-family="monospace" '
                     'font-size="12" text-anchor="middle">%g</text>'
                     % (lx, ly, color, contour["elevation"]))
    for control in asset.get("controls", []):
        cx, cy = tx(control["position"])
        parts.append('<circle cx="%.1f" cy="%.1f" r="4" fill="#ffffff"/>' % (cx, cy))
        parts.append('<text x="%.1f" y="%.1f" fill="#ffffff" font-family="monospace" '
                     'font-size="11">%s %g</text>'
                     % (cx + 7, cy - 6, control["name"], control["elevation"]))
    parts.append('<text x="16" y="24" fill="#8fa6bb" font-family="monospace" font-size="13">%s</text>'
                 % asset["title"])
    parts.append("</svg>")
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(parts) + "\n")


def strip_timings(pipeline):
    """Keep the model reproducible: drop wall-clock values from the saved copy."""
    clone = json.loads(json.dumps(pipeline))
    for key in ("total_ms",):
        clone.pop(key, None)
    mesh = clone.get("mesh", {})
    for key in ("triangulation_ms", "elevation_ms", "total_ms"):
        mesh.pop(key, None)
    clone.get("validation", {}).pop("elapsed_ms", None)
    return clone


def process_recipe(recipe, recipe_path):
    asset_id = recipe["id"]
    os.makedirs(HEIGHTFIELD_DIR, exist_ok=True)
    heightfield_path = os.path.join(HEIGHTFIELD_DIR, asset_id + ".asc")
    asset, stats = generate_asset(recipe, heightfield_path)

    source_path = os.path.abspath(recipe_path)
    if not os.path.exists(source_path):
        raise MakerError("recipe not found: " + source_path)
    # Hash the recipe file as authored.  The maker never rewrites it, so a
    # hash mismatch during rebuild means the recipe itself changed.
    recipe_hash = sha256_file(source_path)

    asset["provenance"]["recipe_sha256"] = recipe_hash
    contour_path = os.path.join(CONTOUR_DIR, asset_id + ".json")
    asset_text = write_json(contour_path, asset)

    prefix, pipeline, kernel_log = build_model(contour_path, asset_id)
    model = {
        "name": asset_id,
        "title": recipe["title"],
        "provenance": {
            "generator": GENERATOR_VERSION,
            "recipe": "assets/source/%s.json" % asset_id,
            "recipe_sha256": recipe_hash,
            "heightfield": "assets/heightfields/%s.asc" % asset_id,
            "heightfield_sha256": asset["provenance"]["heightfield_sha256"],
            "contours": "assets/contours/%s.json" % asset_id,
            "contours_sha256": sha256_text(asset_text),
            "kernel": "build/terrain build (CDT constrained TIN)",
            "kernel_summary": kernel_log.splitlines()[:2],
        },
        "tolerance": pipeline["tolerance"],
        "mesh": {
            "ok": pipeline["mesh"]["ok"],
            "vertex_count": pipeline["mesh"]["vertex_count"],
            "triangle_count": pipeline["mesh"]["triangle_count"],
            "edge_count": pipeline["mesh"]["edge_count"],
            "interpolation": pipeline["mesh"]["interpolation"],
            "positions": pipeline["mesh"]["positions"],
            "indices": pipeline["mesh"]["indices"],
            "sources": pipeline["mesh"]["sources"],
            "unsupported": pipeline["mesh"]["unsupported"],
            "constrained_edges_indices": pipeline["mesh"]["constrained_edges_indices"],
            "constrained_kinds": pipeline["mesh"]["constrained_kinds"],
        },
    }
    write_json(os.path.join(MODEL_DIR, asset_id + "_model.json"), model)
    write_json(os.path.join(MODEL_DIR, asset_id + "_pipeline.json"), strip_timings(pipeline))

    preview_path = os.path.join(PREVIEW_DIR, asset_id + ".svg")
    write_preview(asset, preview_path)

    return {
        "id": asset_id,
        "title": recipe["title"],
        "description": recipe["description"],
        "cases": recipe["cases"],
        "recipe": os.path.relpath(source_path, ROOT),
        "recipe_sha256": recipe_hash,
        "contours": os.path.relpath(contour_path, ROOT),
        "contours_sha256": sha256_text(asset_text),
        "model": os.path.relpath(os.path.join(MODEL_DIR, asset_id + "_model.json"), ROOT),
        "mesh_obj": os.path.relpath(prefix + ".obj", ROOT),
        "preview": os.path.relpath(preview_path, ROOT),
        "verification": os.path.relpath(os.path.join(VERIFY_DIR, asset_id + ".json"), ROOT),
        "level_stats": stats["levels"],
        "extrema": stats["extrema"],
        "contour_count": len(asset["contours"]),
    }


def update_index(entries):
    index = {
        "schema": "terrain-assets/1",
        "generator": GENERATOR_VERSION,
        "note": "Each asset is rebuilt from its recipe by scripts/rebuild_assets.py; "
                "contours, models and previews are derived, not hand-edited.",
        "assets": sorted(entries, key=lambda e: e["id"]),
    }
    write_json(INDEX_PATH, index)


def load_recipe(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def main(argv):
    parser = argparse.ArgumentParser(description="Generate contour assets from terrain recipes")
    parser.add_argument("ids", nargs="*", help="recipe ids to build (default: all)")
    parser.add_argument("--recipe", action="append", default=[],
                        help="explicit recipe path (repeatable)")
    args = parser.parse_args(argv)

    os.makedirs(SOURCE_DIR, exist_ok=True)
    os.makedirs(CONTOUR_DIR, exist_ok=True)
    os.makedirs(MODEL_DIR, exist_ok=True)
    os.makedirs(PREVIEW_DIR, exist_ok=True)

    recipe_paths = list(args.recipe)
    if not recipe_paths:
        if args.ids:
            recipe_paths = [os.path.join(SOURCE_DIR, asset_id + ".json") for asset_id in args.ids]
        else:
            recipe_paths = sorted(os.path.join(SOURCE_DIR, name)
                                  for name in os.listdir(SOURCE_DIR)
                                  if name.endswith(".json"))
    if not recipe_paths:
        raise SystemExit("no recipes found in " + os.path.relpath(SOURCE_DIR, ROOT))

    entries = []
    if os.path.exists(INDEX_PATH):
        try:
            with open(INDEX_PATH, encoding="utf-8") as handle:
                entries = json.load(handle).get("assets", [])
        except (OSError, ValueError):
            entries = []
    by_id = {entry["id"]: entry for entry in entries}

    for path in recipe_paths:
        if not os.path.exists(path):
            raise SystemExit("recipe not found: " + path)
        recipe = load_recipe(path)
        if "id" not in recipe:
            raise SystemExit(path + ": missing required key 'id'")
        entry = process_recipe(recipe, path)
        by_id[entry["id"]] = entry
        print("asset %-22s %2d contours  %s" % (entry["id"], entry["contour_count"],
                                                entry["model"]))
    update_index(list(by_id.values()))
    print("wrote " + os.path.relpath(INDEX_PATH, ROOT))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except (MakerError, tl.RecipeError) as error:
        print("terrain maker: " + str(error), file=sys.stderr)
        sys.exit(1)
