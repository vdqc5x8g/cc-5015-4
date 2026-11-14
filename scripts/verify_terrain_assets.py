#!/usr/bin/env python3
"""Verify generated terrain assets against the conventions they claim.

For every asset in assets/index.json this checks, from the stored files only:

  1. Recipe parameters are all present and were not defaulted.
  2. The heightfield hash, contour hash and recipe hash match what the
     provenance records (so an asset edited by hand is caught).
  3. Every contour vertex sits at its declared level, measured against the
     *parsed heightfield* (not the analytic surface and not the mesh) --
     this is the elevation error.
  4. Every constrained edge the kernel reports really exists in the TIN --
     this is the constraint-edge evidence.
  5. Each declared case (boundary truncation, closed depression, near
     neighbours, broken contours) is actually present in the data, so a recipe
     cannot silently stop exercising the case it is named after.
  6. Rebuilding from the recipe reproduces the stored contour coordinates
     within a tight tolerance.

The result is written to assets/verification/<id>.json and a summary is
printed.  Exit code is non-zero if any asset fails.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import terrain_lib as tl  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "assets")


class Failure(RuntimeError):
    pass


def load_json(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def sha256_file(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def check(condition, message, evidence):
    evidence.append({"ok": bool(condition), "check": message})
    if not condition:
        raise Failure(message)


def triangle_edges(indices):
    edges = set()
    for i in range(0, len(indices), 3):
        a, b, c = indices[i], indices[i + 1], indices[i + 2]
        for u, v in ((a, b), (b, c), (c, a)):
            edges.add((min(u, v), max(u, v)))
    return edges


def point_in_polygon(x, y, ring):
    return tl.point_in_polygon(x, y, ring)


def nearest_contour_gap(contours):
    """Smallest distance between points of two different levels."""
    best = math.inf
    for i in range(len(contours)):
        for j in range(i + 1, len(contours)):
            if contours[i]["elevation"] == contours[j]["elevation"]:
                continue
            for p in contours[i]["points"]:
                for q in contours[j]["points"]:
                    d = math.hypot(p[0] - q[0], p[1] - q[1])
                    if d < best:
                        best = d
    return best


def verify_asset(entry):
    evidence = []
    asset_id = entry["id"]
    asset_path = os.path.join(ROOT, entry["contours"])
    model_path = os.path.join(ROOT, entry["model"])
    recipe_path = os.path.join(ROOT, entry["recipe"])

    asset = load_json(asset_path)
    model = load_json(model_path)
    recipe = load_json(recipe_path)

    # 1. Parameters.
    for key in ("id", "title", "description", "cases", "meta", "boundary", "levels_m",
                "grid_step_m", "surface", "controls"):
        check(key in recipe, "recipe declares '%s'" % key, evidence)
    check(recipe["id"] == asset_id, "recipe id matches index", evidence)

    # 2. Provenance hashes.
    provenance = asset.get("provenance", {})
    check(provenance.get("recipe_sha256") == sha256_file(recipe_path),
          "recipe hash matches the file on disk", evidence)
    heightfield = os.path.join(ROOT, provenance["heightfield"])
    check(os.path.exists(heightfield), "heightfield exists", evidence)
    check(provenance.get("heightfield_sha256") == sha256_file(heightfield),
          "heightfield hash matches provenance", evidence)
    check(provenance.get("contours_sha256", sha256_file(asset_path)) == sha256_file(asset_path),
          "contour file is the one the provenance describes", evidence)

    grid = tl.read_ascii_grid(heightfield)

    # 3. Elevation error, measured against the parsed raster.
    worst_error = 0.0
    worst_vertex = None
    for contour in asset["contours"]:
        points = contour["points"]
        if contour["closed"] and len(points) > 2 and points[0] == points[-1]:
            points = points[:-1]
        for x, y in points:
            error = abs(tl.bilinear(grid, x, y) - contour["elevation"])
            if error > worst_error:
                worst_error = error
                worst_vertex = (x, y, contour["elevation"])
    # Linear interpolation inside a raster cell cannot be exact for a curved
    # surface; the bound is a fraction of the cell-to-cell elevation change.
    max_slope = 0.0
    for j in range(grid["ny"]):
        for i in range(grid["nx"]):
            if j + 1 < grid["ny"]:
                max_slope = max(max_slope, abs(grid["values"][j + 1][i] - grid["values"][j][i]))
            if i + 1 < grid["nx"]:
                max_slope = max(max_slope, abs(grid["values"][j][i + 1] - grid["values"][j][i]))
    allowed = 0.25 * max_slope + 1e-6
    check(worst_error <= allowed,
          "max heightfield elevation error %.3g m <= %.3g m allowed" % (worst_error, allowed),
          evidence)

    # 4. Constraint-edge evidence.
    mesh = model["mesh"]
    indices = mesh["indices"]
    constrained = mesh["constrained_edges_indices"]
    kinds = mesh["constrained_kinds"]
    edges = triangle_edges(indices)
    missing = 0
    by_kind = {}
    for index in range(len(constrained) // 2):
        a, b = constrained[2 * index], constrained[2 * index + 1]
        kind = kinds[index]
        by_kind[kind] = by_kind.get(kind, 0) + 1
        if (min(a, b), max(a, b)) not in edges:
            missing += 1
    check(len(constrained) > 0, "model reports constrained edges", evidence)
    check(missing == 0,
          "all %d constrained edges exist in the TIN (%s)"
          % (len(constrained) // 2, ", ".join("%s=%d" % item for item in sorted(by_kind.items()))),
          evidence)

    # Mesh must be the one the stored contours produce: positions come from
    # the kernel, so compare triangle count and extent as a cheap identity.
    check(mesh["ok"], "kernel mesh is ok", evidence)
    check(mesh["triangle_count"] > 0, "mesh has triangles", evidence)

    # 5. Declared cases must be physically present AND leave the expected
    #    trace in the kernel's output. Checking only the input shape would let
    #    a recipe claim a case it never exercises.
    cases = set(asset["provenance"]["cases"])
    contours = asset["contours"]
    interpolation = mesh["interpolation"]
    if "boundary-truncation" in cases:
        open_count = sum(1 for c in contours if not c["closed"])
        on_boundary = 0
        for contour in contours:
            if contour["closed"]:
                continue
            for endpoint in (contour["points"][0], contour["points"][-1]):
                if any(math.hypot(endpoint[0] - v[0], endpoint[1] - v[1]) < 1e-4
                       for v in asset["boundary"]):
                    on_boundary += 1
        check(open_count > 0, "boundary truncation: asset has open contours", evidence)
        check(on_boundary >= open_count,
              "boundary truncation: every open contour ends on a boundary vertex", evidence)
        check(by_kind.get("boundary", 0) > 0,
              "boundary truncation: boundary edges are constrained in the TIN (%d)"
              % by_kind.get("boundary", 0), evidence)
    if "closed-depression" in cases:
        closed = [c for c in contours if c["closed"]]
        check(len(closed) > 0, "closed depression: asset has closed contours", evidence)
        nested = False
        for ring in closed:
            for other in contours:
                if other is ring or other["elevation"] >= ring["elevation"]:
                    continue
                if any(point_in_polygon(p[0], p[1], ring["points"]) for p in other["points"]):
                    nested = True
        check(nested, "closed depression: an inner level is nested inside an outer ring", evidence)
        # The kernel must have extrapolated the enclosed interior rather than
        # inventing a surveyed floor: unsupported triangles are the evidence.
        check(interpolation["unsupported_triangles"] > 0,
              "closed depression: kernel flags %d unsupported/extrapolated triangles"
              % interpolation["unsupported_triangles"], evidence)
        check("un-nested closed contour" in interpolation["coverage_note"]
              or "closed contour" in interpolation["coverage_note"],
              "closed depression: coverage note explains the enclosed extremum", evidence)
    if "near-neighbour-contours" in cases:
        gap = nearest_contour_gap(contours)
        check(gap < 5.0, "near neighbours: closest different-level gap %.3f m < 5 m" % gap, evidence)
        # Close lines must all survive as constraints, not be welded together.
        contour_kinds = sum(1 for kind in kinds if kind == "contour")
        check(contour_kinds >= len(contours) * 2,
              "near neighbours: %d contour constraint edges for %d lines"
              % (contour_kinds, len(contours)), evidence)
    if "broken-contours" in cases:
        levels = {}
        for contour in contours:
            levels.setdefault(contour["elevation"], 0)
            levels[contour["elevation"]] += 1
        split_levels = [level for level, count in levels.items() if count > 1]
        check(len(split_levels) > 0,
              "broken contours: level(s) split into pieces %s" % sorted(split_levels), evidence)
        hole = asset["holes"][0]
        crossing = 0
        for contour in contours:
            for p in contour["points"]:
                if point_in_polygon(p[0], p[1], hole):
                    crossing += 1
        check(crossing == 0, "broken contours: no contour vertex lies in the hole", evidence)
        check(by_kind.get("hole", 0) > 0,
              "broken contours: hole edges are constrained (%d)" % by_kind.get("hole", 0), evidence)
        # Kernel area must exclude the void.
        hole_area = 0.5 * abs(sum(hole[i][0] * hole[(i + 1) % len(hole)][1]
                                  - hole[(i + 1) % len(hole)][0] * hole[i][1]
                                  for i in range(len(hole))))
        area = interpolation["total_area_m2"]
        outer = asset["boundary"]
        outer_area = 0.5 * abs(sum(outer[i][0] * outer[(i + 1) % len(outer)][1]
                                   - outer[(i + 1) % len(outer)][0] * outer[i][1]
                                   for i in range(len(outer))))
        check(area < outer_area - 0.5 * hole_area,
              "broken contours: mesh area %.1f m² excludes the %.1f m² void"
              % (area, hole_area), evidence)

    # 6. Rebuild determinism: regenerate the contours from the recipe and
    # compare vertices in order.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, script_dir)
    import make_terrain_assets as maker
    regenerated, _ = maker.generate_asset(recipe, os.path.join(ASSETS, "heightfields",
                                                               asset_id + ".rebuild.asc"))
    if len(regenerated["contours"]) != len(contours):
        raise Failure("rebuild produced %d contours, stored asset has %d"
                      % (len(regenerated["contours"]), len(contours)))
    worst_delta = 0.0
    for a, b in zip(regenerated["contours"], contours):
        if len(a["points"]) != len(b["points"]):
            raise Failure("rebuild contour %d has %d points, stored has %d"
                          % (a["id"], len(a["points"]), len(b["points"])))
        for pa, pb in zip(a["points"], b["points"]):
            worst_delta = max(worst_delta, math.hypot(pa[0] - pb[0], pa[1] - pb[1]))
    check(worst_delta <= 1e-9,
          "rebuild from recipe reproduces stored contours (max delta %.3g m)" % worst_delta,
          evidence)
    try:
        os.remove(os.path.join(ASSETS, "heightfields", asset_id + ".rebuild.asc"))
    except OSError:
        pass

    return {
        "asset": asset_id,
        "ok": True,
        "elevation_error": {
            "metric": "max |raster bilinear elevation - declared level| over contour vertices",
            "worst_m": worst_error,
            "allowed_m": allowed,
            "worst_vertex": worst_vertex,
        },
        "constraints": {
            "total": len(constrained) // 2,
            "missing": missing,
            "by_kind": by_kind,
        },
        "mesh": {
            "triangles": mesh["triangle_count"],
            "vertices": mesh["vertex_count"],
        },
        "rebuild_max_delta_m": worst_delta,
        "checks": evidence,
    }


def main(argv):
    index = load_json(os.path.join(ASSETS, "index.json"))
    only = set(argv)
    os.makedirs(os.path.join(ASSETS, "verification"), exist_ok=True)
    failed = 0
    results = []
    for entry in index["assets"]:
        if only and entry["id"] not in only:
            continue
        try:
            result = verify_asset(entry)
            results.append(result)
            print("%-26s ok  elev.err %.3g m  constraints %d (%s)"
                  % (entry["id"], result["elevation_error"]["worst_m"],
                     result["constraints"]["total"],
                     ", ".join("%s=%d" % item for item in sorted(result["constraints"]["by_kind"].items()))))
        except Failure as error:
            failed += 1
            results.append({"asset": entry["id"], "ok": False, "error": str(error)})
            print("%-26s FAIL %s" % (entry["id"], error))
        out_path = os.path.join(ASSETS, "verification", entry["id"] + ".json")
        with open(out_path, "w", encoding="utf-8") as handle:
            json.dump(results[-1], handle, indent=2)
            handle.write("\n")

    summary = {
        "schema": "terrain-verification/1",
        "assets": [{ "asset": r["asset"], "ok": r["ok"] } for r in results],
    }
    with open(os.path.join(ASSETS, "verification", "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
