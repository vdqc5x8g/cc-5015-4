#!/usr/bin/env python3
"""Terrain surface + marching-squares contour extraction.

This module is the numerical core shared by the sample maker, the asset
rebuild step and the verifier.  It contains no defaults for recipe values:
every coefficient a recipe needs must be present, because a silent fallback
would hide a wrong recipe behind a plausible-looking asset.

Coordinate convention: x/y are metres in the asset's declared unit, z is
metres.  The surface is a sum of anisotropic Gaussians plus an optional plane
tilt, which is analytic and therefore lets the verifier measure elevation
error against the terrain that produced the contours.
"""

from __future__ import annotations

import math


RECIPE_SCHEMA_TERRAIN = {
    "kind": str,
    "base_m": float,
    "slope_x_m_per_m": float,
    "slope_y_m_per_m": float,
    "terms": list,
}
TERM_SCHEMA = {"amplitude_m": float, "cx": float, "cy": float, "sx": float, "sy": float}

# Marching-squares edge ids.  A crossing is identified by the grid edge it lies
# on, never by a recomputed floating-point coordinate, so two cells sharing an
# edge always produce the exact same endpoint.
H, V = 0, 1


class RecipeError(ValueError):
    """A recipe is missing a value or contains one that cannot be honoured."""


def _require_float(container, key, where):
    if key not in container:
        raise RecipeError("%s: missing required parameter '%s'" % (where, key))
    value = container[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RecipeError("%s: parameter '%s' must be a number, got %r" % (where, key, value))
    value = float(value)
    if not math.isfinite(value):
        raise RecipeError("%s: parameter '%s' must be finite" % (where, key))
    return value


def validate_surface(surface, where="surface"):
    if not isinstance(surface, dict):
        raise RecipeError(where + ": must be an object")
    kind = surface.get("kind")
    if kind != "sum_of_gaussians":
        raise RecipeError("%s: kind must be 'sum_of_gaussians', got %r" % (where, kind))
    base = _require_float(surface, "base_m", where)
    tilt_x = _require_float(surface, "slope_x_m_per_m", where)
    tilt_y = _require_float(surface, "slope_y_m_per_m", where)
    terms = surface.get("terms")
    if not isinstance(terms, list) or not terms:
        raise RecipeError(where + ": 'terms' must be a non-empty array")
    cleaned = []
    for index, term in enumerate(terms):
        if not isinstance(term, dict):
            raise RecipeError("%s.terms[%d]: must be an object" % (where, index))
        row = {key: _require_float(term, key, "%s.terms[%d]" % (where, index))
               for key in TERM_SCHEMA}
        if row["sx"] <= 0.0 or row["sy"] <= 0.0:
            raise RecipeError("%s.terms[%d]: sx and sy must be > 0" % (where, index))
        cleaned.append(row)
    return {"kind": kind, "base_m": base, "slope_x_m_per_m": tilt_x,
            "slope_y_m_per_m": tilt_y, "terms": cleaned}


def surface_elevation(surface, x, y):
    z = surface["base_m"] + surface["slope_x_m_per_m"] * x + surface["slope_y_m_per_m"] * y
    for term in surface["terms"]:
        dx = (x - term["cx"]) / term["sx"]
        dy = (y - term["cy"]) / term["sy"]
        z += term["amplitude_m"] * math.exp(-0.5 * (dx * dx + dy * dy))
    return z


def surface_gradient(surface, x, y, h=1e-4):
    gx = (surface_elevation(surface, x + h, y) - surface_elevation(surface, x - h, y)) / (2 * h)
    gy = (surface_elevation(surface, x, y + h) - surface_elevation(surface, x, y - h)) / (2 * h)
    return gx, gy


def surface_extrema(surface, bounds, step):
    """Brute-force grid extrema, used by the maker to require sane levels."""
    min_x, min_y, max_x, max_y = bounds
    best_lo = (math.inf, 0.0, 0.0)
    best_hi = (-math.inf, 0.0, 0.0)
    nx = max(2, int(round((max_x - min_x) / step)) + 1)
    ny = max(2, int(round((max_y - min_y) / step)) + 1)
    for j in range(ny):
        y = min_y + (max_y - min_y) * j / (ny - 1)
        for i in range(nx):
            x = min_x + (max_x - min_x) * i / (nx - 1)
            z = surface_elevation(surface, x, y)
            if z < best_lo[0]:
                best_lo = (z, x, y)
            if z > best_hi[0]:
                best_hi = (z, x, y)
    return best_lo, best_hi


def point_in_polygon(x, y, ring):
    inside = False
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xin:
                inside = not inside
    return inside


def boundary_bounds(ring, where="boundary"):
    if not isinstance(ring, list) or len(ring) < 3:
        raise RecipeError(where + ": boundary must have at least three [x, y] points")
    xs = []
    ys = []
    for index, point in enumerate(ring):
        if not isinstance(point, list) or len(point) != 2:
            raise RecipeError("%s[%d]: must be [x, y]" % (where, index))
        x = _require_float({"x": point[0]}, "x", "%s[%d]" % (where, index))
        y = _require_float({"y": point[1]}, "y", "%s[%d]" % (where, index))
        xs.append(x)
        ys.append(y)
    return min(xs), min(ys), max(xs), max(ys)


def _axis_values(lo, hi, step):
    count = int(math.floor((hi - lo) / step + 1e-9)) + 1
    values = [lo + step * i for i in range(count)]
    if values[-1] < hi - 1e-9:
        values.append(hi)
    else:
        values[-1] = hi
    return values


def sample_heightfield(surface, bounds, step):
    """Rasterise the terrain into an elevation grid.

    The grid is the asset's terrain source: contours are later extracted from
    the *parsed* raster, so the pipeline really does go through a heightfield
    rather than bypassing it.
    """
    min_x, min_y, max_x, max_y = bounds
    xs = _axis_values(min_x, max_x, step)
    ys = _axis_values(min_y, max_y, step)
    values = [[surface_elevation(surface, x, y) for x in xs] for y in ys]
    return {"x0": xs[0], "y0": ys[0], "step": step, "nx": len(xs), "ny": len(ys),
            "xs": xs, "ys": ys, "values": values}


def write_ascii_grid(path, grid, meta):
    """Write an ESRI-ASCII-like elevation grid (rows north to south)."""
    lines = []
    lines.append("ncols %d" % grid["nx"])
    lines.append("nrows %d" % grid["ny"])
    lines.append("xllcorner %r" % grid["x0"])
    lines.append("yllcorner %r" % grid["y0"])
    lines.append("cellsize %r" % grid["step"])
    lines.append("NODATA_value -99999")
    lines.append("z_unit %s" % meta["z_unit"])
    lines.append("z_meters_per_unit %r" % meta["z_meters_per_unit"])
    lines.append("z_datum %s" % meta["z_datum"].replace(" ", "_"))
    for row in reversed(grid["values"]):
        lines.append(" ".join("%.6f" % value for value in row))
    os_path = path
    import os as _os
    _os.makedirs(_os.path.dirname(os_path), exist_ok=True)
    with open(os_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def read_ascii_grid(path):
    """Parse the elevation grid back out of the file.

    Parsing validates every structural value; a malformed raster is an error,
    not a silently zero-filled grid.
    """
    header = {}
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            head = parts[0].lower()
            if head in ("ncols", "nrows", "xllcorner", "yllcorner", "cellsize",
                        "nodata_value", "z_unit", "z_meters_per_unit", "z_datum"):
                header[head] = parts[1] if len(parts) > 1 else ""
            else:
                rows.append([float(value) for value in parts])

    for key in ("ncols", "nrows", "xllcorner", "yllcorner", "cellsize", "nodata_value"):
        if key not in header:
            raise tl_error("heightfield %s: missing header '%s'" % (path, key))
    nx = int(header["ncols"])
    ny = int(header["nrows"])
    if len(rows) != ny:
        raise tl_error("heightfield %s: expected %d rows, found %d" % (path, ny, len(rows)))
    for index, row in enumerate(rows):
        if len(row) != nx:
            raise tl_error("heightfield %s: row %d has %d cells, expected %d"
                           % (path, index, len(row), nx))
    nodata = float(header["nodata_value"])
    x0 = float(header["xllcorner"])
    y0 = float(header["yllcorner"])
    step = float(header["cellsize"])
    # File rows run north -> south; internal rows run south -> north.
    values = list(reversed(rows))
    for j, row in enumerate(values):
        for i, value in enumerate(row):
            if value == nodata:
                raise tl_error("heightfield %s: NODATA cell at (%d, %d); the maker does not "
                               "invent elevations" % (path, i, j))
    xs = [x0 + step * i for i in range(nx)]
    ys = [y0 + step * j for j in range(ny)]
    return {"x0": x0, "y0": y0, "step": step, "nx": nx, "ny": ny,
            "xs": xs, "ys": ys, "values": values,
            "z_unit": header.get("z_unit"), "z_datum": header.get("z_datum")}


def tl_error(message):
    raise RecipeError(message)


def grid_value(grid, i, j):
    return grid["values"][j][i]


def bilinear(grid, x, y):
    """Bilinear elevation at (x, y), clamped to the raster footprint."""
    nx, ny = grid["nx"], grid["ny"]
    fi = (x - grid["x0"]) / grid["step"]
    fj = (y - grid["y0"]) / grid["step"]
    fi = 0.0 if fi < 0 else nx - 1.000001 if fi > nx - 1 else fi
    fj = 0.0 if fj < 0 else ny - 1.000001 if fj > ny - 1 else fj
    i0 = int(math.floor(fi))
    j0 = int(math.floor(fj))
    i1 = min(i0 + 1, nx - 1)
    j1 = min(j0 + 1, ny - 1)
    tx = fi - i0
    ty = fj - j0
    v00 = grid["values"][j0][i0]
    v10 = grid["values"][j0][i1]
    v01 = grid["values"][j1][i0]
    v11 = grid["values"][j1][i1]
    return (v00 * (1 - tx) * (1 - ty) + v10 * tx * (1 - ty)
            + v01 * (1 - tx) * ty + v11 * tx * ty)



def _crossing(value_a, value_b, level):
    denom = value_b - value_a
    if denom == 0.0:
        return 0.5
    t = (level - value_a) / denom
    return 0.0 if t < 0.0 else 1.0 if t > 1.0 else t


def split_ring_at_points(ring, points, tol=1e-7):
    """Insert ring vertices at every location where a contour meets the ring.

    The kernel audits constrained edges exactly: a contour ending part-way
    along a boundary segment produces a T-junction the TIN cannot hold.  After
    this split the boundary carries a vertex at the meeting point, so the
    contour endpoint and the boundary share one welded vertex.

    Returns (new_ring_without_final_duplicate, count_inserted).
    """
    if len(ring) < 3:
        return list(ring), 0
    open_ring = list(ring)
    if len(open_ring) >= 2 and open_ring[0] == open_ring[-1]:
        open_ring.pop()
    n = len(open_ring)
    inserts = [[] for _ in range(n)]  # per ring segment, (t, point)

    for p in points:
        px, py = p[0], p[1]
        # Skip a point that already coincides with a ring vertex.
        on_vertex = any(math.hypot(px - v[0], py - v[1]) <= tol for v in open_ring)
        if on_vertex:
            continue
        best = None
        for i in range(n):
            x1, y1 = open_ring[i]
            x2, y2 = open_ring[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            length2 = dx * dx + dy * dy
            if length2 == 0.0:
                continue
            t = ((px - x1) * dx + (py - y1) * dy) / length2
            if t < -1e-9 or t > 1 + 1e-9:
                continue
            qx, qy = x1 + t * dx, y1 + t * dy
            d = math.hypot(px - qx, py - qy)
            if d <= tol and (best is None or d < best[0]):
                best = (d, i, min(1.0, max(0.0, t)))
        if best is not None:
            inserts[best[1]].append((best[2], [px, py]))

    out = []
    inserted = 0
    for i in range(n):
        out.append(list(open_ring[i]))
        for _, p in sorted(inserts[i], key=lambda item: item[0]):
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > tol:
                out.append(p)
                inserted += 1
    return out, inserted


def point_in_or_on(x, y, ring, tol=1e-9):
    if point_in_polygon(x, y, ring):
        return True
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        length2 = dx * dx + dy * dy
        if length2 == 0.0:
            continue
        t = ((x - x1) * dx + (y - y1) * dy) / length2
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
        px, py = x1 + t * dx, y1 + t * dy
        if math.hypot(x - px, y - py) <= tol:
            return True
    return False


def _segment_intersection_t(a, b, c, d):
    """Return t in (0,1) where segment a-b crosses c-d, or None."""
    rx, ry = b[0] - a[0], b[1] - a[1]
    sx, sy = d[0] - c[0], d[1] - c[1]
    denom = rx * sy - ry * sx
    if abs(denom) < 1e-15:
        return None
    qx, qy = c[0] - a[0], c[1] - a[1]
    t = (qx * sy - qy * sx) / denom
    u = (qx * ry - qy * rx) / denom
    if -1e-12 <= t <= 1 + 1e-12 and -1e-12 <= u <= 1 + 1e-12:
        return min(1.0, max(0.0, t))
    return None


def _inside_domain(x, y, boundary, holes):
    if not point_in_or_on(x, y, boundary):
        return False
    for hole in holes:
        if point_in_polygon(x, y, hole):
            return False
    return True


def clip_to_domain(points, closed, boundary, holes, tol=1e-6):
    """Clip a stitched polyline against the boundary and holes.

    Returns a list of polylines.  Intersections are computed on the original
    segment, so an open contour ends exactly on the boundary instead of one
    grid cell short.  A contour that runs through a hole is split into
    several pieces, which is the broken-contour case.
    """
    if len(points) < 2:
        return []
    rings = [boundary] + list(holes)
    n = len(points)
    segment_count = n if closed else n - 1

    pieces = []
    current = []
    for si in range(segment_count):
        a = points[si]
        b = points[(si + 1) % n]
        cuts = [0.0, 1.0]
        for ring in rings:
            m = len(ring)
            for k in range(m):
                t = _segment_intersection_t(a, b, ring[k], ring[(k + 1) % m])
                if t is not None and 1e-12 < t < 1 - 1e-12:
                    cuts.append(t)
        cuts = sorted(set(round(t, 12) for t in cuts))
        for q in range(len(cuts) - 1):
            t0, t1 = cuts[q], cuts[q + 1]
            if t1 - t0 < 1e-12:
                continue
            p0 = (a[0] + (b[0] - a[0]) * t0, a[1] + (b[1] - a[1]) * t0)
            p1 = (a[0] + (b[0] - a[0]) * t1, a[1] + (b[1] - a[1]) * t1)
            mid = (0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]))
            if not _inside_domain(mid[0], mid[1], boundary, holes):
                if current:
                    pieces.append(current)
                    current = []
                continue
            if not current:
                current = [p0]
            elif math.hypot(current[-1][0] - p0[0], current[-1][1] - p0[1]) > tol:
                pieces.append(current)
                current = [p0]
            current.append(p1)
    if current:
        pieces.append(current)

    result = []
    for piece in pieces:
        # Remove consecutive duplicates produced by cut points at region edges.
        deduped = [piece[0]]
        for p in piece[1:]:
            if math.hypot(p[0] - deduped[-1][0], p[1] - deduped[-1][1]) > tol:
                deduped.append(p)
        if len(deduped) < 2:
            continue
        is_closed = closed and math.hypot(deduped[0][0] - deduped[-1][0],
                                          deduped[0][1] - deduped[-1][1]) <= tol and len(deduped) >= 4
        result.append({"points": deduped, "closed": is_closed})
    return result


def extract_level_from_grid(grid, boundary, level, holes=()):
    """Return polylines (list of [x,y]) following one level of a parsed raster.

    Cells whose centre is outside the domain are skipped; stitched lines are
    then clipped exactly against boundary and holes, so boundary-truncated
    contours end on the ring and a line crossing a hole becomes broken pieces.
    """
    holes = list(holes)
    xs = grid["xs"]
    ys = grid["ys"]
    nx, ny = grid["nx"], grid["ny"]
    values = grid["values"]

    points = {}

    def hpoint(i, j):  # edge from (i,j) to (i+1,j)
        key = (H, i, j)
        if key not in points:
            t = _crossing(values[j][i], values[j][i + 1], level)
            points[key] = (xs[i] + (xs[i + 1] - xs[i]) * t, ys[j])
        return key

    def vpoint(i, j):  # edge from (i,j) to (i,j+1)
        key = (V, i, j)
        if key not in points:
            t = _crossing(values[j][i], values[j + 1][i], level)
            points[key] = (xs[i], ys[j] + (ys[j + 1] - ys[j]) * t)
        return key

    edges = []

    for j in range(ny - 1):
        for i in range(nx - 1):
            cx = 0.5 * (xs[i] + xs[i + 1])
            cy = 0.5 * (ys[j] + ys[j + 1])
            if not _inside_domain(cx, cy, boundary, holes):
                continue
            bl = values[j][i]
            br = values[j][i + 1]
            tr = values[j + 1][i + 1]
            tl = values[j + 1][i]
            case = (1 if bl >= level else 0) | (2 if br >= level else 0) \
                 | (4 if tr >= level else 0) | (8 if tl >= level else 0)
            if case == 0 or case == 15:
                continue
            bottom = hpoint(i, j)
            right = vpoint(i + 1, j)
            top = hpoint(i, j + 1)
            left = vpoint(i, j)
            center = 0.25 * (bl + br + tr + tl)
            if case == 1:
                edges.append((left, bottom))
            elif case == 2:
                edges.append((bottom, right))
            elif case == 3:
                edges.append((left, right))
            elif case == 4:
                edges.append((right, top))
            elif case == 5:
                if center >= level:
                    edges.append((bottom, right))
                    edges.append((top, left))
                else:
                    edges.append((left, bottom))
                    edges.append((right, top))
            elif case == 6:
                edges.append((bottom, top))
            elif case == 7:
                edges.append((left, top))
            elif case == 8:
                edges.append((top, left))
            elif case == 9:
                edges.append((bottom, top))
            elif case == 10:
                if center >= level:
                    edges.append((left, bottom))
                    edges.append((right, top))
                else:
                    edges.append((bottom, right))
                    edges.append((top, left))
            elif case == 11:
                edges.append((right, top))
            elif case == 12:
                edges.append((right, left))
            elif case == 13:
                edges.append((bottom, right))
            elif case == 14:
                edges.append((bottom, left))

    adjacency = {}
    for index, (a, b) in enumerate(edges):
        adjacency.setdefault(a, []).append(index)
        adjacency.setdefault(b, []).append(index)

    used = [False] * len(edges)

    def walk_from(start, first_edge):
        coordinates = []
        used[first_edge] = True
        a, b = edges[first_edge]
        prev, cur = (a, b) if a == start else (b, a)
        coordinates.append(points[prev])
        coordinates.append(points[cur])
        while True:
            candidates = [k for k in adjacency.get(cur, []) if not used[k]]
            if not candidates:
                break
            nxt_edge = candidates[0]
            used[nxt_edge] = True
            na, nb = edges[nxt_edge]
            nxt = nb if na == cur else na
            coordinates.append(points[nxt])
            prev, cur = cur, nxt
        return coordinates

    polylines = []
    for vertex, incident in adjacency.items():
        if len(incident) != 1 or used[incident[0]]:
            continue
        polylines.append({"points": walk_from(vertex, incident[0]), "closed": False})
    for index in range(len(edges)):
        if used[index]:
            continue
        a, _ = edges[index]
        coordinates = walk_from(a, index)
        if coordinates and coordinates[0] != coordinates[-1]:
            coordinates.append(coordinates[0])
        polylines.append({"points": coordinates, "closed": True})

    clipped = []
    for line in polylines:
        for piece in clip_to_domain(line["points"], line["closed"], boundary, holes):
            clipped.append(piece)

    # Coordinates are left unrounded: rounding would move a boundary crossing
    # off the ring by up to a rounding unit, and the kernel's constraint audit
    # requires the contour endpoint and the ring to share an exact vertex.
    # Vertices shared by adjacent cells are identical because they come from
    # the same crossing key, so no decimal snapping is needed for welding.
    for line in clipped:
        # A closed contour is stored without a repeated final point: the
        # kernel rejects the duplicate closing vertex as a self-intersection.
        if (line["closed"] and len(line["points"]) >= 2
                and line["points"][0] == line["points"][-1]):
            line["points"].pop()
    return clipped


def extract_level(surface, bounds, boundary, level, step, holes=()):
    """Sample the analytic surface into a raster and extract one level from it."""
    grid = sample_heightfield(surface, bounds, step)
    return extract_level_from_grid(grid, boundary, level, holes)


def polyline_distance(a, b):
    best = math.inf
    for p in a:
        for q in b:
            d = math.hypot(p[0] - q[0], p[1] - q[1])
            if d < best:
                best = d
    return best


def measured_level_error(surface, level, points):
    """Max |z(point) - level| over polyline vertices.

    Marching-squares vertices are exact only along the cell edge; this reports
    the analytic error introduced by linear interpolation inside the cell.
    """
    worst = 0.0
    for x, y in points:
        worst = max(worst, abs(surface_elevation(surface, x, y) - level))
    return worst
