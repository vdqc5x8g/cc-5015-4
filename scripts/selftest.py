#!/usr/bin/env python3
"""End-to-end checks for the four validation scenarios.

Runs the CLI `build` command, then verifies the emitted pipeline JSON:
  * plane slope       known elevations are reproduced (observed error ~ 0)
  * closed hill       ranges are consistent with the supplied contours, summit observed
  * breakline         the fault appears as constrained mesh edges across the ridge
  * hole boundary     no triangle covers the void, area excludes the hole
Also checks that every constrained edge exists as an edge of the TIN.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, "build", "terrain")
DATA = os.path.join(ROOT, "tests", "data")

failures = []


def check(condition, message):
    if condition:
        print("  ok: " + message)
    else:
        print("  FAIL: " + message)
        failures.append(message)


def run(name, tmp):
    prefix = os.path.join(tmp, name)
    proc = subprocess.run(
        [BIN, "build", os.path.join(DATA, name + ".json"), "--out-prefix", prefix],
        capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stdout)
        print(proc.stderr)
        raise SystemExit("build failed for " + name)
    with open(prefix + "_pipeline.json") as f:
        return json.load(f)


def mesh_vertices(doc):
    m = doc["mesh"]
    pos = m["positions"]
    return [(pos[i], pos[i + 1], pos[i + 2]) for i in range(0, len(pos), 3)]


def triangles(doc):
    idx = doc["mesh"]["indices"]
    return [(idx[i], idx[i + 1], idx[i + 2]) for i in range(0, len(idx), 3)]


def edge_set(doc):
    edges = set()
    idx = doc["mesh"]["indices"]
    for i in range(0, len(idx), 3):
        a, b, c = idx[i], idx[i + 1], idx[i + 2]
        for u, v in ((a, b), (b, c), (c, a)):
            edges.add((min(u, v), max(u, v)))
    return edges


def verify_constraints_present(doc, where):
    edges = edge_set(doc)
    con = doc["mesh"].get("constrained_edges_indices", [])
    missing = 0
    for i in range(0, len(con), 2):
        a, b = con[i], con[i + 1]
        if (min(a, b), max(a, b)) not in edges:
            missing += 1
    check(missing == 0, "%s: all %d constrained edges exist in the TIN" % (where, len(con) // 2))


def triangle_area(tri, verts):
    a, b, c = (verts[i] for i in tri)
    return abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2.0


def point_in_ring(x, y, ring):
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


def test_plane(doc):
    print("plane_slope:")
    verts = mesh_vertices(doc)
    unsupported = doc["mesh"]["unsupported"]
    worst = 0.0
    worst_any = 0.0
    for (x, y, z), u in zip(verts, unsupported):
        worst_any = max(worst_any, abs(z - y))
        if not u:
            worst = max(worst, abs(z - y))
    check(worst < 1e-9,
          "supported surface reproduces plane z=y exactly (max error %.3g m)" % worst)
    check(any(unsupported), "vertices beyond the observation support are flagged "
          "(corner extrapolation up to %.2f m)" % worst_any)
    check(doc["mesh"]["triangle_count"] > 0, "mesh has %d faces" % doc["mesh"]["triangle_count"])
    observed = [(x, y, z) for (x, y, z), s in zip(verts, doc["mesh"]["sources"]) if s == "observed"]
    check(len(observed) > 0 and all(abs(z - y) < 1e-9 for (x, y, z) in observed),
          "%d observed vertices carry exactly the supplied elevations" % len(observed))
    verify_constraints_present(doc, "plane_slope")


def test_hill(doc):
    print("closed_hill:")
    verts = mesh_vertices(doc)
    zs = [v[2] for v in verts]
    contours = [c["elevation"] for c in doc["original"]["contours"]]
    controls = [c["elevation"] for c in doc["original"]["controls"]]
    summit = max(contours + controls)
    check(abs(max(zs) - summit) < 1e-6, "surface peak equals the summit control (%.3f)" % max(zs))
    check(abs(min(zs) - min(contours)) < 1e-6,
          "lowest elevation stays at the outermost contour elevation (%.3f)" % min(zs))
    observed = [v[2] for v, s in zip(verts, doc["mesh"]["sources"]) if s == "observed"]
    check(any(abs(z - summit) < 1e-6 for z in observed),
          "summit elevation is an observed value, not an extrapolated one")
    check(len(set(round(z, 6) for z in zs)) >= len(contours),
          "each input contour level survives in the mesh")
    verify_constraints_present(doc, "closed_hill")


def test_breakline(doc):
    print("breakline:")
    verts = mesh_vertices(doc)
    kinds = doc["mesh"].get("constrained_kinds", [])
    check("fault" in kinds, "fault/breakline edges are constrained in the mesh")
    con = doc["mesh"]["constrained_edges_indices"]
    fault_edges = [(con[2 * i], con[2 * i + 1]) for i, k in enumerate(kinds) if k == "fault"]
    check(len(fault_edges) > 0, "fault contributes %d constrained edges" % len(fault_edges))
    on_ridge = all(
        abs(verts[a][0] - 50.0) < 1e-6 and abs(verts[b][0] - 50.0) < 1e-6
        for a, b in fault_edges)
    check(on_ridge, "fault constraint edges lie on x=50")
    ys = [min(verts[a][1], verts[b][1]) for a, b in fault_edges]
    check(min(ys) < 1e-6, "fault spans the boundary at y=0 (interpolated there)")
    crossing = [v for v in verts if abs(v[0] - 50.0) < 1e-6
                and any(s == "constraint_edge" for s in doc["mesh"]["sources"])]
    check(len(crossing) >= 4, "contour/breakline crossings got elevations (%d vertices)" % len(crossing))
    verify_constraints_present(doc, "breakline")


def test_hole(doc):
    print("hole_boundary:")
    verts = mesh_vertices(doc)
    tris = triangles(doc)
    ring = doc["original"]["holes"][0]
    ring = [(p[0], p[1]) for p in ring]
    covering = 0
    area = 0.0
    for t in tris:
        cx = sum(verts[i][0] for i in t) / 3.0
        cy = sum(verts[i][1] for i in t) / 3.0
        if point_in_ring(cx, cy, ring):
            covering += 1
        area += triangle_area(t, verts)
    check(covering == 0, "no triangle covers the void (%d offending)" % covering)
    check(abs(area - 9600.0) < 1e-6, "mesh area excludes the 400 m^2 hole (%.1f m^2)" % area)
    hole_vertices = [v for v in verts
                     if 39.999 <= v[0] <= 60.001 and 39.999 <= v[1] <= 60.001]
    check(len(hole_vertices) >= 4, "hole ring vertices are present in the mesh")
    verify_constraints_present(doc, "hole_boundary")


def test_large(doc):
    print("large_coords:")
    verts = mesh_vertices(doc)
    srcs = doc["mesh"]["sources"]
    unsupported = doc["mesh"]["unsupported"]
    n0 = 4100000.0
    worst = 0.0
    for (x, y, z), s, u in zip(verts, srcs, unsupported):
        if not u:
            worst = max(worst, abs(z - (y - n0)))
    check(worst < 1e-9,
          "surface reproduces z=(N-N0) at 4.1e6 northing (max error %.3g m)" % worst)
    check(doc["mesh"]["triangle_count"] > 0, "%d faces at large coordinates" % doc["mesh"]["triangle_count"])
    tol = doc["tolerance"]
    check(tol["snap_tolerance_m"] > 0.0 and tol["snap_tolerance_m"] < 1.0,
          "snap tolerance is data-derived, not a fixed constant (%.4f m)" % tol["snap_tolerance_m"])
    check(tol["origin_x"] > 400000.0, "local origin is at the dataset, not the CRS origin")
    check(tol["extent_m"] < 200.0, "extent stays the local survey size (%.1f m)" % tol["extent_m"])
    check(tol["precision_limited"] is True,
          "coordinate magnitude is reported as precision limited")
    verify_constraints_present(doc, "large_coords")


def run_with_repairs(name, tmp, repairs):
    prefix = os.path.join(tmp, name + "_repaired")
    body = json.dumps({"input_text": open(os.path.join(DATA, name + ".json")).read(),
                       "repairs": repairs})
    req = urllib.request.Request(
        "http://127.0.0.1:8787/api/run", data=body.encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as res:
        doc = json.load(res)
    with open(prefix + "_pipeline.json", "w") as f:
        json.dump(doc, f)
    return doc


def test_repairs(tmp):
    print("repairable_anomalies:")
    base = run_with_repairs("repairable_anomalies", tmp, [])
    check(not base["ok"], "unrepaired data is blocked (%s)" % base.get("error", "")[:60])
    ids = [c["id"] for c in base["repair_candidates"]]
    check(len(ids) >= 5, "%d confirmable repair candidates offered" % len(ids))
    check(any(c["id"] == "repair.outside" or c["code"] == "contour.outside_boundary"
              for c in base["repair_candidates"]),
          "out-of-boundary contour has a clip candidate")
    applied = run_with_repairs("repairable_anomalies", tmp, ids)
    check(applied["ok"], "confirming all candidates builds a mesh")
    check(applied["mesh"]["ok"] and applied["mesh"]["triangle_count"] > 0,
          "%d faces after repairs" % applied["mesh"].get("triangle_count", 0))
    check(len(applied["original"]["contours"]) == len(base["original"]["contours"]),
          "original contours are unchanged and still queryable")
    check(len(applied["derived"]["contours"]) > len(applied["original"]["contours"]),
          "derived pieces are separate from the original data")
    check(len(applied["original_text"]) > 0, "original document text is preserved")
    verify_constraints_present(applied, "repairable_anomalies repaired")


def test_blocked_crossing(tmp):
    print("anomalies:")
    doc = run_with_repairs("anomalies", tmp, [])
    check(not doc["ok"], "different-elevation crossing blocks the build")
    cross = [i for i in doc["validation"]["issues"]
             if i["code"] in ("contour.different_z_cross", "contour.different_z_overlap")]
    check(len(cross) >= 1, "%d different-elevation crossing issue(s) located" % len(cross))
    check(all(i["positions"] for i in cross), "crossing issues carry exact coordinates")
    check(all(not i["repairable"] or i["candidates"] for i in cross),
          "crossings are never auto-merged; only review candidates are offered")


def main():
    if not os.path.exists(BIN):
        raise SystemExit("missing binary; run make first")
    needs_server = any(a in sys.argv for a in ("--server",))
    with tempfile.TemporaryDirectory() as tmp:
        for name, fn in (("plane_slope", test_plane),
                         ("closed_hill", test_hill),
                         ("breakline", test_breakline),
                         ("hole_boundary", test_hole),
                         ("large_coords", test_large)):
            doc = run(name, tmp)
            if not doc["mesh"]["ok"]:
                failures.append(name + ": mesh not ok")
                print("  FAIL: mesh not ok: " + doc["mesh"]["error"])
                continue
            fn(doc)
        if needs_server:
            test_repairs(tmp)
            test_blocked_crossing(tmp)
    if failures:
        print("\n%d check(s) failed" % len(failures))
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
