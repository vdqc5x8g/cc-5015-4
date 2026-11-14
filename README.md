# Contour → constrained TIN

A production tool that turns synthetic contour lines (plus control points, a
boundary and optional holes and breaklines) into a **real constrained
triangulated terrain mesh**. The browser view shows the input lines, their
elevations, and the triangulated surface *simultaneously*; the surface is
WebGL geometry, not a contour texture.

## Build and run

```sh
make                      # builds build/terrain
./build/terrain serve 8787
# open http://127.0.0.1:8787/
```

Requires `g++` with C++17 and Python 3 for the self-test. The triangulation
library (CDT 2.0.2), Three.js and nlohmann/json are vendored in `third_party/`;
no network access is needed at run time.

CLI:

```sh
./build/terrain validate tests/data/closed_hill.json
./build/terrain build    tests/data/closed_hill.json --out-prefix out/hill
./build/terrain build    tests/data/repairable_anomalies.json --out-prefix out/rep
# repairs are confirmed by the UI; from the CLI they are reported, not applied
./build/terrain serve 8787 [webroot]
```

`build` writes `<prefix>.obj`, `<prefix>.stl` (binary), `<prefix>_vertices.csv`,
`<prefix>_triangles.csv`, `<prefix>_mesh.json` and
`<prefix>_pipeline.json` (original + derived + validation + mesh). It prints
face and vertex counts, the triangulation / elevation / total timings, and the
elevation-source breakdown.

## Input format

```json
{
  "name": "example",
  "meta": {
    "xy_unit": "m", "xy_meters_per_unit": 1.0,
    "z_unit": "m", "z_meters_per_unit": 1.0,
    "z_datum": "local benchmark",
    "coordinate_note": "optional free text"
  },
  "boundary": [[0,0],[100,0],[100,100],[0,100],[0,0]],
  "holes": [ [[40,40],[60,40],[60,60],[40,60],[40,40]] ],
  "contours": [
    { "id": 1, "name": "z=10", "closed": false, "elevation": 10,
      "points": [[0,10],[20,10],[40,10]] }
  ],
  "controls": [ { "id": 1, "name": "summit", "position": [50,50], "elevation": 50 } ],
  "faults":   [ { "id": 1, "name": "ridge", "points": [[50,0],[50,100]] } ]
}
```

* **XY unit** is declared per dataset (`xy_unit` + `xy_meters_per_unit`). All
  positions, tolerances and exports are in that unit; areas are reported in m².
* **Elevation unit and datum** are declared separately (`z_unit`,
  `z_meters_per_unit`, `z_datum`). Elevations are never rescaled silently: the
  datum string is shown in the UI and copied into the exports.
* `closed: true` treats the contour as a loop; a repeated last point is
  accepted for both contours and rings.
* `faults` are hard constraints with **no** elevation of their own; their
  vertices get a harmonic value.
* `controls` are fixed observations.

## Numerical stability with large coordinates

Coordinates are translated to the dataset centre and divided by the extent
before triangulating. The CDT always sees roughly `[-0.5, 0.5]` regardless of
whether the survey is at 0 m or 4 100 000 m northing, while local detail
survives because the transform is exact in double precision.

Tolerances are **derived from the data**, never fixed constants:

| quantity | derivation |
| --- | --- |
| precision floor | `max(4 ulp(maxAbsCoord), extent / 2^(53-8))` |
| point spacing | median / 5th-percentile nearest-neighbour distance (duplicates excluded) |
| snap (weld) radius | `max(precision floor, 0.001 × P05 spacing)`, capped at `0.001 × extent` |
| short-segment threshold | `0.15 × P05 spacing`, floored by the precision floor |
| elevation equality | `0.25 × smallest gap between contour levels` |
| user tolerance | accepted, but raised to the precision floor if finer than double can resolve |

`tests/data/large_coords.json` places the plane-slope survey at easting
500 000 / northing 4 100 000. The observer reproduces the known plane exactly,
the snap radius is 5 mm (derived from the 5 m point spacing) and the run is
flagged `precision_limited`. The CLI prints the full derivation.

## What is guaranteed

* **Intersections are never force-merged.** Two constrained lines that cross
  or overlap with elevations differing by more than the elevation tolerance
  are a blocking error with exact coordinates. No candidate merges them; the
  user must edit, split or drop a line. Vertices weld only when they are within
  the snap radius **and** their elevations agree within tolerance; the same
  position with a different elevation stays two vertices.
* **Constraint preservation is audited.** Every constrained edge is looked up
  in the finished TIN; if any is missing the build fails with the count instead
  of emitting a subtly wrong mesh.
* **Holes stay empty.** Hole rings are inserted as constraints and triangles
  whose centroid falls inside a hole are removed. No hole is ever filled.
* **Known elevations are fixed.** Contour vertices and control points carry
  their supplied elevations unchanged (`source: observed`).

## Validation and repairs

`validateInput` reports, with positions for every hit:

* boundary: open ring, degenerate area, self-intersection;
* holes: too few points, self-intersection, outside the boundary, nesting;
* contours: duplicate consecutive vertices, short segments, self-intersection
  or T-junction, leaving the boundary, vertices near a different-elevation line,
  identical duplicates and embedded (sub-line) duplicates;
* cross-contour: different-elevation crossing / overlap (blocking, never merged);
* control points: outside the boundary, inside a hole, duplicates,
  same position with conflicting elevations;
* breaklines: too few points, self-intersection.

Repair candidates are suggestions: split at self-touch points, clip to the
domain, merge duplicate/short-inside runs, close the boundary, drop a duplicate
or embedded contour, exclude an out-of-boundary control. Nothing is applied
until the user ticks a candidate and rebuilds. `runPipeline` copies the input;
`ContourInput::originalText` and `DataInput::sourceDocument` keep the verbatim
source. The UI draws applied repairs as solid lines and the untouched original
as dashed grey.

## Interpolation

Two passes, both documented in the UI:

1. **Along constraints.** A vertex created where two constraints meet (a
   contour cut by a breakline, for instance) is not an observation. Its
   elevation is linearly interpolated **in arc length** along the contour
   between the nearest supplied vertices. Vertices on the same contour that
   carry no supplied elevation are filled the same way. Breakline vertices
   that are not contour crossings are left unknown on purpose.
2. **Discrete harmonic (Laplace) solve.** Every remaining unknown vertex is the
   average of its TIN neighbours, with all observed values held fixed (Dirichlet
   conditions). The system is solved by successive over-relaxation
   (ω = 1.7) to a residual of `max(0.005 × zTolerance, 1e-13)`.

**Uncovered areas.** Each vertex records its distance to the nearest
observation. A support radius is derived from the median contour-to-contour
spacing (60 % of it). A vertex is `unsupported` when it is farther than that
radius, or when it lies inside a closed contour that encloses an extremum
(an un-nested innermost ring — there is no data above the summit or below the
depression). Triangles with at least one unsupported vertex are counted, given
an area in m² and a percentage, and drawn in amber. The coverage note in the
UI states the radius, the fraction and the farthest distance. Components with
no observation at all are marked `unobserved`.

## Rendering

WebGL via Three.js. The surface mesh is real triangle geometry with per-vertex
elevation colouring. Contour lines, breaklines and boundary/hole rings are
draped onto the interpolated surface by sampling the TIN, so they are not
hidden inside the terrain. Constrained edges come from the *triangulation's*
own edges and are highlighted by kind (boundary, hole, contour, fault).
Elevation labels, control points, issue markers and hit-testing all use original
coordinates; a toggle applies vertical exaggeration for display only (the mesh
and exports keep real units). Timings (triangulation, elevation, mesh total,
pipeline total) and face/vertex counts are shown next to the viewport.

## Verification

```sh
python3 scripts/selftest.py            # CLI checks on five samples
python3 scripts/selftest.py --server   # also exercises repairs and crossings
```

Checks performed:

| sample | assertion |
| --- | --- |
| `plane_slope` | supported surface reproduces `z = y` exactly; observed vertices exact; corners beyond support flagged |
| `closed_hill` | peak equals the summit control (observed, not extrapolated); lowest = outer ring; all contour levels survive |
| `breakline` | fault edges are constrained, lie on x = 50, span the boundary; contour/breakline crossings get arc-length elevations |
| `hole_boundary` | no triangle inside the void; area is exactly 9600 m² of 10 000 |
| `large_coords` | exact plane at 4.1e6 northing; derived tolerance; precision flag |
| `repairable_anomalies` | raw data blocked; every candidate listable; confirming all produces a valid mesh; original data unchanged |
| `anomalies` | different-elevation crossings block the build, are located, and are never auto-merged |

Every sample also asserts that all constrained edges reported by the mesh
actually exist in the triangle set.

The UI was exercised in headless Chromium with SwiftShader WebGL: the mesh
geometry, overlays, validation markers, repair confirmation, rebuild and export
paths were driven through the real interface, not mocked.
