#include "terrain/validate.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace terrain {

namespace {

std::string num(double v, int digits = 6)
{
    std::ostringstream os;
    os.precision(digits);
    os << v;
    return os.str();
}

void pushIssue(ValidationReport& report, ValidationIssue issue)
{
    if (issue.severity == "error")
        ++report.errorCount;
    else if (issue.severity == "warning")
        ++report.warningCount;
    else
        ++report.infoCount;
    report.issues.push_back(std::move(issue));
}

std::vector<double> allContourElevations(const DataInput& input)
{
    std::vector<double> out;
    for (const ContourInput& c : input.contours)
        out.push_back(c.elevation);
    return out;
}

std::vector<double> allControlElevations(const DataInput& input)
{
    std::vector<double> out;
    for (const ControlPoint& c : input.controls)
        out.push_back(c.elevation);
    return out;
}

// Build a lookup of every segment for between-contour checks.
struct SegmentRef {
    int contourId;
    int segmentIndex;
    Vec2 a;
    Vec2 b;
    double elevation;
};

// Snaps a point to an existing vertex only when the geometric distance is
// within the data-derived tolerance AND the elevation difference is within the
// elevation-equality tolerance. This is the guard that prevents crossing
// contours at different elevations from being merged.
struct VertexRegistry {
    struct Entry {
        Vec2 p;
        double elevation;
        int contourId;
        bool isControl;
    };
    std::vector<Entry> entries;
    Tolerance tol;
    int mergesAccepted = 0;
    int mergesRejectedByElevation = 0;

    bool find(const Vec2& p, double z, int* outIndex) const
    {
        bool found = false;
        double best = tol.snapTolUnits;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const double d = distance(p, entries[i].p);
            if (d > best)
                continue;
            const bool sameZ = std::abs(z - entries[i].elevation) <= tol.zTolUnits;
            if (!sameZ)
                continue;
            best = d;
            found = true;
            if (outIndex)
                *outIndex = static_cast<int>(i);
        }
        return found;
    }

    bool hasNearDifferentElevation(const Vec2& p, double z) const
    {
        for (const Entry& e : entries) {
            if (distance(p, e.p) <= tol.snapTolUnits && std::abs(z - e.elevation) > tol.zTolUnits)
                return true;
        }
        return false;
    }
};

} // namespace

ValidationReport validateInput(const DataInput& input, const Tolerance& tol)
{
    ValidationReport report;
    const double snap = tol.snapTolUnits;

    // ---- metadata and boundary ----
    if (input.meta.xyUnitName.empty())
        pushIssue(report, {"meta.xy_unit_missing", "warning", "XY unit is not declared; assume the values are already in metres.", "meta", -1, -1, {}, false, {}});
    if (input.meta.zDatum.empty())
        pushIssue(report, {"meta.z_datum_missing", "warning", "Elevation datum is not declared; results are relative unless a datum is recorded.", "meta", -1, -1, {}, false, {}});
    if (input.meta.xyMetersPerUnit <= 0.0 || input.meta.zMetersPerUnit <= 0.0)
        pushIssue(report, {"meta.unit_scale_invalid", "error", "Unit conversion factors must be positive.", "meta", -1, -1, {}, false, {}});

    Ring outer = dedupeConsecutive(input.outer, snap);
    const bool outerClosedInput = !input.outer.empty() && distance(input.outer.front(), input.outer.back()) <= snap;
    if (outer.size() >= 2 && distance(outer.front(), outer.back()) <= snap)
        outer.pop_back();
    if (outer.size() < 3) {
        pushIssue(report, {"boundary.too_few_points", "error", "Boundary must have at least three distinct vertices.", "boundary", -1, -1, input.outer, false, {}});
    } else {
        if (!outerClosedInput) {
            pushIssue(report, {"boundary.open", "warning",
                               "Boundary ring is open; it will be closed with a straight segment from the last to the first vertex.",
                               "boundary", -1, -1, {outer.front(), outer.back()}, true, {"repair.boundary.close"}});
            report.candidates.push_back({"repair.boundary.close", "boundary.open",
                                        "Close boundary ring", "Add a closing segment from the last vertex to the first.",
                                        RepairKind::CloseBoundary, -1, -1, {}, true});
        }
        if (std::abs(ringSignedArea(outer)) < snap * snap)
            pushIssue(report, {"boundary.degenerate", "error", "Boundary encloses (almost) zero area.", "boundary", -1, -1, outer, false, {}});
        const std::vector<SelfHit> selfHits = findSelfIntersections(outer, true, snap);
        for (const SelfHit& h : selfHits) {
            pushIssue(report, {"boundary.self_intersection", "error",
                               std::string(h.overlap ? "Boundary has overlapping segments" : "Boundary self-intersects") +
                                   " near (" + num(h.point.x) + ", " + num(h.point.y) + ").",
                               "boundary", -1, -1, {h.point}, false, {}});
        }
    }

    // ---- holes ----
    for (std::size_t hi = 0; hi < input.holes.size(); ++hi) {
        Ring hole = dedupeConsecutive(input.holes[hi], snap);
        if (hole.size() >= 2 && distance(hole.front(), hole.back()) <= snap)
            hole.pop_back();
        if (hole.size() < 3) {
            pushIssue(report, {"hole.too_few_points", "error", "Hole #" + std::to_string(hi + 1) + " has fewer than three distinct vertices.", "hole", -1, static_cast<int>(hi), input.holes[hi], false, {}});
            continue;
        }
        const std::vector<SelfHit> selfHits = findSelfIntersections(hole, true, snap);
        for (const SelfHit& h : selfHits)
            pushIssue(report, {"hole.self_intersection", "error", "Hole #" + std::to_string(hi + 1) + " self-intersects near (" + num(h.point.x) + ", " + num(h.point.y) + ").", "hole", -1, static_cast<int>(hi), {h.point}, false, {}});
        bool anyOutside = false;
        for (const Vec2& p : hole)
            if (classifyPoint(p, outer, {}, snap) == DomainLoc::Outside)
                anyOutside = true;
        if (anyOutside)
            pushIssue(report, {"hole.outside_boundary", "warning", "Hole #" + std::to_string(hi + 1) + " lies partly outside the boundary; regions outside the boundary are not triangulated.", "hole", -1, static_cast<int>(hi), hole, false, {}});
    }
    // Hole nesting: a hole inside another hole is reported because it reopens
    // the domain in ways most survey data does not intend.
    for (std::size_t i = 0; i < input.holes.size(); ++i) {
        for (std::size_t j = 0; j < input.holes.size(); ++j) {
            if (i == j)
                continue;
            const Ring& hi = input.holes[i];
            const Ring& hj = input.holes[j];
            if (hi.empty() || hj.empty())
                continue;
            bool allInside = true;
            for (const Vec2& p : hi) {
                if (classifyPoint(p, hj, {}, snap) != DomainLoc::Inside) {
                    allInside = false;
                    break;
                }
            }
            if (allInside) {
                pushIssue(report, {"hole.nested", "error", "Hole #" + std::to_string(i + 1) + " is nested inside hole #" + std::to_string(j + 1) + "; nesting is not supported.", "hole", -1, static_cast<int>(i), hi, false, {}});
            }
        }
    }

    // ---- contour-level checks ----
    VertexRegistry registry;
    registry.tol = tol;
    std::vector<SegmentRef> allSegments;
    for (const ContourInput& c : input.contours) {
        const std::size_t n = c.points.size();
        const std::size_t segCount = (c.closed && n > 1) ? n : (n > 1 ? n - 1 : 0);
        for (std::size_t i = 0; i < segCount; ++i) {
            Vec2 a = c.points[i];
            Vec2 b = c.points[(i + 1) % n];
            allSegments.push_back({c.id, static_cast<int>(i), a, b, c.elevation});
            registry.entries.push_back({a, c.elevation, c.id, false});
            if (i + 1 == segCount)
                registry.entries.push_back({b, c.elevation, c.id, false});
        }
        for (const ControlPoint& cp : input.controls)
            registry.entries.push_back({cp.position, cp.elevation, -1, true});
    }

    std::map<int, std::vector<Polyline>> candidateReplacements;
    std::map<int, std::vector<std::string>> repairability;

    for (const ContourInput& c : input.contours) {
        if (c.points.size() < 2) {
            pushIssue(report, {"contour.too_few_points", "error", "Contour '" + c.name + "' has fewer than two vertices.", "contour", c.id, -1, c.points, true, {}});
            report.candidates.push_back({"repair.drop.c" + std::to_string(c.id), "contour.too_few_points",
                                        "Drop contour " + std::to_string(c.id), "The contour cannot form a segment.", RepairKind::DropContour, c.id, -1, {}, false});
            continue;
        }

        bool closedHit = 0;
        Polyline deduped = dedupeConsecutive(c.points, snap, &closedHit);
        if (closedHit) {
            pushIssue(report, {"contour.duplicate_vertices", "warning", "Contour '" + c.name + "' contains consecutive vertices closer than the snap tolerance (" + num(tol.snapTolMeters) + " " + tol.xyUnitName + ").", "contour", c.id, -1, {}, true, {}});
            report.candidates.push_back({"repair.dedupe.c" + std::to_string(c.id), "contour.duplicate_vertices",
                                        "Remove duplicate vertices", "Keep the first occurrence of each near-coincident run.", RepairKind::ReplaceContour, c.id, -1, {}, true});
        }

        // very short segments
        int shortCount = 0;
        std::vector<Vec2> shortPositions;
        for (std::size_t i = 0; i + 1 < deduped.size(); ++i) {
            if (distance(deduped[i], deduped[i + 1]) < tol.shortSegmentUnits) {
                ++shortCount;
                shortPositions.push_back(deduped[i]);
            }
        }
        if (c.closed && deduped.size() > 2 && distance(deduped.front(), deduped.back()) < tol.shortSegmentUnits) {
            ++shortCount;
            shortPositions.push_back(deduped.back());
        }
        if (shortCount > 0) {
            pushIssue(report, {"contour.short_segment", "warning", "Contour '" + c.name + "' has " + std::to_string(shortCount) + " segment(s) shorter than " + num(tol.shortSegmentMeters) + " " + tol.xyUnitName + ".", "contour", c.id, -1, shortPositions, true, {}});
            report.candidates.push_back({"repair.short.c" + std::to_string(c.id), "contour.short_segment",
                                        "Remove short segments", "Merge vertices separated by less than the short-segment threshold.", RepairKind::ReplaceContour, c.id, -1, {}, false});
        }

        // self intersections
        const std::vector<SelfHit> selfHits = findSelfIntersections(deduped, c.closed, snap);
        if (!selfHits.empty()) {
            std::vector<Vec2> positions;
            bool overlap = false;
            for (const SelfHit& h : selfHits) {
                positions.push_back(h.point);
                overlap = overlap || h.overlap;
            }
            std::string msg = "Contour '" + c.name + "' " + (overlap ? "has an overlapping segment" : "self-intersects") +
                              " (" + std::to_string(selfHits.size()) + " hit(s)); geometry cannot be constrained as a simple line.";
            pushIssue(report, {"contour.self_intersection", "error", msg, "contour", c.id, -1, positions, !overlap, {}});
            if (!overlap) {
                report.candidates.push_back({"repair.split.c" + std::to_string(c.id), "contour.self_intersection",
                                            "Split at self-intersection", "Split the line at the first crossing into simple sub-lines; the user must confirm because elevation continuity is a judgement.", RepairKind::ReplaceContour, c.id, -1, {}, false});
            }
        }

        // boundary clipping candidates
        const std::size_t segCountC = (c.closed && c.points.size() > 1) ? c.points.size()
                                                                       : (c.points.size() > 1 ? c.points.size() - 1 : 0);
        bool leavesDomain = false;
        for (std::size_t i = 0; i < segCountC; ++i) {
            const Vec2 mid = (c.points[i] + c.points[(i + 1) % c.points.size()]) * 0.5;
            if (classifyPoint(mid, outer, input.holes, snap) == DomainLoc::Outside)
                leavesDomain = true;
            if (classifyPoint(c.points[i], outer, input.holes, snap) == DomainLoc::Outside)
                leavesDomain = true;
        }
        if (leavesDomain) {
            std::vector<PolylinePiece> pieces = clipPolylineToDomain(c.points, c.closed, outer, input.holes, snap);
            // The triangulation can only constrain edges inside the domain, so
            // a contour that leaves the boundary must be cut before building;
            // it is reported as a blocking error with a confirmable candidate.
            pushIssue(report, {"contour.outside_boundary", "error", "Contour '" + c.name + "' leaves the boundary; the outside part cannot be constrained by the triangulation.", "contour", c.id, -1, c.points, true, {}});
            report.candidates.push_back({"repair.clip.c" + std::to_string(c.id), "contour.outside_boundary",
                                        "Clip to boundary (keep inside pieces)", "Retain only the portions inside the boundary and outside holes.", RepairKind::ReplaceContour, c.id, -1, pieces, true});
        }
        for (const Vec2& p : c.points) {
            if (registry.hasNearDifferentElevation(p, c.elevation)) {
                pushIssue(report, {"contour.vertex_near_other_elevation", "warning",
                                   "Contour '" + c.name + "' has a vertex within the snap tolerance of a different-elevation line. They are NOT merged; check the survey data.",
                                   "contour", c.id, -1, {p}, false, {}});
            }
        }
    }

    // ---- cross-contour relationships ----
    std::vector<std::tuple<int, int, std::string, Vec2, bool>> crossHits; // cA,cB,kind,pos,overlap
    for (std::size_t i = 0; i < allSegments.size(); ++i) {
        for (std::size_t j = i + 1; j < allSegments.size(); ++j) {
            const SegmentRef& a = allSegments[i];
            const SegmentRef& b = allSegments[j];
            if (a.contourId == b.contourId)
                continue;
            const double dz = std::abs(a.elevation - b.elevation);
            const SegmentHit h = intersectSegments(a.a, a.b, b.a, b.b, snap);
            if (h.kind == SegmentHit::None)
                continue;
            const bool sameZ = dz <= tol.zTolUnits;
            const bool isCrossing = h.kind == SegmentHit::Overlap ||
                                    (h.interiorAB || h.interiorCD || std::abs(h.t) < 1e-12 || std::abs(h.t - 1.0) < 1e-12);
            if (!isCrossing)
                continue;
            if (!sameZ) {
                crossHits.emplace_back(a.contourId, b.contourId,
                                       h.kind == SegmentHit::Overlap ? "different_z_overlap" : "different_z_cross",
                                       h.point, h.kind == SegmentHit::Overlap);
            }
        }
    }
    for (const auto& hit : crossHits) {
        const int ca = std::get<0>(hit);
        const int cb = std::get<1>(hit);
        const std::string kind = std::get<2>(hit);
        const Vec2 p = std::get<3>(hit);
        const std::string code = kind == "different_z_overlap" ? "contour.different_z_overlap" : "contour.different_z_cross";
        for (const ContourInput& c : input.contours) {
            if (c.id != ca)
                continue;
            for (const ContourInput& d : input.contours) {
                if (d.id != cb)
                    continue;
                const std::string sev = kind == "different_z_cross" ? "error" : "error";
                std::string msg = kind == "different_z_overlap"
                                      ? "Contours '" + c.name + "' and '" + d.name + "' overlap while their elevations differ by " + num(std::abs(c.elevation - d.elevation)) + " " + tol.zUnitName + ". They are kept separate; the overlap must be resolved by the user."
                                      : "Contours '" + c.name + "' and '" + d.name + "' cross at (" + num(p.x) + ", " + num(p.y) + ") with elevations differing by " + num(std::abs(c.elevation - d.elevation)) + " " + tol.zUnitName + ". Crossing lines at different elevations cannot form a single constrained edge.";
                // No automatic merge is offered: a crossing at different
                // elevations is a survey conflict the user must resolve, so the
                // issue is not repairable by a single candidate.
                pushIssue(report, {code, sev, msg, "contour", ca, -1, {p}, false, {}});
            }
        }
        report.candidates.push_back({"repair.cross.c" + std::to_string(ca) + ".c" + std::to_string(cb), code,
                                    "Review crossing (" + std::to_string(ca) + ", " + std::to_string(cb) + ")",
                                    "No automatic merging is offered for different-elevation crossings. Candidate options: drop one line or edit its elevation/source vertices, then re-run validation.",
                                    RepairKind::ReplaceContour, ca, -1, {}, false});
    }

    // ---- control points ----
    std::vector<double> zs = allContourElevations(input);
    const std::vector<double> controlElevations = allControlElevations(input);
    zs.insert(zs.end(), controlElevations.begin(), controlElevations.end());
    double zMin = zs.empty() ? 0.0 : *std::min_element(zs.begin(), zs.end());
    double zMax = zs.empty() ? 0.0 : *std::max_element(zs.begin(), zs.end());
    if (!zs.empty() && std::abs(zMax - zMin) < tol.zTolUnits)
        pushIssue(report, {"elevation.no_relief", "warning", "All elevations are within " + num(tol.zTolUnits) + " " + tol.zUnitName + "; the surface will be flat.", "meta", -1, -1, {}, false, {}});

    for (std::size_t ci = 0; ci < input.controls.size(); ++ci) {
        const ControlPoint& cp = input.controls[ci];
        const DomainLoc loc = classifyPoint(cp.position, outer, input.holes, snap);
        if (loc == DomainLoc::Outside) {
            pushIssue(report, {"control.outside_boundary", "warning", "Control point '" + cp.name + "' lies outside the boundary and cannot influence the surface.", "control", cp.id, -1, {cp.position}, true, {}});
            report.candidates.push_back({"repair.dropControl.k" + std::to_string(cp.id), "control.outside_boundary",
                                        "Exclude control point", "Keep it in the original data but exclude it from interpolation.", RepairKind::DropControl, -1, cp.id, {}, true});
        } else if (loc == DomainLoc::HoleInside) {
            pushIssue(report, {"control.in_hole", "warning", "Control point '" + cp.name + "' lies inside a hole; the hole is not filled.", "control", cp.id, -1, {cp.position}, true, {}});
            report.candidates.push_back({"repair.dropControl.k" + std::to_string(cp.id), "control.in_hole",
                                        "Exclude control point", "Keep it in the original data but exclude it from interpolation.", RepairKind::DropControl, -1, cp.id, {}, true});
        }
    }
    // duplicate controls
    for (std::size_t i = 0; i < input.controls.size(); ++i) {
        for (std::size_t j = i + 1; j < input.controls.size(); ++j) {
            const ControlPoint& a = input.controls[i];
            const ControlPoint& b = input.controls[j];
            const double d = distance(a.position, b.position);
            if (d <= snap) {
                if (std::abs(a.elevation - b.elevation) <= tol.zTolUnits) {
                    pushIssue(report, {"control.duplicate", "warning", "Control points '" + a.name + "' and '" + b.name + "' coincide within tolerance and have the same elevation; they are redundant.", "control", a.id, -1, {a.position}, true, {}});
                    report.candidates.push_back({"repair.dropControl.k" + std::to_string(b.id), "control.duplicate",
                                                "Drop redundant control point", "Keep '" + a.name + "'; both are shown in the original data.", RepairKind::DropControl, -1, b.id, {}, false});
                } else {
                    pushIssue(report, {"control.duplicate_conflict", "error", "Control points '" + a.name + "' and '" + b.name + "' coincide but have elevations differing by " + num(std::abs(a.elevation - b.elevation)) + " " + tol.zUnitName + ".", "control", a.id, -1, {a.position}, false, {}});
                }
            }
        }
    }

    // ---- faults ----
    for (const FaultInput& f : input.faults) {
        if (f.points.size() < 2) {
            pushIssue(report, {"fault.too_few_points", "error", "Fault line '" + f.name + "' needs at least two vertices.", "fault", f.id, -1, f.points, false, {}});
            continue;
        }
        const std::vector<SelfHit> selfHits = findSelfIntersections(f.points, false, snap);
        for (const SelfHit& h : selfHits)
            pushIssue(report, {"fault.self_intersection", "error", "Fault line '" + f.name + "' self-intersects near (" + num(h.point.x) + ", " + num(h.point.y) + ").", "fault", f.id, -1, {h.point}, false, {}});
        const bool outside = [&]() {
            for (const Vec2& p : f.points)
                if (classifyPoint(p, outer, input.holes, snap) == DomainLoc::Outside)
                    return true;
            return false;
        }();
        if (outside)
            pushIssue(report, {"fault.outside_boundary", "warning", "Fault line '" + f.name + "' leaves the boundary; the outside part is clipped.", "fault", f.id, -1, f.points, true, {}});
    }

    // ---- contour overlap / duplicates (same elevation) ----
    for (std::size_t i = 0; i < input.contours.size(); ++i) {
        for (std::size_t j = i + 1; j < input.contours.size(); ++j) {
            const ContourInput& a = input.contours[i];
            const ContourInput& b = input.contours[j];
            if (std::abs(a.elevation - b.elevation) > tol.zTolUnits)
                continue;
            if (polylinesEquivalent(a.points, b.points, a.closed, snap)) {
                pushIssue(report, {"contour.duplicate", "warning", "Contours '" + a.name + "' and '" + b.name + "' are geometrically identical with the same elevation " + num(a.elevation) + " " + tol.zUnitName + ".", "contour", a.id, -1, a.points, true, {}});
                report.candidates.push_back({"repair.drop.c" + std::to_string(b.id), "contour.duplicate",
                                            "Drop duplicate contour '" + b.name + "'", "Keep '" + a.name + "'; duplicates constrain the same edges twice.", RepairKind::DropContour, b.id, -1, {}, false});
                continue;
            }
            // Partial duplicate: the whole of b must lie on a's segments. Sampling
            // segment midpoints as well as vertices keeps a two-vertex fragment
            // from trivially matching endpoints of unrelated lines.
            int sampled = 0;
            int onCount = 0;
            auto liesOnA = [&](const Vec2& p) {
                for (std::size_t k = 0; k < a.points.size(); ++k) {
                    const std::size_t k2 = (k + 1) % a.points.size();
                    if (!a.closed && k + 1 >= a.points.size())
                        break;
                    if (pointOnSegment(p, a.points[k], a.points[k2], snap))
                        return true;
                }
                return false;
            };
            const std::size_t segCountB = (b.closed && b.points.size() > 1)
                                              ? b.points.size()
                                              : (b.points.size() > 1 ? b.points.size() - 1 : 0);
            for (std::size_t k = 0; k < b.points.size(); ++k) {
                ++sampled;
                if (liesOnA(b.points[k]))
                    ++onCount;
            }
            for (std::size_t k = 0; k < segCountB; ++k) {
                const Vec2 mid = (b.points[k] + b.points[(k + 1) % b.points.size()]) * 0.5;
                ++sampled;
                if (liesOnA(mid))
                    ++onCount;
            }
            if (sampled > 0 && onCount == sampled && b.points.size() >= 2) {
                pushIssue(report, {"contour.subline", "warning", "Contour '" + b.name + "' lies on top of contour '" + a.name + "' at the same elevation; it duplicates constraint edges.", "contour", b.id, -1, b.points, true, {}});
                report.candidates.push_back({"repair.drop.c" + std::to_string(b.id), "contour.subline",
                                            "Drop embedded contour '" + b.name + "'", "Its elevation is already represented by '" + a.name + "'.", RepairKind::DropContour, b.id, -1, {}, false});
            }
        }
    }

    // ---- fills for the UI so a user can confirm candidates without losing raw data ----
    for (const ContourInput& c : input.contours) {
        for (RepairCandidate& cand : report.candidates) {
            if (cand.contourId != c.id)
                continue;
            if (cand.code == "repair.boundary.close")
                continue;
            if (cand.code == "contour.outside_boundary" && cand.pieces.empty()) {
                cand.pieces = clipPolylineToDomain(c.points, c.closed, outer, input.holes, snap);
            }
            if (cand.code == "contour.self_intersection" && cand.pieces.empty()) {
                Polyline dedupedCopy = dedupeConsecutive(c.points, snap);
                const std::vector<Polyline> parts = splitPolylineAtSelfIntersections(dedupedCopy, c.closed, snap);
                for (const Polyline& p : parts)
                    cand.pieces.push_back({p, c.closed && distance(p.front(), p.back()) <= snap});
            }
            if (cand.code == "contour.duplicate_vertices" && cand.pieces.empty()) {
                Polyline dedupedCopy = dedupeConsecutive(c.points, snap);
                cand.pieces.push_back({dedupedCopy, c.closed});
            }
            if (cand.code == "contour.short_segment" && cand.pieces.empty()) {
                Polyline cleaned = dedupeConsecutive(c.points, tol.shortSegmentUnits);
                bool closedFlag = c.closed;
                if (closedFlag && cleaned.size() > 3 && distance(cleaned.front(), cleaned.back()) <= snap)
                    cleaned.pop_back();
                cand.pieces.push_back({cleaned, closedFlag});
            }
        }
    }
    // attach contour ids to code strings for the UI (already encoded in ids).
    return report;
}

DataInput applyRepairs(const DataInput& input, const std::vector<RepairCandidate>& candidates,
                       const std::vector<std::string>& selectedIds, std::vector<std::string>* notes)
{
    DataInput out = input;
    out.contours.clear();
    std::map<int, const RepairCandidate*> chosenPerContour;
    std::set<int> dropControl;
    std::set<int> dropContour;
    bool closeBoundary = false;
    for (const std::string& id : selectedIds) {
        for (const RepairCandidate& cand : candidates) {
            if (cand.id != id)
                continue;
            if (notes)
                notes->push_back(cand.label);
            if (cand.kind == RepairKind::CloseBoundary) {
                closeBoundary = true;
            } else if (cand.kind == RepairKind::DropContour) {
                dropContour.insert(cand.contourId);
            } else if (cand.kind == RepairKind::DropControl) {
                dropControl.insert(cand.controlId);
            } else if (cand.kind == RepairKind::ReplaceContour) {
                // Only one replacement per contour; later selections supersede
                // earlier ones so the same vertices are not constrained twice.
                chosenPerContour[cand.contourId] = &cand;
            }
        }
    }

    int nextId = 0;
    for (const ContourInput& c : input.contours)
        nextId = std::max(nextId, c.id + 1);

    for (const ContourInput& c : input.contours) {
        if (dropContour.count(c.id))
            continue;
        auto it = chosenPerContour.find(c.id);
        if (it == chosenPerContour.end() || it->second->pieces.empty()) {
            out.contours.push_back(c);
            continue;
        }
        for (const PolylinePiece& piece : it->second->pieces) {
            if (piece.points.size() < 2)
                continue;
            ContourInput derived = c;
            derived.id = nextId++;
            derived.points = piece.points;
            derived.closed = piece.closed;
            derived.derivedFrom = c.id;
            derived.derivedKind = it->second->code;
            derived.name = c.name + " [repair]";
            out.contours.push_back(derived);
        }
    }

    out.controls.clear();
    for (const ControlPoint& cp : input.controls) {
        if (dropControl.count(cp.id))
            continue;
        out.controls.push_back(cp);
    }
    if (closeBoundary && !out.outer.empty()) {
        if (distance(out.outer.front(), out.outer.back()) > 0.0)
            out.outer.push_back(out.outer.front());
    }
    return out;
}

void finalizeDerivedContours(DataInput& data, int baseId)
{
    (void)data;
    (void)baseId;
}

std::string validationReportText(const ValidationReport& report)
{
    std::ostringstream os;
    os << "Validation: " << report.errorCount << " error(s), "
       << report.warningCount << " warning(s), " << report.infoCount << " info, "
       << report.candidates.size() << " repair candidate(s)\n";
    for (const ValidationIssue& issue : report.issues) {
        os << "[" << issue.severity << "] " << issue.code << ": " << issue.message << "\n";
    }
    return os.str();
}

} // namespace terrain
