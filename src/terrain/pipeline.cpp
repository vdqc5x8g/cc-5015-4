#include "terrain/pipeline.h"

#include "terrain/json_io.h"

#include "json.hpp"

#include <chrono>
#include <sstream>

namespace terrain {

namespace {

void collectPoints(const DataInput& input, std::vector<Vec2>& pts, std::vector<double>& lens,
                   std::vector<double>& contourZ, std::vector<double>& controlZ)
{
    pts.clear();
    lens.clear();
    contourZ.clear();
    controlZ.clear();
    for (const Vec2& p : input.outer)
        pts.push_back(p);
    for (const Ring& h : input.holes)
        for (const Vec2& p : h)
            pts.push_back(p);
    for (const ContourInput& c : input.contours) {
        contourZ.push_back(c.elevation);
        for (std::size_t i = 0; i < c.points.size(); ++i) {
            pts.push_back(c.points[i]);
            const std::size_t n = c.points.size();
            if (i + 1 < n)
                lens.push_back(distance(c.points[i], c.points[i + 1]));
            else if (c.closed && n > 1)
                lens.push_back(distance(c.points[i], c.points[0]));
        }
    }
    for (const ControlPoint& c : input.controls) {
        pts.push_back(c.position);
        controlZ.push_back(c.elevation);
    }
    for (const FaultInput& f : input.faults)
        for (const Vec2& p : f.points)
            pts.push_back(p);
}

} // namespace

PipelineResult runPipeline(const std::string& jsonText, const PipelineOptions& options)
{
    PipelineResult result;
    const auto t0 = std::chrono::steady_clock::now();
    std::string parseError;
    if (!loadDataInputFromJson(jsonText, result.original, &parseError)) {
        result.error = "JSON parse error: " + parseError;
        return result;
    }
    result.original.meta.userSnapToleranceMeters = options.snapToleranceMeters;
    result.original.meta.userZNearToleranceMeters = options.zNearToleranceMeters;

    std::vector<Vec2> pts;
    std::vector<double> lens, contourZ, controlZ;
    collectPoints(result.original, pts, lens, contourZ, controlZ);
    result.tolerance = computeTolerance(pts, lens, contourZ, controlZ, result.original.meta, 53);

    result.validation = validateInput(result.original, result.tolerance);

    result.repaired = result.original;
    result.hasRepairs = false;
    if (!options.selectedRepairIds.empty() && !result.validation.candidates.empty()) {
        result.repaired = applyRepairs(result.original, result.validation.candidates,
                                       options.selectedRepairIds, &result.appliedRepairs);
        result.hasRepairs = result.appliedRepairs.size() > 0;
    }
    if (result.hasRepairs) {
        collectPoints(result.repaired, pts, lens, contourZ, controlZ);
        result.tolerance = computeTolerance(pts, lens, contourZ, controlZ,
                                            result.repaired.meta, 53);
        result.repairedValidation = validateInput(result.repaired, result.tolerance);
        if (result.repairedValidation.hasBlockingErrors()) {
            result.error = "repairs did not remove all blocking errors; review the remaining issues";
            result.totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            return result;
        }
    } else {
        result.repairedValidation = result.validation;
        if (result.validation.hasBlockingErrors()) {
            result.error = "input has blocking errors; confirm a repair or edit the source data before building";
            result.totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            return result;
        }
    }

    result.mesh = buildMesh(result.repaired, result.tolerance);
    if (!result.mesh.ok)
        result.error = result.mesh.error;
    result.totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return result;
}

namespace {

nlohmann::json issueJson(const ValidationIssue& i)
{
    nlohmann::json iss;
    iss["code"] = i.code;
    iss["severity"] = i.severity;
    iss["message"] = i.message;
    iss["entity_type"] = i.entityType;
    iss["entity_id"] = i.entityId;
    iss["hole_index"] = i.holeIndex;
    iss["repairable"] = i.repairable;
    iss["candidates"] = i.candidateIds;
    nlohmann::json pos = nlohmann::json::array();
    for (const Vec2& p : i.positions)
        pos.push_back({p.x, p.y});
    iss["positions"] = pos;
    return iss;
}

nlohmann::json lineJson(const Polyline& line)
{
    nlohmann::json a = nlohmann::json::array();
    for (const Vec2& p : line)
        a.push_back({p.x, p.y});
    return a;
}

} // namespace

std::string pipelineToJson(const PipelineResult& result, bool includeMesh)
{
    using nlohmann::json;
    json doc;
    doc["ok"] = result.error.empty() && result.mesh.ok;
    doc["error"] = result.error;
    doc["total_ms"] = result.totalMs;
    doc["has_repairs"] = result.hasRepairs;
    doc["applied_repairs"] = result.appliedRepairs;

    // ---- original data (never mutated) ----
    json original;
    original["name"] = result.original.name;
    original["meta"] = {
        {"name", result.original.meta.name},
        {"xy_unit", result.original.meta.xyUnitName},
        {"xy_meters_per_unit", result.original.meta.xyMetersPerUnit},
        {"z_unit", result.original.meta.zUnitName},
        {"z_meters_per_unit", result.original.meta.zMetersPerUnit},
        {"z_datum", result.original.meta.zDatum},
        {"coordinate_note", result.original.meta.coordinateNote},
    };
    original["boundary"] = lineJson(result.original.outer);
    original["holes"] = json::array();
    for (const Ring& h : result.original.holes)
        original["holes"].push_back(lineJson(h));
    original["contours"] = json::array();
    for (const ContourInput& c : result.original.contours) {
        original["contours"].push_back({
            {"id", c.id},
            {"name", c.name},
            {"elevation", c.elevation},
            {"closed", c.closed},
            {"points", lineJson(c.points)},
            {"derived_from", c.derivedFrom},
            {"source_index", c.sourceIndex},
            {"original_text", c.originalText},
            {"attributes", c.attributes},
        });
    }
    original["controls"] = json::array();
    for (const ControlPoint& c : result.original.controls) {
        original["controls"].push_back(
            {{"id", c.id}, {"name", c.name}, {"x", c.position.x}, {"y", c.position.y},
             {"elevation", c.elevation}, {"original_text", c.originalText}});
    }
    original["faults"] = json::array();
    for (const FaultInput& f : result.original.faults) {
        original["faults"].push_back({{"id", f.id}, {"name", f.name}, {"points", lineJson(f.points)}});
    }
    doc["original"] = original;
    doc["original_text"] = result.original.sourceDocument;

    // ---- repaired/derived data ----
    json derived;
    derived["boundary"] = lineJson(result.repaired.outer);
    derived["holes"] = json::array();
    for (const Ring& h : result.repaired.holes)
        derived["holes"].push_back(lineJson(h));
    derived["contours"] = json::array();
    for (const ContourInput& c : result.repaired.contours) {
        derived["contours"].push_back({
            {"id", c.id},
            {"name", c.name},
            {"elevation", c.elevation},
            {"closed", c.closed},
            {"points", lineJson(c.points)},
            {"derived_from", c.derivedFrom},
            {"derived_kind", c.derivedKind},
        });
    }
    derived["controls"] = json::array();
    for (const ControlPoint& c : result.repaired.controls) {
        derived["controls"].push_back({{"id", c.id}, {"name", c.name}, {"x", c.position.x},
                                       {"y", c.position.y}, {"elevation", c.elevation}});
    }
    derived["faults"] = json::array();
    for (const FaultInput& f : result.repaired.faults)
        derived["faults"].push_back({{"id", f.id}, {"name", f.name}, {"points", lineJson(f.points)}});
    doc["derived"] = derived;

    // ---- validation ----
    json issues = json::array();
    for (const ValidationIssue& i : result.validation.issues)
        issues.push_back(issueJson(i));
    doc["validation"] = {
        {"errors", result.validation.errorCount},
        {"warnings", result.validation.warningCount},
        {"info", result.validation.infoCount},
        {"issues", issues},
        {"elapsed_ms", result.validation.elapsedMs},
    };
    json candidates = json::array();
    for (const RepairCandidate& c : result.validation.candidates) {
        json pieces = json::array();
        for (const PolylinePiece& p : c.pieces) {
            pieces.push_back({{"points", lineJson(p.points)}, {"closed", p.closed}});
        }
        candidates.push_back({{"id", c.id}, {"code", c.code}, {"label", c.label},
                              {"detail", c.detail}, {"contour_id", c.contourId},
                              {"control_id", c.controlId}, {"recommended", c.recommended},
                              {"pieces", pieces}});
    }
    doc["repair_candidates"] = candidates;

    json repairedIssues = json::array();
    for (const ValidationIssue& i : result.repairedValidation.issues)
        repairedIssues.push_back(issueJson(i));
    doc["repaired_validation"] = {
        {"errors", result.repairedValidation.errorCount},
        {"warnings", result.repairedValidation.warningCount},
        {"issues", repairedIssues},
    };

    // ---- tolerance ----
    const Tolerance& t = result.tolerance;
    doc["tolerance"] = {
        {"xy_unit", t.xyUnitName}, {"z_unit", t.zUnitName}, {"z_datum", t.zDatum},
        {"xy_meters_per_unit", t.metersPerXyUnit}, {"z_meters_per_unit", t.metersPerZUnit},
        {"origin_x", t.origin.x}, {"origin_y", t.origin.y}, {"local_scale", t.xyScale},
        {"extent_m", t.extentMeters}, {"max_abs_coordinate", t.maxAbsCoordUnits},
        {"snap_tolerance_m", t.snapTolMeters}, {"z_tolerance", t.zTolUnits},
        {"spacing_p05_m", t.spacingP05Meters}, {"median_segment", t.medianSegmentUnits},
        {"precision_floor_m", t.precisionFloorMeters}, {"precision_limited", t.precisionLimited},
        {"short_segment_m", t.shortSegmentMeters}, {"explanation", t.explanation},
    };
    doc["tolerance"]["text"] = describeTolerance(t);

    // ---- mesh ----
    json mesh;
    mesh["ok"] = result.mesh.ok;
    mesh["error"] = result.mesh.error;
    mesh["triangulation_ms"] = result.mesh.triangulationMs;
    mesh["elevation_ms"] = result.mesh.elevationMs;
    mesh["total_ms"] = result.mesh.totalMs;
    const InterpolationSummary& is = result.mesh.interpolation;
    mesh["interpolation"] = {
        {"method", is.method},
        {"observed_vertices", is.observedVertices},
        {"edge_interpolated_vertices", is.edgeInterpolatedVertices},
        {"face_interpolated_vertices", is.faceInterpolatedVertices},
        {"boundary_interpolated_vertices", is.boundaryInterpolatedVertices},
        {"unobserved_vertices", is.unobservedVertices},
        {"boundary_vertices", is.boundaryVertices},
        {"constrained_edges", is.constrainedEdges},
        {"unsupported_triangles", is.unsupportedTriangles},
        {"unsupported_vertices", is.unsupportedVertices},
        {"unsupported_area_m2", is.unsupportedArea},
        {"total_area_m2", is.totalArea},
        {"support_radius_m", is.supportRadius},
        {"max_distance_to_observation_m", is.maxDistanceToObservation},
        {"coverage_note", is.coverageNote},
        {"min_elevation", is.minElevation},
        {"max_elevation", is.maxElevation},
    };
    mesh["notes"] = json::array();
    for (const ValidationIssue& n : result.mesh.notes)
        mesh["notes"].push_back(issueJson(n));
    mesh["holes"] = json::array();
    for (const Ring& h : result.mesh.holes)
        mesh["holes"].push_back(lineJson(h));
    mesh["faults"] = json::array();
    for (const Polyline& f : result.mesh.faults)
        mesh["faults"].push_back(lineJson(f));
    mesh["vertex_count"] = result.mesh.vertices.size();
    mesh["triangle_count"] = result.mesh.triangles.size();
    mesh["edge_count"] = result.mesh.edges.size();
    if (includeMesh && result.mesh.ok) {
        json verts = json::array();
        json tri = json::array();
        json sources = json::array();
        json unsup = json::array();
        json dist = json::array();
        for (const MeshVertex& v : result.mesh.vertices) {
            verts.push_back(v.position.x);
            verts.push_back(v.position.y);
            verts.push_back(v.elevation);
            sources.push_back(v.source);
            unsup.push_back(v.unsupported ? 1 : 0);
            dist.push_back(v.distanceToObservation);
        }
        for (const MeshTriangle& mt : result.mesh.triangles) {
            tri.push_back(mt.v[0]);
            tri.push_back(mt.v[1]);
            tri.push_back(mt.v[2]);
        }
        mesh["positions"] = verts;
        mesh["sources"] = sources;
        mesh["unsupported"] = unsup;
        mesh["distance"] = dist;
        mesh["indices"] = tri;
        json constrained = json::array();
        for (const MeshEdge& e : result.mesh.edges) {
            constrained.push_back(e.a);
            constrained.push_back(e.b);
        }
        mesh["constrained_edges_indices"] = constrained;
        json kinds = json::array();
        for (const MeshEdge& e : result.mesh.edges)
            kinds.push_back(e.constraintKind);
        mesh["constrained_kinds"] = kinds;
    }
    doc["mesh"] = mesh;
    return doc.dump();
}

} // namespace terrain
