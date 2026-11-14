#include "terrain/export.h"

#include "json.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace terrain {

namespace {

std::string fmtCoord(double v)
{
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(6);
    os << v;
    std::string s = os.str();
    // trim trailing zeros but keep at least one decimal
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0')
            s.pop_back();
        if (!s.empty() && s.back() == '.')
            s.pop_back();
    }
    return s;
}

} // namespace

std::string meshToObj(const MeshBuildResult& mesh, const Tolerance& tol)
{
    std::ostringstream os;
    os << "# constraints-to-TIN export\n";
    os << "# xy unit: " << tol.xyUnitName << " (1 = " << tol.metersPerXyUnit << " m)\n";
    os << "# z unit: " << tol.zUnitName << " (1 = " << tol.metersPerZUnit << " m), datum: " << tol.zDatum << "\n";
    os << "# vertices: " << mesh.vertices.size() << ", faces: " << mesh.triangles.size() << "\n";
    os << "# elevation source: observed | constraint_edge | face | boundary_extrapolated | unobserved\n";
    os << "# unsupported vertices (beyond observation support) are marked in the v comment\n";
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const MeshVertex& v = mesh.vertices[i];
        os << "v " << fmtCoord(v.position.x) << " " << fmtCoord(v.position.y) << " "
           << fmtCoord(v.elevation) << "  # " << (i + 1) << " "
           << v.source << (v.unsupported ? " unsupported" : "") << "\n";
    }
    // Constraint edges are exported as comments with their kind so downstream
    // consumers can rebuild the constraints without re-running this tool.
    os << "# constrained edges:\n";
    for (const MeshEdge& e : mesh.edges)
        os << "# e " << (e.a + 1) << " " << (e.b + 1) << " " << e.constraintKind << "\n";
    for (const MeshTriangle& t : mesh.triangles) {
        os << "f " << (t.v[0] + 1) << " " << (t.v[1] + 1) << " " << (t.v[2] + 1);
        if (t.support != 0)
            os << "  # unsupported";
        os << "\n";
    }
    return os.str();
}

std::string meshToStlBinary(const MeshBuildResult& mesh, const Tolerance& tol)
{
    // Binary STL requires a byte buffer; this returns the bytes as a string.
    std::ostringstream os;
    std::ofstream dummy;
    const std::size_t count = mesh.triangles.size();
    std::ostringstream header;
    header << "constraints-to-TIN; xy " << tol.xyUnitName << " z " << tol.zUnitName;
    std::string text = header.str();
    text.resize(80, ' ');
    // Direct write is handled by the caller; here we build the byte string.
    std::string out;
    out.append(text.substr(0, 80));
    const std::uint32_t triCount = static_cast<std::uint32_t>(count);
    out.append(reinterpret_cast<const char*>(&triCount), 4);
    auto putFloat = [&](float f) { out.append(reinterpret_cast<const char*>(&f), 4); };
    for (const MeshTriangle& t : mesh.triangles) {
        const Vec2& a = mesh.vertices[t.v[0]].position;
        const Vec2& b = mesh.vertices[t.v[1]].position;
        const Vec2& c = mesh.vertices[t.v[2]].position;
        const double za = mesh.vertices[t.v[0]].elevation;
        const double zb = mesh.vertices[t.v[1]].elevation;
        const double zc = mesh.vertices[t.v[2]].elevation;
        const Vec2 ab{b.x - a.x, b.y - a.y};
        const Vec2 ac{c.x - a.x, c.y - a.y};
        const double nz_xy = cross(ab, ac);
        Vec2 n{-ab.y, ab.x};
        // The horizontal plane normal is (0,0,1); the mesh is a height field, so
        // a valid normal is derived from the two in-plane vectors with z
        // differences.
        const double nx = (ab.y * (zc - za) - (zb - za) * ac.y);
        const double ny = ((zb - za) * ac.x - ab.x * (zc - za));
        const double nzz = nz_xy;
        double len = std::sqrt(nx * nx + ny * ny + nzz * nzz);
        if (len <= 0.0)
            len = 1.0;
        putFloat(static_cast<float>(nx / len));
        putFloat(static_cast<float>(ny / len));
        putFloat(static_cast<float>(nzz / len));
        putFloat(static_cast<float>(a.x));
        putFloat(static_cast<float>(a.y));
        putFloat(static_cast<float>(za));
        putFloat(static_cast<float>(b.x));
        putFloat(static_cast<float>(b.y));
        putFloat(static_cast<float>(zb));
        putFloat(static_cast<float>(c.x));
        putFloat(static_cast<float>(c.y));
        putFloat(static_cast<float>(zc));
        const std::uint16_t attr = t.support ? 1 : 0;
        out.append(reinterpret_cast<const char*>(&attr), 2);
        (void)n;
        (void)dummy;
    }
    return out;
}

std::string meshToCsvVertices(const MeshBuildResult& mesh)
{
    std::ostringstream os;
    os << "index,x,y,elevation,source,observation_id,unsupported,distance_to_observation\n";
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const MeshVertex& v = mesh.vertices[i];
        os << i << ',';
        os.setf(std::ios::fixed);
        os.precision(9);
        os << v.position.x << ',' << v.position.y << ',' << v.elevation << ','
           << v.source << ',' << v.observationId << ',' << (v.unsupported ? 1 : 0) << ','
           << v.distanceToObservation << '\n';
    }
    return os.str();
}

std::string meshToCsvTriangles(const MeshBuildResult& mesh)
{
    std::ostringstream os;
    os << "index,v0,v1,v2,supported\n";
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        const MeshTriangle& t = mesh.triangles[i];
        os << i << ',' << t.v[0] << ',' << t.v[1] << ',' << t.v[2] << ',' << (t.support ? 0 : 1) << '\n';
    }
    return os.str();
}

std::string meshToJson(const MeshBuildResult& mesh, const Tolerance& tol, const DataInput& input)
{
    using nlohmann::json;
    json doc;
    doc["ok"] = mesh.ok;
    doc["error"] = mesh.error;
    doc["units"]["xy"] = tol.xyUnitName;
    doc["units"]["z"] = tol.zUnitName;
    doc["units"]["z_datum"] = tol.zDatum;
    doc["units"]["xy_meters_per_unit"] = tol.metersPerXyUnit;
    doc["units"]["z_meters_per_unit"] = tol.metersPerZUnit;
    doc["stats"]["vertices"] = mesh.vertices.size();
    doc["stats"]["triangles"] = mesh.triangles.size();
    doc["stats"]["triangulation_ms"] = mesh.triangulationMs;
    doc["stats"]["elevation_ms"] = mesh.elevationMs;
    doc["stats"]["total_ms"] = mesh.totalMs;
    doc["stats"]["constrained_edges"] = mesh.edges.size();

    json verts = json::array();
    for (const MeshVertex& v : mesh.vertices) {
        verts.push_back(json{{"x", v.position.x}, {"y", v.position.y}, {"z", v.elevation},
                             {"source", v.source}, {"obs", v.observationId},
                             {"unsupported", v.unsupported}, {"dist", v.distanceToObservation}});
    }
    json tris = json::array();
    for (const MeshTriangle& t : mesh.triangles)
        tris.push_back(json::array({t.v[0], t.v[1], t.v[2]}));
    json constrained = json::array();
    for (const MeshEdge& e : mesh.edges)
        constrained.push_back(json{{"a", e.a}, {"b", e.b}, {"kind", e.constraintKind}});
    doc["vertices"] = verts;
    doc["triangles"] = tris;
    doc["constrained_edges"] = constrained;
    doc["interpolation"]["method"] = mesh.interpolation.method;
    doc["interpolation"]["observed_vertices"] = mesh.interpolation.observedVertices;
    doc["interpolation"]["edge_interpolated_vertices"] = mesh.interpolation.edgeInterpolatedVertices;
    doc["interpolation"]["face_interpolated_vertices"] = mesh.interpolation.faceInterpolatedVertices;
    doc["interpolation"]["boundary_vertices"] = mesh.interpolation.boundaryVertices;
    doc["interpolation"]["unobserved_vertices"] = mesh.interpolation.unobservedVertices;
    doc["interpolation"]["unsupported_triangles"] = mesh.interpolation.unsupportedTriangles;
    doc["interpolation"]["unsupported_area_m2"] = mesh.interpolation.unsupportedArea;
    doc["interpolation"]["total_area_m2"] = mesh.interpolation.totalArea;
    doc["interpolation"]["support_radius_m"] = mesh.interpolation.supportRadius;
    doc["interpolation"]["max_distance_to_observation_m"] = mesh.interpolation.maxDistanceToObservation;
    doc["interpolation"]["coverage_note"] = mesh.interpolation.coverageNote;
    doc["elevation_range"] = json{{"min", mesh.interpolation.minElevation}, {"max", mesh.interpolation.maxElevation}};
    doc["source_name"] = input.name;
    return doc.dump(2);
}

} // namespace terrain
