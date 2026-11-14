#pragma once

#include "terrain/input.h"
#include "terrain/tolerance.h"

#include <string>
#include <vector>

namespace terrain {

struct MeshVertex {
    Vec2 position; // original coordinates
    double elevation = 0.0;
    // observed              elevation supplied by a contour vertex or control point
    // constraint_edge       linearly interpolated along a constrained contour segment
    // face                  harmonic (Laplace) interpolation inside the TIN
    // boundary_extrapolated harmonic value on the outer boundary / hole ring
    // unobserved            no observation reachable; display-only fallback
    std::string source;
    int observationId = -1;
    double distanceToObservation = 0.0; // metres
    bool unsupported = false;           // farther than the observation support radius
};

struct MeshTriangle {
    int v[3] = {0, 0, 0};
    int support = 0; // 0 supported by observations, 1 beyond support radius
};

struct MeshEdge {
    int a = -1;
    int b = -1;
    std::string constraintKind; // boundary | hole | contour | fault
};

struct InterpolationSummary {
    std::string method;
    int observedVertices = 0;
    int edgeInterpolatedVertices = 0;
    int faceInterpolatedVertices = 0;
    int boundaryInterpolatedVertices = 0;
    int unobservedVertices = 0;
    int boundaryVertices = 0;
    double minElevation = 0.0;
    double maxElevation = 0.0;
    int constrainedEdges = 0;
    int unsupportedTriangles = 0;
    int unsupportedVertices = 0;
    double unsupportedArea = 0.0; // square metres
    double totalArea = 0.0;       // square metres
    double supportRadius = 0.0;   // metres
    double maxDistanceToObservation = 0.0;
    std::string coverageNote;
};

struct MeshBuildResult {
    bool ok = false;
    std::string error;
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;
    std::vector<MeshEdge> edges;
    std::vector<Ring> holes;      // echoed so the viewer can draw voids
    std::vector<Polyline> faults; // echoed
    InterpolationSummary interpolation;
    double triangulationMs = 0.0;
    double elevationMs = 0.0;
    double totalMs = 0.0;
    std::vector<ValidationIssue> notes; // non-fatal build observations
};

struct MeshBuildOptions {
    // Reserved for density control. The triangulation currently uses the
    // supplied vertices only, so every constraint is represented exactly.
    int reserved = 0;
};

MeshBuildResult buildMesh(const DataInput& input, const Tolerance& tol,
                          const MeshBuildOptions& options = MeshBuildOptions{});

} // namespace terrain
