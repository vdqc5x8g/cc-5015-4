#pragma once

#include "terrain/mesh.h"

#include <string>

namespace terrain {

// Exports the real triangulated surface (not a contour texture).
// OBJ stores original coordinates; the vertical axis is Z and the horizontal
// pair is X/Y with the declared unit. STL is a binary triangle soup.
std::string meshToObj(const MeshBuildResult& mesh, const Tolerance& tol);
std::string meshToStlBinary(const MeshBuildResult& mesh, const Tolerance& tol);
std::string meshToCsvVertices(const MeshBuildResult& mesh);
std::string meshToCsvTriangles(const MeshBuildResult& mesh);
std::string meshToJson(const MeshBuildResult& mesh, const Tolerance& tol, const DataInput& input);

} // namespace terrain
