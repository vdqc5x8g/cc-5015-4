#pragma once

#include "terrain/input.h"
#include "terrain/mesh.h"
#include "terrain/tolerance.h"

#include <string>
#include <vector>

namespace terrain {

// Selected repair ids come from the UI; an empty vector means "build the raw
// data" (which is only possible if validation reported no blocking errors).
struct PipelineOptions {
    std::vector<std::string> selectedRepairIds;
    double snapToleranceMeters = 0.0;
    double zNearToleranceMeters = 0.0;
};

struct PipelineResult {
    DataInput original;
    DataInput repaired;
    bool hasRepairs = false;
    ValidationReport validation;
    ValidationReport repairedValidation;
    Tolerance tolerance;
    MeshBuildResult mesh;
    std::vector<std::string> appliedRepairs;
    std::string error;
    double totalMs = 0.0;
};

// Runs parse -> tolerance -> validation -> optional repair -> mesh. The
// original document and entities are never mutated.
PipelineResult runPipeline(const std::string& jsonText, const PipelineOptions& options);

// JSON for the UI. Contains original data, derived data, issues, candidates,
// mesh and timing. Mesh size can be large; it is included so the viewer draws
// real geometry rather than a texture.
std::string pipelineToJson(const PipelineResult& result, bool includeMesh);

} // namespace terrain
