#pragma once

#include "terrain/geometry.h"
#include "terrain/tolerance.h"

#include <map>
#include <string>
#include <vector>

namespace terrain {

struct ContourInput {
    int id = -1;
    std::string name;
    Polyline points; // original coordinates, never modified after load
    bool closed = false;
    double elevation = 0.0;
    std::map<std::string, std::string> attributes;
    int sourceIndex = 0;
    std::string originalText; // verbatim JSON fragment for auditing
    int derivedFrom = -1;    // original contour id when this is a repair piece
    std::string derivedKind;  // clip / split / dedupe / ...
};

struct ControlPoint {
    int id = -1;
    std::string name;
    Vec2 position; // original coordinates
    double elevation = 0.0;
    int sourceIndex = 0;
    std::string originalText;
};

struct FaultInput {
    int id = -1;
    std::string name;
    Polyline points; // original coordinates
    int sourceIndex = 0;
};

struct DataInput {
    std::string name;
    Meta meta;
    Ring outer; // original coordinates, open or closed
    std::vector<Ring> holes;
    std::vector<ContourInput> contours;
    std::vector<ControlPoint> controls;
    std::vector<FaultInput> faults;
    std::string sourceDocument; // verbatim JSON document
};

struct ValidationIssue {
    std::string code;
    std::string severity; // error | warning | info
    std::string message;
    std::string entityType; // boundary | hole | contour | control | fault | meta
    int entityId = -1;
    int holeIndex = -1;
    std::vector<Vec2> positions; // original coordinates for UI focus
    bool repairable = false;
    std::vector<std::string> candidateIds;
};

enum class RepairKind {
    ReplaceContour, // pieces replace the original contour
    DropContour,
    DropControl,
    CloseBoundary,
};

struct RepairCandidate {
    std::string id;
    std::string code;
    std::string label;
    std::string detail;
    RepairKind kind = RepairKind::ReplaceContour;
    int contourId = -1;
    int controlId = -1;
    std::vector<PolylinePiece> pieces; // original coordinates
    bool recommended = false;
};

struct ValidationReport {
    std::vector<ValidationIssue> issues;
    std::vector<RepairCandidate> candidates;
    int errorCount = 0;
    int warningCount = 0;
    int infoCount = 0;
    double elapsedMs = 0.0;
    bool hasBlockingErrors() const { return errorCount > 0; }
};

ValidationReport validateInput(const DataInput& input, const Tolerance& tol);
std::string validationReportText(const ValidationReport& report);

// Applies only the user-confirmed candidates. The input is never mutated.
DataInput applyRepairs(const DataInput& input, const std::vector<RepairCandidate>& candidates,
                       const std::vector<std::string>& selectedIds, std::vector<std::string>* notes = nullptr);

// Fills in derived ids on repair pieces; deterministic for identical input.
void finalizeDerivedContours(DataInput& data, int baseId);

} // namespace terrain
