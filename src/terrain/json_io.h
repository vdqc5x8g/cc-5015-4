#pragma once

#include "terrain/input.h"

#include <string>

namespace terrain {

// Parses a JSON document. The exact source text is stored in
// DataInput::sourceDocument before any repair, together with each entity's
// verbatim fragment, so the original data remains queryable.
bool loadDataInputFromJson(const std::string& text, DataInput& out, std::string* error);

std::string dataInputToJson(const DataInput& input, bool pretty = true);

} // namespace terrain
