#include "terrain/json_io.h"

#include "json.hpp"

#include <sstream>

namespace terrain {

namespace {

using nlohmann::json;

Vec2 readPoint(const json& j)
{
    if (!j.is_array() || j.size() < 2)
        throw std::runtime_error("point must be an array [x, y]");
    return Vec2{j[0].get<double>(), j[1].get<double>()};
}

Polyline readLine(const json& j)
{
    if (!j.is_array())
        throw std::runtime_error("line must be an array of points");
    Polyline line;
    line.reserve(j.size());
    for (const json& p : j)
        line.push_back(readPoint(p));
    return line;
}

double getOr(const json& j, const char* key, double fallback)
{
    if (!j.contains(key) || j[key].is_null())
        return fallback;
    return j[key].get<double>();
}

std::string getStr(const json& j, const char* key, const std::string& fallback)
{
    if (!j.contains(key) || j[key].is_null())
        return fallback;
    return j[key].get<std::string>();
}

bool getBool(const json& j, const char* key, bool fallback)
{
    if (!j.contains(key) || j[key].is_null())
        return fallback;
    return j[key].get<bool>();
}

} // namespace

bool loadDataInputFromJson(const std::string& text, DataInput& out, std::string* error)
{
    try {
        const json doc = json::parse(text, nullptr, true, true);
        out = DataInput{};
        out.sourceDocument = text;
        json meta = doc.value("meta", json::object());
        out.name = getStr(doc, "name", "terrain");
        out.meta.name = getStr(meta, "name", out.name);
        out.meta.xyUnitName = getStr(meta, "xy_unit", "m");
        out.meta.xyMetersPerUnit = getOr(meta, "xy_meters_per_unit", 1.0);
        out.meta.zUnitName = getStr(meta, "z_unit", "m");
        out.meta.zMetersPerUnit = getOr(meta, "z_meters_per_unit", 1.0);
        out.meta.zDatum = getStr(meta, "z_datum", "");
        out.meta.coordinateNote = getStr(meta, "coordinate_note", "");
        out.meta.userSnapToleranceMeters = getOr(meta, "snap_tolerance_m", 0.0);
        out.meta.userZNearToleranceMeters = getOr(meta, "z_near_tolerance_m", 0.0);

        if (doc.contains("boundary")) {
            out.outer = readLine(doc["boundary"]);
        }
        if (doc.contains("holes") && doc["holes"].is_array()) {
            for (const json& h : doc["holes"])
                out.holes.push_back(readLine(h));
        }

        int nextContourId = 0;
        if (doc.contains("contours") && doc["contours"].is_array()) {
            int idx = 0;
            for (const json& c : doc["contours"]) {
                ContourInput ci;
                ci.id = c.value("id", nextContourId++);
                ci.sourceIndex = idx++;
                ci.name = getStr(c, "name", "contour " + std::to_string(ci.id));
                ci.points = readLine(c.at("points"));
                ci.closed = getBool(c, "closed", false);
                ci.elevation = c.value("elevation", 0.0);
                ci.originalText = c.dump();
                if (c.contains("attributes") && c["attributes"].is_object()) {
                    for (auto it = c["attributes"].begin(); it != c["attributes"].end(); ++it) {
                        if (it.value().is_string())
                            ci.attributes[it.key()] = it.value().get<std::string>();
                        else
                            ci.attributes[it.key()] = it.value().dump();
                    }
                }
                out.contours.push_back(std::move(ci));
            }
        }

        int nextControlId = 0;
        if (doc.contains("controls") && doc["controls"].is_array()) {
            int idx = 0;
            for (const json& c : doc["controls"]) {
                ControlPoint cp;
                cp.id = c.value("id", nextControlId++);
                cp.sourceIndex = idx++;
                cp.name = getStr(c, "name", "control " + std::to_string(cp.id));
                if (c.contains("position"))
                    cp.position = readPoint(c["position"]);
                else
                    cp.position = Vec2{c.value("x", 0.0), c.value("y", 0.0)};
                cp.elevation = c.value("elevation", 0.0);
                cp.originalText = c.dump();
                out.controls.push_back(std::move(cp));
            }
        }

        int nextFaultId = 0;
        if (doc.contains("faults") && doc["faults"].is_array()) {
            int idx = 0;
            for (const json& f : doc["faults"]) {
                FaultInput fi;
                fi.id = f.value("id", nextFaultId++);
                fi.sourceIndex = idx++;
                fi.name = getStr(f, "name", "fault " + std::to_string(fi.id));
                fi.points = readLine(f.at("points"));
                out.faults.push_back(std::move(fi));
            }
        }
        return true;
    } catch (const std::exception& e) {
        if (error)
            *error = e.what();
        return false;
    }
}

std::string dataInputToJson(const DataInput& input, bool pretty)
{
    json doc;
    doc["name"] = input.name;
    doc["meta"]["name"] = input.meta.name;
    doc["meta"]["xy_unit"] = input.meta.xyUnitName;
    doc["meta"]["xy_meters_per_unit"] = input.meta.xyMetersPerUnit;
    doc["meta"]["z_unit"] = input.meta.zUnitName;
    doc["meta"]["z_meters_per_unit"] = input.meta.zMetersPerUnit;
    doc["meta"]["z_datum"] = input.meta.zDatum;
    if (!input.meta.coordinateNote.empty())
        doc["meta"]["coordinate_note"] = input.meta.coordinateNote;

    auto toArray = [](const Polyline& line) {
        json a = json::array();
        for (const Vec2& p : line)
            a.push_back({p.x, p.y});
        return a;
    };
    doc["boundary"] = toArray(input.outer);
    doc["holes"] = json::array();
    for (const Ring& h : input.holes)
        doc["holes"].push_back(toArray(h));
    doc["contours"] = json::array();
    for (const ContourInput& c : input.contours) {
        json j;
        j["id"] = c.id;
        j["name"] = c.name;
        j["points"] = toArray(c.points);
        j["closed"] = c.closed;
        j["elevation"] = c.elevation;
        if (!c.attributes.empty()) {
            j["attributes"] = json::object();
            for (const auto& kv : c.attributes)
                j["attributes"][kv.first] = kv.second;
        }
        doc["contours"].push_back(j);
    }
    doc["controls"] = json::array();
    for (const ControlPoint& c : input.controls) {
        doc["controls"].push_back({{"id", c.id}, {"name", c.name}, {"position", {c.position.x, c.position.y}}, {"elevation", c.elevation}});
    }
    doc["faults"] = json::array();
    for (const FaultInput& f : input.faults) {
        doc["faults"].push_back({{"id", f.id}, {"name", f.name}, {"points", toArray(f.points)}});
    }
    return pretty ? doc.dump(2) : doc.dump();
}

} // namespace terrain
