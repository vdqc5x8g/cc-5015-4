#pragma once

#include "terrain/geometry.h"

#include <string>
#include <vector>

namespace terrain {

struct Meta {
    std::string name;
    std::string xyUnitName = "m";
    double xyMetersPerUnit = 1.0;
    std::string zUnitName = "m";
    double zMetersPerUnit = 1.0;
    std::string zDatum = "local benchmark";
    std::string coordinateNote;

    // All user tolerances are given in metres and converted to the dataset's
    // coordinate units. Zero means "derive from data".
    double userSnapToleranceMeters = 0.0;
    double userZNearToleranceMeters = 0.0;
};

struct Tolerance {
    std::string xyUnitName;
    double metersPerXyUnit = 1.0;
    std::string zUnitName;
    double metersPerZUnit = 1.0;
    std::string zDatum;

    Vec2 origin;
    double xyScale = 1.0;
    double extentUnits = 0.0;
    double extentMeters = 0.0;
    double maxAbsCoordUnits = 0.0;
    double inputUlpUnits = 0.0;
    double precisionFloorUnits = 0.0;
    double precisionFloorMeters = 0.0;

    double spacingP05Units = 0.0;
    double spacingMedianUnits = 0.0;
    double spacingP05Meters = 0.0;
    double medianSegmentUnits = 0.0;

    double snapTolUnits = 0.0;
    double snapTolMeters = 0.0;
    double zTolUnits = 0.0;
    double zTolMeters = 0.0;
    double shortSegmentUnits = 0.0;
    double shortSegmentMeters = 0.0;
    double minElevationGap = 0.0;

    bool precisionLimited = false;
    bool userToleranceCapped = false;
    std::string explanation;

    Vec2 toLocal(const Vec2& p) const;
    Vec2 fromLocal(const Vec2& p) const;
    double lengthToLocal(double lengthUnits) const;
    double lengthFromLocal(double lengthLocal) const;
};

Tolerance computeTolerance(
    std::vector<Vec2> points,
    std::vector<double> segmentLengthsUnits,
    std::vector<double> contourElevations,
    std::vector<double> controlElevations,
    const Meta& meta,
    int doubleMantissaBits = 53);

std::string describeTolerance(const Tolerance& t);

} // namespace terrain
