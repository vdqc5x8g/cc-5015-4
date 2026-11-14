#include "terrain/tolerance.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>

namespace terrain {

namespace {

double percentile(std::vector<double> values, double q)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    const double pos = q * (values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    const double f = pos - lo;
    return values[lo] * (1.0 - f) + values[hi] * f;
}

std::string fmt(double v, int digits = 6)
{
    std::ostringstream os;
    os.precision(digits);
    os << v;
    return os.str();
}

double ulpAt(double value)
{
    const double a = std::abs(value);
    if (a == 0.0)
        return std::numeric_limits<double>::denorm_min();
    return std::nextafter(a, std::numeric_limits<double>::infinity()) - a;
}

} // namespace

Vec2 Tolerance::toLocal(const Vec2& p) const
{
    return {(p.x - origin.x) / xyScale, (p.y - origin.y) / xyScale};
}

Vec2 Tolerance::fromLocal(const Vec2& p) const
{
    return {p.x * xyScale + origin.x, p.y * xyScale + origin.y};
}

double Tolerance::lengthToLocal(double lengthUnits) const { return lengthUnits / xyScale; }
double Tolerance::lengthFromLocal(double lengthLocal) const { return lengthLocal * xyScale; }

Tolerance computeTolerance(
    std::vector<Vec2> points,
    std::vector<double> segmentLengthsUnits,
    std::vector<double> contourElevations,
    std::vector<double> controlElevations,
    const Meta& meta,
    int doubleMantissaBits)
{
    Tolerance t;
    t.xyUnitName = meta.xyUnitName;
    t.metersPerXyUnit = meta.xyMetersPerUnit > 0.0 ? meta.xyMetersPerUnit : 1.0;
    t.zUnitName = meta.zUnitName;
    t.metersPerZUnit = meta.zMetersPerUnit > 0.0 ? meta.zMetersPerUnit : 1.0;
    t.zDatum = meta.zDatum;

    BBox box;
    double maxAbs = 0.0;
    for (const Vec2& p : points) {
        box.include(p);
        maxAbs = std::max(maxAbs, std::max(std::abs(p.x), std::abs(p.y)));
    }
    if (!box.valid)
        box = BBox{0.0, 0.0, 1.0, 1.0, true};
    t.origin = box.center();
    t.extentUnits = std::max(box.width(), box.height());
    if (!(t.extentUnits > 0.0))
        t.extentUnits = 1.0;
    t.extentMeters = t.extentUnits * t.metersPerXyUnit;
    t.maxAbsCoordUnits = maxAbs;
    // Coordinates are translated to the dataset centre and divided by the
    // extent. The local frame is roughly [-0.5, 0.5] regardless of whether the
    // survey is at 0 m or 1 000 000 m easting.
    t.xyScale = t.extentUnits;
    t.inputUlpUnits = ulpAt(maxAbs);
    t.precisionFloorUnits = std::max(4.0 * t.inputUlpUnits, t.extentUnits / std::pow(2.0, doubleMantissaBits - 8));
    t.precisionFloorMeters = t.precisionFloorUnits * t.metersPerXyUnit;

    // Robust low-percentile nearest-neighbour spacing. Sampling bounds the cost
    // for large inputs while still exposing duplicate clusters (distance 0).
    std::vector<double> spacings;
    std::mt19937 rng(12345);
    std::vector<std::size_t> indices(points.size());
    for (std::size_t i = 0; i < indices.size(); ++i)
        indices[i] = i;
    const std::size_t sampleCount = std::min<std::size_t>(points.size(), 4000);
    if (points.size() > sampleCount) {
        std::shuffle(indices.begin(), indices.end(), rng);
        indices.resize(sampleCount);
    }
    for (std::size_t ia = 0; ia < indices.size(); ++ia) {
        const Vec2 p = t.toLocal(points[indices[ia]]);
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t ib = 0; ib < indices.size(); ++ib) {
            if (ia == ib)
                continue;
            const double d = distance(p, t.toLocal(points[indices[ib]]));
            best = std::min(best, d);
        }
        if (std::isfinite(best))
            spacings.push_back(best);
    }
    // Coincident vertices produce a nearest-neighbour distance of zero. A few
    // duplicates must not collapse the spacing estimate (and with it the snap
    // and short-segment thresholds), so points far closer than the typical
    // neighbour distance are excluded from the low-percentile estimate. The
    // duplicates themselves are reported separately by validation.
    std::vector<double> positive;
    for (double d : spacings)
        if (d > 0.0)
            positive.push_back(d);
    if (!positive.empty()) {
        const double typical = percentile(positive, 0.5);
        const double floorDist = std::max(typical * 0.1, t.precisionFloorUnits * 8.0);
        std::vector<double> usable;
        for (double d : positive)
            if (d >= floorDist)
                usable.push_back(d);
        if (usable.empty())
            usable = positive;
        t.spacingMedianUnits = percentile(usable, 0.5) * t.xyScale;
        t.spacingP05Units = percentile(usable, 0.05) * t.xyScale;
    }
    t.spacingP05Meters = t.spacingP05Units * t.metersPerXyUnit;

    std::vector<double> lensLocal;
    lensLocal.reserve(segmentLengthsUnits.size());
    for (double v : segmentLengthsUnits)
        lensLocal.push_back(v / t.xyScale);
    t.medianSegmentUnits = percentile(lensLocal, 0.5) * t.xyScale;

    double shortUnits = t.spacingP05Units > 0.0 ? t.spacingP05Units * 0.15 : t.extentUnits * 1e-4;
    t.shortSegmentUnits = std::max(shortUnits, t.precisionFloorUnits);
    t.shortSegmentMeters = t.shortSegmentUnits * t.metersPerXyUnit;

    // Detection/welding tolerance. This is NOT a fraction of point spacing:
    // distinct survey vertices are often one spacing apart and must never be
    // merged. It is a small fraction of the *local* spacing, floored by what
    // double precision can resolve at this coordinate magnitude and capped by
    // the extent. 1e-3 of the low-percentile spacing means a 10 m survey
    // spacing yields a 1 cm weld radius, while a centimetre-spacing survey
    // yields 0.01 mm.
    double snapUnits = t.spacingP05Units > 0.0 ? t.spacingP05Units * 1e-3 : t.extentUnits * 1e-7;
    snapUnits = std::max(snapUnits, t.precisionFloorUnits);
    if (meta.userSnapToleranceMeters > 0.0) {
        double user = meta.userSnapToleranceMeters / t.metersPerXyUnit;
        if (user < t.precisionFloorUnits) {
            t.userToleranceCapped = true;
            user = t.precisionFloorUnits;
        }
        snapUnits = user;
    }
    // A weld radius above 0.1% of the extent would start collapsing real
    // geometry; enforce it as a hard ceiling.
    snapUnits = std::min(snapUnits, t.extentUnits * 1e-3);
    snapUnits = std::max(snapUnits, t.precisionFloorUnits);
    t.snapTolUnits = snapUnits;
    t.snapTolMeters = snapUnits * t.metersPerXyUnit;

    std::vector<double> levels = contourElevations;
    std::sort(levels.begin(), levels.end());
    double minLevelGap = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < levels.size(); ++i) {
        const double d = levels[i] - levels[i - 1];
        if (d > 0.0)
            minLevelGap = std::min(minLevelGap, d);
    }
    t.minElevationGap = std::isfinite(minLevelGap) ? minLevelGap : 0.0;

    double zUnits = 0.0;
    if (meta.userZNearToleranceMeters > 0.0) {
        zUnits = meta.userZNearToleranceMeters / t.metersPerZUnit;
    } else if (t.minElevationGap > 0.0) {
        zUnits = t.minElevationGap * 0.25;
    } else {
        std::vector<double> elevs = contourElevations;
        elevs.insert(elevs.end(), controlElevations.begin(), controlElevations.end());
        double maxAbsZ = 0.0;
        for (double z : elevs)
            maxAbsZ = std::max(maxAbsZ, std::abs(z));
        zUnits = std::max(4.0 * ulpAt(maxAbsZ), t.metersPerZUnit * 1e-9);
    }
    t.zTolUnits = std::max(zUnits, 4.0 * ulpAt(t.minElevationGap));
    t.zTolMeters = t.zTolUnits * t.metersPerZUnit;

    t.precisionLimited = t.maxAbsCoordUnits > 0.0 &&
                         t.precisionFloorUnits / std::max(t.extentUnits, 1e-300) > 1e-12;

    std::ostringstream why;
    why << "snap " << fmt(t.snapTolMeters) << " " << t.xyUnitName
        << " = max(precision floor " << fmt(t.precisionFloorMeters) << ", 0.001 x P05 spacing "
        << fmt(t.spacingP05Meters) << " " << t.xyUnitName << ")";
    t.explanation = why.str();
    return t;
}

std::string describeTolerance(const Tolerance& t)
{
    std::ostringstream os;
    os << "XY unit: " << t.xyUnitName << " (1 " << t.xyUnitName << " = " << t.metersPerXyUnit << " m)\n";
    os << "Elevation unit: " << t.zUnitName << " (1 " << t.zUnitName << " = " << t.metersPerZUnit << " m)\n";
    os << "Elevation datum: " << t.zDatum << "\n";
    os << "Local origin: (" << fmt(t.origin.x, 9) << ", " << fmt(t.origin.y, 9) << ") " << t.xyUnitName << "\n";
    os << "Local scale: " << fmt(t.xyScale, 6) << " " << t.xyUnitName << " -> 1 (coordinates normalized to unit extent)\n";
    os << "Extent: " << fmt(t.extentMeters, 3) << " m; max |coordinate|: " << fmt(t.maxAbsCoordUnits, 3) << " " << t.xyUnitName << "\n";
    os << "Snap tolerance: " << fmt(t.snapTolMeters) << " " << t.xyUnitName
       << " (" << fmt(t.snapTolUnits) << " local units)\n";
    os << "Elevation equality tolerance: " << fmt(t.zTolUnits) << " " << t.zUnitName << "\n";
    os << "Short-segment threshold: " << fmt(t.shortSegmentMeters) << " " << t.xyUnitName << "\n";
    os << "P05 point spacing: " << fmt(t.spacingP05Meters) << " " << t.xyUnitName
       << "; median segment: " << fmt(t.medianSegmentUnits) << " " << t.xyUnitName << "\n";
    os << "Precision floor: " << fmt(t.precisionFloorMeters) << " " << t.xyUnitName
       << (t.precisionLimited ? " (limited by coordinate magnitude)" : "") << "\n";
    if (t.userToleranceCapped)
        os << "Note: requested tolerance was raised to the precision floor; double cannot resolve finer at this coordinate magnitude.\n";
    return os.str();
}

} // namespace terrain
