#pragma once

#include <cstddef>
#include <vector>

namespace terrain {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(const Vec2& a, double s) { return {a.x * s, a.y * s}; }

double dot(const Vec2& a, const Vec2& b);
double cross(const Vec2& a, const Vec2& b);
double norm(const Vec2& a);
double distance(const Vec2& a, const Vec2& b);
double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b);

struct BBox {
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    bool valid = false;

    void include(const Vec2& p);
    void include(const BBox& other);
    double width() const;
    double height() const;
    Vec2 center() const;
    Vec2 minCorner() const;
    Vec2 maxCorner() const;
};

using Polyline = std::vector<Vec2>;
using Ring = Polyline;

BBox bboxOf(const Polyline& line);

bool pointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b, double tol);
bool pointInRing(const Vec2& p, const Ring& ring, double tol, bool* onBoundary = nullptr);
double ringSignedArea(const Ring& ring);
bool ringIsCCW(const Ring& ring);

enum class DomainLoc {
    Outside,
    Inside,
    Boundary,
    HoleInside,
    HoleBoundary,
};

DomainLoc classifyPoint(const Vec2& p, const Ring& outer, const std::vector<Ring>& holes, double tol);
const char* domainLocName(DomainLoc loc);

struct SegmentHit {
    enum Kind { None, Point, Overlap } kind = None;
    Vec2 point;
    Vec2 overlapA;
    Vec2 overlapB;
    double t = 0.0;
    double u = 0.0;
    bool interiorAB = false;
    bool interiorCD = false;
};

SegmentHit intersectSegments(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, double tol);

struct SelfHit {
    int segA = -1;
    int segB = -1;
    Vec2 point;
    bool overlap = false;
    bool interior = false;
};

std::vector<SelfHit> findSelfIntersections(const Polyline& line, bool closed, double tol);

struct PolylinePiece {
    Polyline points;
    bool closed = false;
};

std::vector<PolylinePiece> clipPolylineToDomain(
    const Polyline& line,
    bool closed,
    const Ring& outer,
    const std::vector<Ring>& holes,
    double tol);

bool polylinesEquivalent(const Polyline& a, const Polyline& b, bool closed, double tol);

Polyline dedupeConsecutive(const Polyline& line, double tol, bool* changed = nullptr);

std::vector<Polyline> splitPolylineAtSelfIntersections(const Polyline& line, bool closed, double tol);

} // namespace terrain
