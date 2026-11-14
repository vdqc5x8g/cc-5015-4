#include "terrain/geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace terrain {

double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
double cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
double norm(const Vec2& a) { return std::sqrt(dot(a, a)); }
double distance(const Vec2& a, const Vec2& b) { return norm(a - b); }

double pointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b)
{
    const Vec2 ab = b - a;
    const double len2 = dot(ab, ab);
    if (len2 == 0.0)
        return distance(p, a);
    double t = dot(p - a, ab) / len2;
    t = std::max(0.0, std::min(1.0, t));
    return distance(p, a + ab * t);
}

void BBox::include(const Vec2& p)
{
    if (!valid) {
        minX = maxX = p.x;
        minY = maxY = p.y;
        valid = true;
        return;
    }
    minX = std::min(minX, p.x);
    minY = std::min(minY, p.y);
    maxX = std::max(maxX, p.x);
    maxY = std::max(maxY, p.y);
}

void BBox::include(const BBox& other)
{
    if (!other.valid)
        return;
    include(Vec2{other.minX, other.minY});
    include(Vec2{other.maxX, other.maxY});
}

double BBox::width() const { return valid ? maxX - minX : 0.0; }
double BBox::height() const { return valid ? maxY - minY : 0.0; }
Vec2 BBox::center() const { return {0.5 * (minX + maxX), 0.5 * (minY + maxY)}; }
Vec2 BBox::minCorner() const { return {minX, minY}; }
Vec2 BBox::maxCorner() const { return {maxX, maxY}; }

BBox bboxOf(const Polyline& line)
{
    BBox box;
    for (const Vec2& p : line)
        box.include(p);
    return box;
}

bool pointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b, double tol)
{
    if (pointSegmentDistance(p, a, b) > tol)
        return false;
    return p.x >= std::min(a.x, b.x) - tol && p.x <= std::max(a.x, b.x) + tol &&
           p.y >= std::min(a.y, b.y) - tol && p.y <= std::max(a.y, b.y) + tol;
}

double ringSignedArea(const Ring& ring)
{
    double area = 0.0;
    if (ring.size() < 3)
        return 0.0;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const Vec2& a = ring[i];
        const Vec2& b = ring[(i + 1) % ring.size()];
        area += cross(a, b);
    }
    return 0.5 * area;
}

bool ringIsCCW(const Ring& ring) { return ringSignedArea(ring) > 0.0; }

namespace {

// Signed distance sign test using the even-odd rule. Points on the boundary are
// detected separately with pointOnSegment so that a tiny tolerance is used for
// equality while the inside test stays purely combinatorial.
int ringParity(const Vec2& p, const Ring& ring)
{
    int parity = 0;
    const std::size_t n = ring.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = ring[j];
        const Vec2& b = ring[i];
        const double dy = b.y - a.y;
        if (dy == 0.0)
            continue;
        if ((b.y > p.y) == (a.y > p.y))
            continue;
        const double xCross = a.x + (p.y - a.y) * (b.x - a.x) / dy;
        if (p.x < xCross)
            parity ^= 1;
    }
    return parity;
}

bool ringBoundary(const Vec2& p, const Ring& ring, double tol)
{
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (pointOnSegment(p, ring[i], ring[(i + 1) % n], tol))
            return true;
    }
    return false;
}

} // namespace

bool pointInRing(const Vec2& p, const Ring& ring, double tol, bool* onBoundary)
{
    if (ring.size() < 3)
        return false;
    if (ringBoundary(p, ring, tol)) {
        if (onBoundary)
            *onBoundary = true;
        return true;
    }
    if (onBoundary)
        *onBoundary = false;
    return ringParity(p, ring) != 0;
}

DomainLoc classifyPoint(const Vec2& p, const Ring& outer, const std::vector<Ring>& holes, double tol)
{
    bool outerBoundary = false;
    const bool inOuter = pointInRing(p, outer, tol, &outerBoundary);
    if (outerBoundary)
        return DomainLoc::Boundary;
    if (!inOuter)
        return DomainLoc::Outside;
    for (const Ring& hole : holes) {
        bool holeBoundary = false;
        const bool inHole = pointInRing(p, hole, tol, &holeBoundary);
        if (holeBoundary)
            return DomainLoc::HoleBoundary;
        if (inHole)
            return DomainLoc::HoleInside;
    }
    return DomainLoc::Inside;
}

const char* domainLocName(DomainLoc loc)
{
    switch (loc) {
    case DomainLoc::Outside: return "outside";
    case DomainLoc::Inside: return "inside";
    case DomainLoc::Boundary: return "boundary";
    case DomainLoc::HoleInside: return "hole_inside";
    case DomainLoc::HoleBoundary: return "hole_boundary";
    }
    return "unknown";
}

SegmentHit intersectSegments(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, double tol)
{
    SegmentHit hit;
    const Vec2 r = b - a;
    const Vec2 s = d - c;
    const Vec2 ca = c - a;
    const double rr = dot(r, r);
    const double ss = dot(s, s);
    // A segment shorter than the tolerance has no reliable direction; treat it
    // as absent. Such vertices are reported by the duplicate/short-segment
    // checks instead of masquerading as collinear overlaps.
    if (rr <= tol * tol || ss <= tol * tol)
        return hit;
    const double rxs = cross(r, s);
    if (rxs == 0.0 || std::abs(rxs) <= 1e-14 * (norm(r) * norm(s))) {
        // Parallel or degenerate: test collinear overlap by perpendicular
        // distance, so a short segment cannot flag distant lines as collinear.
        const double perp = std::abs(cross(ca, r)) / norm(r);
        if (perp > tol)
            return hit;
        double t0 = dot(c - a, r) / rr;
        double t1 = dot(d - a, r) / rr;
        if (t0 > t1)
            std::swap(t0, t1);
        const double lo = std::max(0.0, t0);
        const double hi = std::min(1.0, t1);
        const double tolT = tol / norm(r);
        if (hi <= lo + tolT)
            return hit;
        hit.kind = SegmentHit::Overlap;
        hit.overlapA = a + r * lo;
        hit.overlapB = a + r * hi;
        hit.point = hit.overlapA;
        hit.t = lo;
        hit.interiorAB = lo > 0.0;
        hit.interiorCD = false;
        return hit;
    }
    const double t = cross(ca, s) / rxs;
    const double u = cross(ca, r) / rxs;
    const double tolT = tol / std::max(norm(r), tol);
    const double tolU = tol / std::max(norm(s), tol);
    if (t < -tolT || t > 1.0 + tolT || u < -tolU || u > 1.0 + tolU)
        return hit;
    const double tc = std::max(0.0, std::min(1.0, t));
    const double uc = std::max(0.0, std::min(1.0, u));
    if (std::abs(rxs) > 0.0 && distance(a + r * tc, c + s * uc) > tol)
        return hit;
    hit.kind = SegmentHit::Point;
    hit.point = a + r * tc;
    hit.t = tc;
    hit.u = uc;
    // Only strictly-interior crossings can be classified as interior; touching
    // at an endpoint is a shared vertex, not a crossing.
    const double endEps = tolT;
    hit.interiorAB = tc > endEps && tc < 1.0 - endEps;
    hit.interiorCD = uc > tolU && uc < 1.0 - tolU;
    return hit;
}

std::vector<SelfHit> findSelfIntersections(const Polyline& line, bool closed, double tol)
{
    std::vector<SelfHit> hits;
    const int n = static_cast<int>(line.size());
    if (n < 2)
        return hits;
    const int segCount = closed ? n : n - 1;
    for (int i = 0; i < segCount; ++i) {
        const Vec2& a = line[i];
        const Vec2& b = line[(i + 1) % n];
        for (int j = i + 1; j < segCount; ++j) {
            // Adjacent segments share an endpoint by construction.
            if (closed && i == 0 && j == segCount - 1)
                continue;
            if (j == i + 1)
                continue;
            const Vec2& c = line[j];
            const Vec2& d = line[(j + 1) % n];
            const SegmentHit h = intersectSegments(a, b, c, d, tol);
            if (h.kind == SegmentHit::None)
                continue;
            // Non-adjacent segments that meet — even at an endpoint — make the
            // polyline non-simple: a vertex lying on another segment (a T
            // junction) or a revisited vertex. Both must be reported because
            // the triangulation cannot constrain a T-junction as one edge.
            SelfHit sh;
            sh.segA = i;
            sh.segB = j;
            sh.point = h.point;
            sh.overlap = h.kind == SegmentHit::Overlap;
            sh.interior = h.interiorAB && h.interiorCD;
            hits.push_back(sh);
        }
    }
    return hits;
}

std::vector<Polyline> splitPolylineAtSelfIntersections(const Polyline& line, bool closed, double tol)
{
    const std::vector<SelfHit> hits = findSelfIntersections(line, closed, tol);
    if (hits.empty())
        return {line};
    // Overlapping collinear runs cannot be untangled by cutting; leave the
    // line intact so validation keeps reporting it as unresolved.
    for (const SelfHit& h : hits)
        if (h.overlap)
            return {line};

    const int n = static_cast<int>(line.size());
    const int segCount = closed ? n : n - 1;
    auto paramOf = [&](int seg, const Vec2& p) {
        const Vec2 a = line[seg];
        const Vec2 b = line[(seg + 1) % n];
        const Vec2 r = b - a;
        const double rr = dot(r, r);
        double t = rr > 0.0 ? dot(p - a, r) / rr : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        return static_cast<double>(seg) + t;
    };

    std::vector<double> params;
    for (const SelfHit& h : hits) {
        params.push_back(paramOf(h.segA, h.point));
        params.push_back(paramOf(h.segB, h.point));
    }
    params.push_back(0.0);
    params.push_back(static_cast<double>(segCount));
    std::sort(params.begin(), params.end());
    const double paramTol = 1e-9;
    std::vector<double> cuts;
    for (double v : params) {
        if (cuts.empty() || std::abs(v - cuts.back()) > paramTol)
            cuts.push_back(v);
    }
    if (cuts.size() < 3)
        return {line};

    auto pointAt = [&](double s) {
        const int seg = std::min(segCount - 1, std::max(0, static_cast<int>(std::floor(s))));
        const double local = s - seg;
        const Vec2 a = line[seg];
        const Vec2 b = line[(seg + 1) % n];
        return a + (b - a) * std::min(1.0, std::max(0.0, local));
    };

    // Walk consecutive cut parameters; every fragment lies between two
    // self-touch points, so it cannot contain the touch that created them.
    std::vector<Polyline> parts;
    for (std::size_t k = 0; k + 1 < cuts.size(); ++k) {
        const double s0 = cuts[k];
        const double s1 = cuts[k + 1];
        if (s1 - s0 <= paramTol)
            continue;
        Polyline piece;
        piece.push_back(pointAt(s0));
        // Interior source vertices strictly between the two cut points. The
        // bounds are exclusive, so a cut that coincides with a source vertex is
        // not emitted twice (which would create a zero-area loop).
        const int seg0 = static_cast<int>(std::floor(s0));
        const int seg1 = static_cast<int>(std::floor(s1));
        for (int seg = seg0; seg <= seg1; ++seg) {
            const double vertexParam = static_cast<double>(seg + 1);
            if (vertexParam <= s0 + paramTol || vertexParam >= s1 - paramTol)
                continue;
            if (seg + 1 >= segCount && !closed)
                break;
            piece.push_back(line[(seg + 1) % n]);
        }
        piece.push_back(pointAt(s1));
        if (piece.size() >= 2 && distance(piece.front(), piece.back()) > tol)
            parts.push_back(piece);
    }
    return parts.empty() ? std::vector<Polyline>{line} : parts;
}

Polyline dedupeConsecutive(const Polyline& line, double tol, bool* changed)
{
    Polyline out;
    if (line.empty())
        return out;
    out.push_back(line[0]);
    for (const Vec2& p : line) {
        if (distance(p, out.back()) > tol)
            out.push_back(p);
    }
    if (changed)
        *changed = out.size() != line.size();
    return out;
}

bool polylinesEquivalent(const Polyline& a, const Polyline& b, bool closed, double tol)
{
    if (a.size() != b.size())
        return false;
    if (closed) {
        // Closed contours may be stored with an arbitrary rotation and either
        // orientation; compare all rotations in both directions.
        const std::size_t n = a.size();
        for (std::size_t start = 0; start < n; ++start) {
            bool same = true;
            bool reversed = true;
            for (std::size_t i = 0; i < n && (same || reversed); ++i) {
                const Vec2& ai = a[i];
                if (distance(ai, b[(start + i) % n]) > tol)
                    same = false;
                if (distance(ai, b[(start + n - i) % n]) > tol)
                    reversed = false;
            }
            if (same || reversed)
                return true;
        }
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (distance(a[i], b[i]) > tol)
            return false;
    }
    return true;
}

std::vector<PolylinePiece> clipPolylineToDomain(
    const Polyline& line,
    bool closed,
    const Ring& outer,
    const std::vector<Ring>& holes,
    double tol)
{
    std::vector<PolylinePiece> pieces;
    if (line.size() < 2)
        return pieces;
    // Gather parameter values where the line crosses or touches the domain
    // boundary, then sample the interior of each interval. Sampling avoids
    // fragile on-boundary arithmetic and correctly splits at hole boundaries.
    const int n = static_cast<int>(line.size());
    const int segCount = closed ? n : n - 1;
    const double lineLen = 0.0 + n;
    std::vector<double> params;
    params.push_back(0.0);
    params.push_back(static_cast<double>(segCount));
    bool crossed = false;

    auto considerRing = [&](const Ring& ring) {
        const int m = static_cast<int>(ring.size());
        for (int i = 0; i < segCount; ++i) {
            const Vec2& a = line[i];
            const Vec2& b = line[(i + 1) % n];
            for (int k = 0; k < m; ++k) {
                const Vec2& c = ring[k];
                const Vec2& d = ring[(k + 1) % m];
                const SegmentHit h = intersectSegments(a, b, c, d, tol);
                if (h.kind == SegmentHit::Point) {
                    params.push_back(static_cast<double>(i) + h.t);
                    crossed = true;
                } else if (h.kind == SegmentHit::Overlap) {
                    const double len2 = std::max(dot(b - a, b - a), 1e-300);
                    const double t0 = dot(h.overlapA - a, b - a) / len2;
                    const double t1 = dot(h.overlapB - a, b - a) / len2;
                    params.push_back(static_cast<double>(i) + std::min(t0, t1));
                    params.push_back(static_cast<double>(i) + std::max(t0, t1));
                    crossed = true;
                }
            }
        }
    };
    considerRing(outer);
    for (const Ring& h : holes)
        considerRing(h);

    std::sort(params.begin(), params.end());
    const double paramTol = tol / std::max(1e-300, lineLen);
    std::vector<double> merged;
    for (double v : params) {
        if (merged.empty() || std::abs(v - merged.back()) > paramTol)
            merged.push_back(v);
    }
    params.swap(merged);

    auto pointAt = [&](double s) {
        const int seg = std::min(segCount - 1, std::max(0, static_cast<int>(std::floor(s))));
        const double local = s - seg;
        const Vec2 a = line[seg];
        const Vec2 b = line[(seg + 1) % n];
        return a + (b - a) * std::min(1.0, std::max(0.0, local));
    };

    std::vector<int> inside(static_cast<int>(params.size()) - 1, 0);
    for (std::size_t k = 0; k + 1 < params.size(); ++k) {
        const double mid = 0.5 * (params[k] + params[k + 1]);
        const int seg = std::min(segCount - 1, std::max(0, static_cast<int>(std::floor(mid))));
        const double local = mid - seg;
        const Vec2 a = line[seg];
        const Vec2 b = line[(seg + 1) % n];
        const Vec2 p = a + (b - a) * std::min(1.0, std::max(0.0, local));
        const DomainLoc loc = classifyPoint(p, outer, holes, tol);
        // Boundary samples are kept: a contour running along the outer edge is
        // still a legitimate elevation constraint. Hole boundaries are skipped
        // because the hole ring itself already constrains the same edge.
        inside[k] = (loc == DomainLoc::Inside || loc == DomainLoc::Boundary) ? 1 : 0;
    }

    Polyline cur;
    bool curOpen = false;
    for (std::size_t k = 0; k + 1 < params.size(); ++k) {
        if (inside[k]) {
            if (!curOpen) {
                cur.clear();
                cur.push_back(pointAt(params[k]));
                curOpen = true;
            }
            cur.push_back(pointAt(params[k + 1]));
        } else if (curOpen) {
            if (cur.size() >= 2 && distance(cur.front(), cur.back()) > tol)
                pieces.push_back({cur, false});
            cur.clear();
            curOpen = false;
        }
    }
    if (curOpen && cur.size() >= 2 && distance(cur.front(), cur.back()) > tol)
        pieces.push_back({cur, false});

    // A closed contour entirely inside the domain stays closed. A contour that
    // touches the boundary is returned as open pieces so that later stages do
    // not accidentally re-close it across the outside region.
    if (closed && !crossed && pieces.size() == 1) {
        Polyline closedPts = pieces.front().points;
        if (distance(closedPts.front(), closedPts.back()) <= tol)
            closedPts.pop_back();
        if (classifyPoint(closedPts.front(), outer, holes, tol) == DomainLoc::Inside)
            return {{closedPts, true}};
    }
    return pieces;
}

} // namespace terrain
