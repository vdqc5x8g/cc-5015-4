#include "terrain/mesh.h"

#include "CDT.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace terrain {

namespace {

using Clock = std::chrono::steady_clock;

double msSince(Clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

std::string num(double v, int digits = 8)
{
    std::ostringstream os;
    os.precision(digits);
    os << v;
    return os.str();
}

struct SourceLine {
    enum Kind { Boundary, Hole, Contour, Fault };
    Kind kind = Boundary;
    int owner = -1;
    bool closed = false;
    bool hasZ = false;
    double z = 0.0;
    Polyline points;
};

struct Cut {
    double t = 0.0;
    Vec2 p;
};

struct PointSlot {
    Vec2 p;
    bool hasZ = false;
    double z = 0.0;
    bool observed = false;
    bool onBoundary = false;
    bool onHole = false;
    bool onFault = false;
    int contourId = -1;
    int controlId = -1;
};

constexpr long long kHashX = 73856093LL;
constexpr long long kHashY = 19349663LL;

class WeldIndex {
public:
    WeldIndex(double cell, double snap, double exactEps, double zTol)
        : cell_(std::max(cell, 1e-300)), snap_(snap), exactEps_(exactEps), zTol_(zTol)
    {
    }

    // Returns index of the welded point, or -1 on a genuine conflict.
    // Matching rules:
    //  * within exactEps with a different elevation -> conflict (same place,
    //    two elevations: cannot be merged),
    //  * within snap with equal elevation -> merge,
    //  * within snap with different elevation -> keep separate; the caller's
    //    crossing checks decide whether that is legal.
    int insert(const Vec2& p, const PointSlot& meta, std::vector<PointSlot>& slots, std::string* error)
    {
        int same = -1;
        double sameDist = snap_;
        int exactConflict = -1;
        forEachNear(p, [&](int idx) {
            const PointSlot& q = slots[idx];
            const double d = distance(p, q.p);
            if (d > snap_)
                return;
            const bool zComparable = meta.hasZ && q.hasZ;
            if (zComparable && std::abs(meta.z - q.z) > zTol_) {
                if (d <= exactEps_)
                    exactConflict = idx;
                return;
            }
            if (d < sameDist || (d == sameDist && same < 0)) {
                sameDist = d;
                same = idx;
            }
        });
        if (same >= 0 && exactConflict >= 0) {
            // A same-elevation neighbour and a different-elevation neighbour
            // both sit within the snap radius: ambiguous.
            if (error)
                *error = "ambiguous elevation at (" + num(p.x) + ", " + num(p.y) + "): nearby vertices disagree";
            return -1;
        }
        if (exactConflict >= 0) {
            if (error) {
                *error = "two elevations at the same location (" + num(p.x) + ", " + num(p.y) + "): " +
                         num(slots[exactConflict].z) + " and " + num(meta.z) +
                         "; they cannot share a vertex";
            }
            return -1;
        }
        if (same >= 0) {
            PointSlot& q = slots[same];
            if (meta.hasZ && !q.hasZ) {
                q.z = meta.z;
                q.hasZ = true;
            }
            q.observed = q.observed || meta.observed;
            q.onBoundary = q.onBoundary || meta.onBoundary;
            q.onHole = q.onHole || meta.onHole;
            q.onFault = q.onFault || meta.onFault;
            if (meta.contourId >= 0 && q.contourId < 0)
                q.contourId = meta.contourId;
            if (meta.controlId >= 0)
                q.controlId = meta.controlId;
            return same;
        }
        const int idx = static_cast<int>(slots.size());
        slots.push_back(meta);
        slots.back().p = p;
        grid_[cellOf(p)].push_back(idx);
        return idx;
    }

private:
    long long cellOf(const Vec2& p) const
    {
        const long long ix = static_cast<long long>(std::floor(p.x / cell_));
        const long long iy = static_cast<long long>(std::floor(p.y / cell_));
        return ix * kHashX ^ iy * kHashY;
    }
    template <typename F>
    void forEachNear(const Vec2& p, F&& f) const
    {
        const long long cx = static_cast<long long>(std::floor(p.x / cell_));
        const long long cy = static_cast<long long>(std::floor(p.y / cell_));
        for (long long dx = -1; dx <= 1; ++dx) {
            for (long long dy = -1; dy <= 1; ++dy) {
                auto it = grid_.find((cx + dx) * kHashX ^ (cy + dy) * kHashY);
                if (it == grid_.end())
                    continue;
                for (int idx : it->second)
                    f(idx);
            }
        }
    }

    double cell_;
    double snap_;
    double exactEps_;
    double zTol_;
    std::unordered_map<long long, std::vector<int>> grid_;
};

// Fills unknown vertices of one chain between observed vertices by linear
// interpolation in arc length measured along that chain. Works for open and
// closed chains. Vertices without an observed vertex on one side are left
// untouched for the harmonic pass.
void fillChainByArcLength(
    const std::vector<int>& ids,
    bool closed,
    std::vector<MeshVertex>& verts,
    const Tolerance& tol)
{
    int n = static_cast<int>(ids.size());
    if (n < 2)
        return;
    if (closed && n >= 2 && ids.front() == ids.back())
        --n; // drop duplicated closing vertex
    if (n < 2)
        return;

    std::vector<double> cum(n + 1, 0.0);
    for (int i = 0; i < n; ++i) {
        const int next = (i + 1) % n;
        if (!closed && i + 1 >= n)
            break;
        const Vec2 a = tol.toLocal(verts[ids[i]].position);
        const Vec2 b = tol.toLocal(verts[ids[next]].position);
        cum[i + 1] = cum[i] + distance(a, b);
    }
    const double perimeter = cum[closed ? n : n - 1];
    if (!(perimeter > 0.0))
        return;

    auto arcBetween = [&](int from, int to) {
        // forward distance from index `from` to index `to` on the loop
        if (to >= from)
            return cum[to] - cum[from];
        return (cum[n] - cum[from]) + cum[to];
    };

    std::vector<int> observed;
    for (int i = 0; i < n; ++i)
        if (verts[ids[i]].source == "observed")
            observed.push_back(i);
    if (observed.empty())
        return;

    if (!closed) {
        for (std::size_t k = 0; k + 1 < observed.size(); ++k) {
            const int a = observed[k];
            const int b = observed[k + 1];
            const double total = cum[b] - cum[a];
            if (total <= 0.0)
                continue;
            const double za = verts[ids[a]].elevation;
            const double zb = verts[ids[b]].elevation;
            for (int i = a + 1; i < b; ++i) {
                if (verts[ids[i]].source == "observed")
                    continue;
                const double t = (cum[i] - cum[a]) / total;
                verts[ids[i]].elevation = za + (zb - za) * t;
                verts[ids[i]].source = "constraint_edge";
            }
        }
        return;
    }

    // Closed chain: between consecutive observed indices, walking forward.
    for (std::size_t k = 0; k < observed.size(); ++k) {
        const int a = observed[k];
        const int b = observed[(k + 1) % observed.size()];
        if (a == b)
            continue;
        const double total = arcBetween(a, b);
        if (total <= 0.0)
            continue;
        const double za = verts[ids[a]].elevation;
        const double zb = verts[ids[b]].elevation;
        int i = (a + 1) % n;
        while (i != b) {
            if (verts[ids[i]].source != "observed") {
                const double t = arcBetween(a, i) / total;
                verts[ids[i]].elevation = za + (zb - za) * t;
                verts[ids[i]].source = "constraint_edge";
            }
            i = (i + 1) % n;
        }
    }
}

} // namespace

MeshBuildResult buildMesh(const DataInput& input, const Tolerance& tol, const MeshBuildOptions& options)
{
    (void)options;
    MeshBuildResult result;
    const auto tStart = Clock::now();
    const double snap = tol.snapTolUnits;
    const double scale = tol.xyScale > 0.0 ? tol.xyScale : 1.0;
    const double snapLocal = snap / scale;
    const double metersPerUnit = tol.metersPerXyUnit;

    // ------------------------------------------------------------ source lines
    std::vector<SourceLine> lines;
    {
        SourceLine boundary;
        boundary.kind = SourceLine::Boundary;
        boundary.closed = true;
        boundary.points = input.outer;
        // A boundary ring is always closed. If the input repeats the first
        // vertex at the end, drop the duplicate; otherwise the closing segment
        // is implied (validation reports this as a repair candidate).
        if (boundary.points.size() >= 2 && distance(boundary.points.front(), boundary.points.back()) <= snap)
            boundary.points.pop_back();
        if (boundary.points.size() < 3) {
            result.error = "boundary is missing or has fewer than three distinct vertices";
            return result;
        }
        lines.push_back(boundary);
    }
    for (std::size_t hi = 0; hi < input.holes.size(); ++hi) {
        SourceLine hole;
        hole.kind = SourceLine::Hole;
        hole.owner = static_cast<int>(hi);
        hole.closed = true;
        hole.points = input.holes[hi];
        if (hole.points.size() >= 2 && distance(hole.points.front(), hole.points.back()) <= snap)
            hole.points.pop_back();
        if (hole.points.size() < 3) {
            result.error = "hole #" + std::to_string(hi + 1) + " has fewer than three distinct vertices";
            return result;
        }
        lines.push_back(hole);
    }
    for (std::size_t ci = 0; ci < input.contours.size(); ++ci) {
        const ContourInput& c = input.contours[ci];
        if (c.points.size() < 2) {
            result.error = "contour " + std::to_string(c.id) + " has fewer than two vertices";
            return result;
        }
        if (c.closed && c.points.size() < 3) {
            result.error = "closed contour " + std::to_string(c.id) + " needs at least three vertices";
            return result;
        }
        const std::vector<SelfHit> selfHits = findSelfIntersections(c.points, c.closed, snap);
        if (!selfHits.empty()) {
            result.error = "contour " + std::to_string(c.id) + " (elevation " + num(c.elevation) +
                           ") still self-intersects at (" + num(selfHits.front().point.x) + ", " +
                           num(selfHits.front().point.y) + "); confirm a repair before building";
            result.notes.push_back({"mesh.self_intersection", "error", result.error, "contour", c.id, -1,
                                    {selfHits.front().point}, true, {}});
            return result;
        }
        SourceLine line;
        line.kind = SourceLine::Contour;
        line.owner = c.id;
        line.closed = c.closed;
        line.hasZ = true;
        line.z = c.elevation;
        line.points = c.points;
        lines.push_back(line);
    }
    for (const FaultInput& f : input.faults) {
        if (f.points.size() < 2) {
            result.error = "breakline " + std::to_string(f.id) + " has fewer than two vertices";
            return result;
        }
        const std::vector<SelfHit> selfHits = findSelfIntersections(f.points, false, snap);
        if (!selfHits.empty()) {
            result.error = "breakline " + std::to_string(f.id) + " self-intersects near (" +
                           num(selfHits.front().point.x) + ", " + num(selfHits.front().point.y) + ")";
            result.notes.push_back({"mesh.breakline_self_intersection", "error", result.error, "fault", f.id, -1,
                                    {selfHits.front().point}, true, {}});
            return result;
        }
        SourceLine line;
        line.kind = SourceLine::Fault;
        line.owner = f.id;
        line.closed = false;
        line.hasZ = false;
        line.points = f.points;
        lines.push_back(line);
    }

    // --------------------------------------------------------- crossing splits
    struct LineSegRef {
        int line;
        int seg;
    };
    std::vector<LineSegRef> segs;
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const SourceLine& L = lines[li];
        const int count = L.closed ? static_cast<int>(L.points.size()) : static_cast<int>(L.points.size()) - 1;
        for (int s = 0; s < count; ++s)
            segs.push_back({static_cast<int>(li), s});
    }
    // cuts[line][segment] = intersections found on that segment
    std::vector<std::vector<std::vector<Cut>>> cuts(lines.size());
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const std::size_t segCount = lines[i].closed ? lines[i].points.size() : lines[i].points.size() - 1;
        cuts[i].resize(segCount);
    }

    for (std::size_t ia = 0; ia < segs.size(); ++ia) {
        for (std::size_t ib = ia + 1; ib < segs.size(); ++ib) {
            const SourceLine& A = lines[segs[ia].line];
            const SourceLine& B = lines[segs[ib].line];
            if (segs[ia].line == segs[ib].line)
                continue;
            const int sa = segs[ia].seg;
            const int sb = segs[ib].seg;
            const Vec2 a0 = A.points[sa];
            const Vec2 a1 = A.points[(sa + 1) % A.points.size()];
            const Vec2 b0 = B.points[sb];
            const Vec2 b1 = B.points[(sb + 1) % B.points.size()];
            if (std::max(a0.x, a1.x) < std::min(b0.x, b1.x) - snap ||
                std::max(b0.x, b1.x) < std::min(a0.x, a1.x) - snap ||
                std::max(a0.y, a1.y) < std::min(b0.y, b1.y) - snap ||
                std::max(b0.y, b1.y) < std::min(a0.y, a1.y) - snap)
                continue;
            const SegmentHit h = intersectSegments(a0, a1, b0, b1, snap * 0.25);
            if (h.kind == SegmentHit::None)
                continue;
            const bool sharedEnd = (distance(h.point, a0) <= snap || distance(h.point, a1) <= snap) &&
                                   (distance(h.point, b0) <= snap || distance(h.point, b1) <= snap);
            if (A.hasZ && B.hasZ && std::abs(A.z - B.z) > tol.zTolUnits) {
                if (sharedEnd)
                    continue;
                if (h.kind == SegmentHit::Point && !h.interiorAB && !h.interiorCD)
                    continue;
                result.error = "constrained lines at different elevations cross near (" + num(h.point.x) + ", " +
                               num(h.point.y) + "): elevations " + num(A.z) + " and " + num(B.z) +
                               ". They were not merged; resolve the crossing (edit elevation, split or drop a line) first.";
                result.notes.push_back({"mesh.crossing_different_z", "error", result.error,
                                        A.kind == SourceLine::Contour ? "contour" : "fault", A.owner, -1,
                                        {h.point}, true, {}});
                result.totalMs = msSince(tStart);
                return result;
            }
            if (h.kind == SegmentHit::Overlap) {
                cuts[segs[ia].line][sa].push_back({0.0, h.overlapA});
                cuts[segs[ia].line][sa].push_back({1.0, h.overlapB});
                cuts[segs[ib].line][sb].push_back({0.0, h.overlapA});
                cuts[segs[ib].line][sb].push_back({1.0, h.overlapB});
                continue;
            }
            if (sharedEnd)
                continue;
            cuts[segs[ia].line][sa].push_back({h.t, h.point});
            cuts[segs[ib].line][sb].push_back({h.u, h.point});
        }
    }

    // -------------------------------------------------------------- weld points
    const double exactEps = std::max(tol.precisionFloorUnits / scale * 4.0, snapLocal * 1e-6);
    WeldIndex weld(std::max(snapLocal, 1e-12), snapLocal, exactEps, tol.zTolUnits);
    std::vector<PointSlot> slots;
    std::string weldError;

    struct Constraint {
        int a = -1;
        int b = -1;
        std::string kind;
        int owner = -1;
    };
    std::vector<Constraint> constraints;
    std::map<std::pair<int, int>, std::size_t> edgeIndex;
    std::vector<std::vector<int>> chains(lines.size());

    auto kindName = [](SourceLine::Kind k) {
        switch (k) {
        case SourceLine::Boundary: return "boundary";
        case SourceLine::Hole: return "hole";
        case SourceLine::Contour: return "contour";
        case SourceLine::Fault: return "fault";
        }
        return "boundary";
    };

    for (std::size_t li = 0; li < lines.size(); ++li) {
        const SourceLine& L = lines[li];
        const int n = static_cast<int>(L.points.size());
        const int segCount = L.closed ? n : n - 1;
        std::vector<int> chain;
        for (int s = 0; s < segCount; ++s) {
            const Vec2 a = L.points[s];
            const Vec2 b = L.points[(s + 1) % n];
            std::vector<Cut> cs = cuts[li][s];
            cs.push_back({0.0, a});
            cs.push_back({1.0, b});
            std::sort(cs.begin(), cs.end(), [](const Cut& x, const Cut& y) { return x.t < y.t; });
            std::vector<std::pair<Vec2, bool>> pts; // point, is original vertex
            for (const Cut& c : cs) {
                if (!pts.empty() && distance(pts.back().first, c.p) <= snap * 0.01)
                    continue;
                pts.push_back({c.p, false});
            }
            if (pts.empty())
                continue;
            pts.front().second = true;
            pts.back().second = true;
            for (std::size_t q = 0; q < pts.size(); ++q) {
                if (q == 0 && s > 0)
                    continue; // already emitted as the previous segment's end
                PointSlot meta;
                meta.hasZ = L.hasZ && pts[q].second;
                meta.z = L.z;
                meta.observed = L.hasZ && pts[q].second;
                meta.onBoundary = L.kind == SourceLine::Boundary;
                meta.onHole = L.kind == SourceLine::Hole;
                meta.onFault = L.kind == SourceLine::Fault;
                meta.contourId = L.kind == SourceLine::Contour ? L.owner : -1;
                const int vi = weld.insert(pts[q].first, meta, slots, &weldError);
                if (vi < 0) {
                    result.error = weldError;
                    result.notes.push_back({"mesh.weld_conflict", "error", weldError, kindName(L.kind), L.owner,
                                            -1, {pts[q].first}, true, {}});
                    result.totalMs = msSince(tStart);
                    return result;
                }
                if (chain.empty() || chain.back() != vi)
                    chain.push_back(vi);
            }
        }
        if (chain.size() < 2)
            continue;
        if (L.closed && chain.size() >= 3 && chain.front() == chain.back())
            chain.pop_back();
        for (std::size_t k = 0; k < chain.size(); ++k) {
            const int a = chain[k];
            const int b = chain[(k + 1) % chain.size()];
            if (!L.closed && k + 1 == chain.size())
                break;
            if (a == b)
                continue;
            const std::pair<int, int> key(std::min(a, b), std::max(a, b));
            auto it = edgeIndex.find(key);
            if (it == edgeIndex.end()) {
                edgeIndex[key] = constraints.size();
                constraints.push_back({key.first, key.second, kindName(L.kind), L.owner});
            } else {
                Constraint& existing = constraints[it->second];
                if ((L.kind == SourceLine::Boundary || L.kind == SourceLine::Hole) &&
                    (existing.kind == "contour" || existing.kind == "fault"))
                    existing.kind = kindName(L.kind);
            }
        }
        chains[li] = chain;
    }

    for (const ControlPoint& cp : input.controls) {
        PointSlot meta;
        meta.hasZ = true;
        meta.z = cp.elevation;
        meta.observed = true;
        meta.controlId = cp.id;
        const int vi = weld.insert(cp.position, meta, slots, &weldError);
        if (vi < 0) {
            result.error = "control point '" + cp.name + "' conflicts with the line data: " + weldError;
            result.notes.push_back({"mesh.control_conflict", "error", result.error, "control", cp.id, -1,
                                    {cp.position}, true, {}});
            result.totalMs = msSince(tStart);
            return result;
        }
        slots[vi].observed = true;
        slots[vi].controlId = cp.id;
    }
    // A control point that coincides with an unobserved cut point upgrades it.
    for (PointSlot& s : slots) {
        (void)s;
    }

    // ------------------------------------------------------------------- CDT
    const auto tTri0 = Clock::now();
    std::vector<CDT::V2d<double>> cdtVerts;
    cdtVerts.reserve(slots.size());
    for (const PointSlot& p : slots) {
        const Vec2 lp = tol.toLocal(p.p);
        cdtVerts.push_back(CDT::V2d<double>{lp.x, lp.y});
    }
    std::vector<CDT::Edge> cdtEdges;
    cdtEdges.reserve(constraints.size());
    for (const Constraint& c : constraints)
        cdtEdges.push_back(CDT::Edge(static_cast<CDT::VertInd>(c.a), static_cast<CDT::VertInd>(c.b)));

    CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::AsProvided,
                                  CDT::IntersectingConstraintEdges::NotAllowed,
                                  std::max(snapLocal * 0.5, 1e-14));
    try {
        cdt.insertVertices(cdtVerts);
        cdt.insertEdges(cdtEdges);
        // Interior contours and breaklines are hard constraints, but using
        // eraseOuterTrianglesAndHoles would treat every closed constraint ring
        // as a region boundary. Removing only the outside (crossing only the
        // outer ring) keeps interior constraints and the domain intact; holes
        // are cut out explicitly below and verified by area.
        cdt.eraseOuterTriangles();
    } catch (const std::exception& e) {
        result.error = std::string("constrained triangulation failed: ") + e.what();
        result.notes.push_back({"mesh.cdt_failure", "error", result.error, "boundary", -1, -1, {}, false, {}});
        result.totalMs = msSince(tStart);
        return result;
    }
    result.triangulationMs = msSince(tTri0);
    if (cdt.triangles.empty()) {
        result.error = "triangulation produced no triangles";
        result.totalMs = msSince(tStart);
        return result;
    }

    // ------------------------------------------------------------ mesh arrays
    std::vector<MeshVertex> verts(slots.size());
    for (std::size_t i = 0; i < slots.size(); ++i) {
        MeshVertex& mv = verts[i];
        mv.position = slots[i].p;
        mv.elevation = slots[i].hasZ ? slots[i].z : 0.0;
        mv.source = slots[i].observed ? "observed" : "unobserved";
        mv.observationId = slots[i].contourId >= 0 ? slots[i].contourId : slots[i].controlId;
    }

    std::vector<MeshTriangle> tris;
    tris.reserve(cdt.triangles.size());
    for (const CDT::Triangle& t : cdt.triangles) {
        MeshTriangle mt;
        for (int k = 0; k < 3; ++k)
            mt.v[k] = static_cast<int>(t.vertices[k]);
        const Vec2& a = verts[mt.v[0]].position;
        const Vec2& b = verts[mt.v[1]].position;
        const Vec2& c = verts[mt.v[2]].position;
        if (cross(b - a, c - a) < 0.0)
            std::swap(mt.v[1], mt.v[2]);
        tris.push_back(mt);
    }

    auto edgeKey = [](int a, int b) {
        const std::uint32_t lo = static_cast<std::uint32_t>(std::min(a, b));
        const std::uint32_t hi = static_cast<std::uint32_t>(std::max(a, b));
        return (static_cast<std::uint64_t>(lo) << 32) | hi;
    };
    std::unordered_set<std::uint64_t> meshEdges;
    for (const MeshTriangle& t : tris)
        for (int k = 0; k < 3; ++k)
            meshEdges.insert(edgeKey(t.v[k], t.v[(k + 1) % 3]));

    int missing = 0;
    for (const Constraint& c : constraints) {
        if (meshEdges.count(edgeKey(c.a, c.b)))
            continue;
        ++missing;
        if (missing <= 8) {
            const Vec2& pa = verts[c.a].position;
            const Vec2& pb = verts[c.b].position;
            result.notes.push_back({"mesh.constraint_missing", "error",
                                    "constraint edge (" + num(pa.x) + ", " + num(pa.y) + ")-(" + num(pb.x) + ", " +
                                        num(pb.y) + ") of kind '" + c.kind + "' is absent from the mesh",
                                    c.kind, c.owner, -1, {pa, pb}, false, {}});
        }
    }
    if (missing > 0) {
        result.error = std::to_string(missing) + " constraint edge(s) were not preserved";
        result.totalMs = msSince(tStart);
        return result;
    }

    int holeTriangles = 0;
    for (const Ring& hole : input.holes) {
        if (hole.size() < 3)
            continue;
        for (const MeshTriangle& t : tris) {
            const Vec2& a = verts[t.v[0]].position;
            const Vec2& b = verts[t.v[1]].position;
            const Vec2& c = verts[t.v[2]].position;
            const Vec2 centroid{(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0};
            if (pointInRing(centroid, hole, snap * 0.1))
                ++holeTriangles;
        }
    }
    if (holeTriangles > 0) {
        // Cut holes out only after the outer region has been removed. Triangle
        // centroids are used because the hole ring is itself a hard constraint,
        // so no triangle straddles its boundary.
        std::vector<MeshTriangle> kept;
        kept.reserve(tris.size());
        for (const MeshTriangle& t : tris) {
            const Vec2& a = verts[t.v[0]].position;
            const Vec2& b = verts[t.v[1]].position;
            const Vec2& c = verts[t.v[2]].position;
            const Vec2 centroid{(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0};
            bool inHole = false;
            for (const Ring& hole : input.holes) {
                if (hole.size() < 3)
                    continue;
                if (pointInRing(centroid, hole, snap * 0.1)) {
                    inHole = true;
                    break;
                }
            }
            if (!inHole)
                kept.push_back(t);
        }
        tris.swap(kept);
    }
    if (tris.empty()) {
        result.error = "all triangles were removed (boundary or holes cover the whole area)";
        result.totalMs = msSince(tStart);
        return result;
    }

    std::vector<std::vector<int>> adj(verts.size());
    for (const MeshTriangle& t : tris) {
        for (int k = 0; k < 3; ++k) {
            const int a = t.v[k];
            const int b = t.v[(k + 1) % 3];
            adj[a].push_back(b);
            adj[b].push_back(a);
        }
    }
    for (auto& list : adj) {
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
    }

    const auto tEle0 = Clock::now();

    // ------------------------------------- pass 1: arc length along source lines
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const SourceLine& L = lines[li];
        if (L.kind != SourceLine::Contour)
            continue; // breaklines carry no elevation observation of their own
        if (chains[li].size() < 2)
            continue;
        fillChainByArcLength(chains[li], L.closed, verts, tol);
    }

    // --------------------------------------------- pass 2: harmonic interpolation
    std::vector<char> known(verts.size(), 0);
    for (std::size_t i = 0; i < verts.size(); ++i)
        if (verts[i].source == "observed" || verts[i].source == "constraint_edge")
            known[i] = 1;

    std::vector<int> component(verts.size(), -1);
    int componentCount = 0;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (component[i] >= 0)
            continue;
        std::queue<int> q;
        q.push(static_cast<int>(i));
        component[i] = componentCount;
        while (!q.empty()) {
            const int u = q.front();
            q.pop();
            for (int nb : adj[u]) {
                if (component[nb] < 0) {
                    component[nb] = componentCount;
                    q.push(nb);
                }
            }
        }
        ++componentCount;
    }
    std::vector<int> componentKnown(componentCount, 0);
    for (std::size_t i = 0; i < verts.size(); ++i)
        if (known[i])
            ++componentKnown[component[i]];

    int unobservedCount = 0;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (!known[i] && componentKnown[component[i]] == 0) {
            verts[i].source = "unobserved";
            verts[i].unsupported = true;
            ++unobservedCount;
        }
    }

    std::vector<double> value(verts.size(), 0.0);
    for (std::size_t i = 0; i < verts.size(); ++i)
        value[i] = verts[i].elevation;

    const int maxIterations = 50000;
    const double residualTarget = std::max(tol.zTolUnits * 0.005, 1e-13);
    int iterations = 0;
    for (; iterations < maxIterations; ++iterations) {
        double maxDelta = 0.0;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            if (known[i] || componentKnown[component[i]] == 0)
                continue;
            double sum = 0.0;
            int count = 0;
            for (int nb : adj[i]) {
                sum += value[nb];
                ++count;
            }
            if (count == 0)
                continue;
            const double target = sum / count;
            const double next = value[i] + 1.7 * (target - value[i]);
            const double delta = std::abs(next - value[i]);
            if (delta > maxDelta)
                maxDelta = delta;
            value[i] = next;
        }
        if (iterations > 20 && maxDelta <= residualTarget)
            break;
    }

    int faceFilled = 0;
    int boundaryFilled = 0;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (known[i] || componentKnown[component[i]] == 0)
            continue;
        verts[i].elevation = value[i];
        if (slots[i].onBoundary || slots[i].onHole) {
            verts[i].source = "boundary_extrapolated";
            ++boundaryFilled;
        } else {
            verts[i].source = "face";
            ++faceFilled;
        }
    }

    if (unobservedCount > 0) {
        std::vector<int> knownList;
        for (std::size_t i = 0; i < verts.size(); ++i)
            if (known[i])
                knownList.push_back(static_cast<int>(i));
        for (std::size_t i = 0; i < verts.size(); ++i) {
            if (!verts[i].unsupported)
                continue;
            double best = std::numeric_limits<double>::infinity();
            double z = 0.0;
            for (int k : knownList) {
                const double d = distance(verts[i].position, verts[k].position);
                if (d < best) {
                    best = d;
                    z = verts[k].elevation;
                }
            }
            verts[i].elevation = z;
        }
    }

    // ------------------------------------------------------ coverage / support
    struct ObsSeg {
        Vec2 a;
        Vec2 b;
        int contourId;
    };
    std::vector<ObsSeg> obsSegs;
    for (const ContourInput& c : input.contours) {
        if (c.points.size() < 2)
            continue;
        const std::size_t count = c.closed ? c.points.size() : c.points.size() - 1;
        for (std::size_t i = 0; i < count; ++i)
            obsSegs.push_back({tol.toLocal(c.points[i]), tol.toLocal(c.points[(i + 1) % c.points.size()]), c.id});
    }
    std::vector<Vec2> obsControls;
    for (const ControlPoint& cp : input.controls)
        obsControls.push_back(tol.toLocal(cp.position));

    auto nearestObservation = [&](const Vec2& localP, int* whichContour) {
        double best = std::numeric_limits<double>::infinity();
        int which = -1;
        for (const ObsSeg& s : obsSegs) {
            const double d = pointSegmentDistance(localP, s.a, s.b);
            if (d < best) {
                best = d;
                which = s.contourId;
            }
        }
        for (const Vec2& q : obsControls) {
            const double d = distance(localP, q);
            if (d < best) {
                best = d;
                which = -1;
            }
        }
        if (whichContour)
            *whichContour = which;
        return best;
    };

    // Support radius follows contour spacing: for each observed contour vertex,
    // measure the distance to the nearest *different* contour line. The median
    // of those distances approximates the contour interval. Half of it is the
    // farthest a point can be from all observations while still being bracketed
    // by two contour lines. This scales with the survey, not with a constant.
    std::vector<double> contourSpacings;
    for (std::size_t li = 0; li < lines.size(); ++li) {
        if (lines[li].kind != SourceLine::Contour)
            continue;
        const SourceLine& L = lines[li];
        for (const Vec2& p : L.points) {
            const Vec2 lp = tol.toLocal(p);
            double best = std::numeric_limits<double>::infinity();
            for (const ContourInput& c : input.contours) {
                if (c.id == L.owner || c.points.size() < 2)
                    continue;
                const std::size_t count = c.closed ? c.points.size() : c.points.size() - 1;
                for (std::size_t i = 0; i < count; ++i) {
                    const double d = pointSegmentDistance(lp, tol.toLocal(c.points[i]),
                                                          tol.toLocal(c.points[(i + 1) % c.points.size()]));
                    if (d < best)
                        best = d;
                }
            }
            if (std::isfinite(best))
                contourSpacings.push_back(best);
        }
    }
    double supportLocal = 0.0;
    if (!contourSpacings.empty()) {
        std::sort(contourSpacings.begin(), contourSpacings.end());
        supportLocal = contourSpacings[contourSpacings.size() / 2] * 0.6;
    }
    if (!(supportLocal > 0.0)) {
        // Fewer than two contour lines: there is no interval to bracket an
        // unknown elevation, so only the contour line itself carries data.
        supportLocal = 0.0;
    }
    // Never below the snap tolerance (numerically meaningless below that),
    // never above half the domain extent.
    if (supportLocal > 0.0)
        supportLocal = std::min(std::max(supportLocal, snapLocal * 2.0), 0.5);
    const double supportMeters = supportLocal * scale * metersPerUnit;

    // A closed contour with no other contour inside it encloses an extremum
    // (summit or depression). Every point strictly inside is extrapolated,
    // however close it is to the ring, because no observation lies beyond it.
    std::vector<Ring> extremumRings;
    for (const ContourInput& c : input.contours) {
        if (!c.closed || c.points.size() < 3)
            continue;
        bool hasInner = false;
        for (const ContourInput& d : input.contours) {
            if (d.id == c.id || d.points.empty())
                continue;
            for (const Vec2& p : d.points) {
                bool onRing = false;
                if (pointInRing(p, c.points, snap, &onRing) && !onRing) {
                    hasInner = true;
                    break;
                }
            }
            if (hasInner)
                break;
        }
        if (!hasInner)
            extremumRings.push_back(c.points);
    }
    auto insideExtremumRing = [&](const Vec2& p) {
        for (const Ring& r : extremumRings) {
            bool onRing = false;
            if (pointInRing(p, r, snap, &onRing) && !onRing)
                return true;
        }
        return false;
    };

    double maxDistLocal = 0.0;
    for (std::size_t i = 0; i < verts.size(); ++i) {
        if (verts[i].source == "observed") {
            verts[i].distanceToObservation = 0.0;
            continue;
        }
        const double d = nearestObservation(tol.toLocal(verts[i].position), nullptr);
        verts[i].distanceToObservation = std::isfinite(d) ? d * scale * metersPerUnit : 0.0;
        if (std::isfinite(d))
            maxDistLocal = std::max(maxDistLocal, d);
        if (verts[i].source == "unobserved") {
            verts[i].unsupported = true;
            continue;
        }
        // Supported only when bracketed by observations: close enough to a
        // contour, and not enclosed by an extremum ring that has no inner line.
        verts[i].unsupported = !(supportLocal > 0.0 && std::isfinite(d) && d <= supportLocal) ||
                               insideExtremumRing(verts[i].position);
    }

    // ------------------------------------------------------- area accounting
    double totalAreaUnits2 = 0.0;
    double unsupportedAreaUnits2 = 0.0;
    int unsupportedTriangles = 0;
    int unsupportedVertices = 0;
    for (const MeshVertex& v : verts)
        if (v.unsupported)
            ++unsupportedVertices;
    for (MeshTriangle& t : tris) {
        const double area = std::abs(0.5 * cross(verts[t.v[1]].position - verts[t.v[0]].position,
                                                 verts[t.v[2]].position - verts[t.v[0]].position));
        totalAreaUnits2 += area;
        int n = 0;
        for (int k = 0; k < 3; ++k)
            if (verts[t.v[k]].unsupported)
                ++n;
        if (n > 0) {
            // A triangle is counted as unsupported area when any of its
            // vertices is outside observation support. Using any-vertex is
            // conservative: it never hides a void behind one supported corner.
            t.support = 1;
            ++unsupportedTriangles;
            unsupportedAreaUnits2 += area;
        }
    }
    const double areaFactor = metersPerUnit * metersPerUnit;

    // ------------------------------------------------------------------ stats
    int observedCount = 0;
    int edgeCount = 0;
    int boundaryCount = 0;
    double zMin = std::numeric_limits<double>::infinity();
    double zMax = -std::numeric_limits<double>::infinity();
    for (const MeshVertex& v : verts) {
        if (v.source == "observed")
            ++observedCount;
        else if (v.source == "constraint_edge")
            ++edgeCount;
        else if (v.source == "boundary_extrapolated")
            ++boundaryCount;
        zMin = std::min(zMin, v.elevation);
        zMax = std::max(zMax, v.elevation);
    }

    InterpolationSummary& is = result.interpolation;
    is.observedVertices = observedCount;
    is.edgeInterpolatedVertices = edgeCount;
    is.faceInterpolatedVertices = faceFilled;
    is.boundaryInterpolatedVertices = boundaryFilled;
    is.unobservedVertices = unobservedCount;
    is.boundaryVertices = 0;
    for (const PointSlot& s : slots)
        if (s.onBoundary || s.onHole)
            ++is.boundaryVertices;
    is.minElevation = std::isfinite(zMin) ? zMin : 0.0;
    is.maxElevation = std::isfinite(zMax) ? zMax : 0.0;
    is.constrainedEdges = static_cast<int>(constraints.size());
    is.unsupportedTriangles = unsupportedTriangles;
    is.unsupportedVertices = unsupportedVertices;
    is.unsupportedArea = unsupportedAreaUnits2 * areaFactor;
    is.totalArea = totalAreaUnits2 * areaFactor;
    is.supportRadius = supportMeters;
    is.maxDistanceToObservation = maxDistLocal * scale * metersPerUnit;

    {
        std::ostringstream m;
        m << "Supplied elevations (contour vertices and control points) are held fixed. A vertex created "
             "where two constraints intersect keeps the elevation from the contour it lies on and is "
             "filled by linear interpolation in arc length along that contour between the nearest "
             "supplied vertices. Every remaining unknown vertex is the solution of a discrete harmonic "
             "(Laplace) problem over the triangulated surface: its value is the average of its TIN "
             "neighbours, with all supplied elevations fixed. The system is solved by successive "
             "over-relaxation to a residual of "
          << std::scientific << std::setprecision(2) << residualTarget << " elevation units.";
        is.method = m.str();
    }
    {
        std::ostringstream c;
        c << std::fixed << std::setprecision(2);
        if (unsupportedTriangles > 0) {
            const double pct = is.totalArea > 0.0 ? 100.0 * is.unsupportedArea / is.totalArea : 0.0;
            c << unsupportedTriangles << " of " << tris.size() << " triangles (" << std::setprecision(1) << pct
              << "% of " << std::setprecision(1) << is.totalArea << " m^2) have at least one vertex that "
                 "is not bracketed by observations (farther than "
              << std::setprecision(2) << supportMeters
              << " m from any contour, or inside an un-nested closed contour that encloses an extremum). "
                 "Their displayed elevations are extrapolated/interpolated values, not surveyed ones. The "
                 "farthest vertex is "
              << std::setprecision(2) << is.maxDistanceToObservation << " m from any observation.";
        } else {
            c << "Every vertex is bracketed by supplied observations (within " << supportMeters
              << " m of a contour or control point, and not inside an un-nested closed contour). The "
                 "farthest vertex is "
              << std::setprecision(2) << is.maxDistanceToObservation << " m from an observation.";
        }
        is.coverageNote = c.str();
    }

    result.vertices = std::move(verts);
    result.triangles = std::move(tris);
    for (const Constraint& c : constraints)
        result.edges.push_back({c.a, c.b, c.kind});
    result.holes = input.holes;
    for (const FaultInput& f : input.faults)
        result.faults.push_back(f.points);
    result.elevationMs = msSince(tEle0);

    if (unobservedCount > 0)
        result.notes.push_back({"mesh.unobserved_region", "warning",
                                std::to_string(unobservedCount) +
                                    " vertex/vertices belong to components with no observation; displayed values are fallbacks",
                                "contour", -1, -1, {}, false, {}});
    if (unsupportedTriangles > 0)
        result.notes.push_back({"mesh.unsupported_area", "warning", is.coverageNote, "boundary", -1, -1, {}, false, {}});

    result.ok = true;
    result.totalMs = msSince(tStart);
    return result;
}

} // namespace terrain
