#pragma once

// ----------------------------------------------------------------------------
//
//  polytri : polygon triangulation (with holes) based on Seidel's algorithm.
//
//  Pipeline
//    1. Trapezoidal decomposition by randomized incremental construction, with
//       a point location DAG (nodes are converted in place, so any node can be
//       used as the root of a search). Segments are inserted in log* n levels :
//       between two levels the position of every endpoint not inserted yet is
//       refined from its previous position, so that later searches start
//       close to their target (Seidel 1991, O(n log* n) expected time).
//    2. Monotone partitioning : every interior trapezoid whose top or bottom
//       vertex lies strictly inside its horizontal side yields one diagonal.
//       The resulting planar graph is traversed face by face.
//    3. Each face is y-monotone and is triangulated in linear time.
//
//  Input requirements
//    * The first contour is the outer boundary, the following ones are holes.
//      Contours can be given in any orientation, they are normalized
//      internally (output triangles are always counter clockwise).
//    * Contours must be simple, must not touch or cross each other, and
//      vertices must be distinct points.
//    * All geometric predicates are exact (adaptive floating point
//      expansions), so degenerate inputs (equal x / y, collinear vertices)
//      are handled consistently.
//
// ----------------------------------------------------------------------------

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(__FAST_MATH__)
#error "polytri relies on exact floating point predicates : do not compile with -ffast-math."
#endif

/**
 * Compile time options (define before including this file).
 *
 * POLYTRI_ENABLE_LOGSTAR : 1 (default) builds the trapezoidation in levels as in
 *   Seidel's paper : O(n log* n) expected time. 0 builds it in a single level
 *   (O(n log n) expected), mostly useful to compare both.
 *
 * POLYTRI_LOGSTAR_MAX_LEVELS : 0 (default) uses the log* n levels of the paper,
 *   k > 0 keeps only the first k levels (the last one holding the rest).
 *
 * POLYTRI_RANDOM_SEED : seed of the (deterministic) permutation.
 *
 * POLYTRI_STATS : 1 counts the nodes visited by the searches (see `stats_`).
 *
 * POLYTRI_DEBUG_INFO  : 1 enables POLYTRI_LOG traces and internal checks.
 */
#ifndef POLYTRI_ENABLE_LOGSTAR
#define POLYTRI_ENABLE_LOGSTAR 1
#endif

#ifndef POLYTRI_LOGSTAR_MAX_LEVELS
#define POLYTRI_LOGSTAR_MAX_LEVELS 0
#endif

#ifndef POLYTRI_RANDOM_SEED
#define POLYTRI_RANDOM_SEED 0x9E3779B97F4A7C15ull
#endif

#ifndef POLYTRI_STATS
#define POLYTRI_STATS 0
#endif

#ifndef POLYTRI_DEBUG_INFO
#define POLYTRI_DEBUG_INFO 0
#endif

#if POLYTRI_DEBUG_INFO
#include <cstdio>
#define POLYTRI_LOG(...)  fprintf(stderr, __VA_ARGS__)
#else
#define POLYTRI_LOG(...)
#endif

namespace polytri {

/* -------------------------------------------------------------------------- */
/*  Types                                                                     */
/* -------------------------------------------------------------------------- */

struct Vertex {
  Vertex() = default;
  Vertex(double _x, double _y) : x(_x), y(_y) {}
  double x{};
  double y{};
};

struct Triangle {
  Triangle() = default;
  Triangle(uint32_t _v0, uint32_t _v1, uint32_t _v2) : v0(_v0), v1(_v1), v2(_v2) {}
  uint32_t v0{};
  uint32_t v1{};
  uint32_t v2{};
};

using TriangleBuffer = std::vector<Triangle>;

enum class Status : uint8_t {
  Ok = 0,
  EmptyInput,         // no contour, or a null pointer.
  ContourTooSmall,    // a contour has less than 3 vertices.
  ClosedContour,      // a contour repeats its first point at its end.
  DegenerateContour,  // a contour has all its vertices aligned (zero area).
  TooManyVertices,    // more vertices than the library or the index type supports.
  NotSimple,          // contours touch / cross each other or themselves.
  InternalError       // inconsistent result (also caused by non simple input).
};

struct Result {
  static constexpr uint32_t kNoContour = UINT32_MAX;

  Status status = Status::Ok;
  uint32_t contour = kNoContour;

  explicit operator bool() const noexcept { return Status::Ok == status; }

  const char *message() const noexcept {
    switch (status) {
      case Status::Ok:                return "ok";
      case Status::EmptyInput:        return "empty input";
      case Status::ContourTooSmall:   return "contour with less than 3 vertices";
      case Status::ClosedContour:     return "contour repeats its first point at the end";
      case Status::DegenerateContour: return "degenerate (flat) contour";
      case Status::TooManyVertices:   return "too many vertices";
      case Status::NotSimple:         return "input is not a simple polygon";
      case Status::InternalError:     return "inconsistent triangulation";
    }
    return "unknown";
  }
};

/* How to read the coordinates of a point type.
 *
 * The default works for types with public `x` / `y` members and for anything
 * indexable with [0] / [1] (std::array, std::vector, C arrays, ...etc).
 *
 * Specialize it for anything else:
 *   template <> struct polytri::PointTraits<MyPoint> {
 *     static double x(const MyPoint &p) { return p.get_x(); }
 *     static double y(const MyPoint &p) { return p.get_y(); }
 *   };
 */
template <typename P, typename = void>
struct PointTraits {
  static double x(const P &p) { return static_cast<double>(p[0]); }
  static double y(const P &p) { return static_cast<double>(p[1]); }
};

template <typename P>
struct PointTraits<P, std::void_t<decltype(std::declval<const P &>().x),
                                  decltype(std::declval<const P &>().y)>> {
  static double x(const P &p) { return static_cast<double>(p.x); }
  static double y(const P &p) { return static_cast<double>(p.y); }
};

namespace detail {

/* -------------------------------------------------------------------------- */
/*  Geometric predicates                                                      */
/* -------------------------------------------------------------------------- */

inline void two_sum(double a, double b, double &x, double &y) {
  x = a + b;
  const double bv = x - a;
  const double av = x - bv;
  y = (a - av) + (b - bv);
}

inline void two_diff(double a, double b, double &x, double &y) {
  x = a - b;
  const double bv = a - x;
  const double av = x + bv;
  y = (a - av) + (bv - b);
}

inline void two_prod(double a, double b, double &x, double &y) {
  x = a * b;
  y = std::fma(a, b, -x);
}

inline int orient2d(double ax, double ay, double bx, double by, double cx, double cy)
{
  const double detleft  = (bx - ax) * (cy - ay);
  const double detright = (by - ay) * (cx - ax);
  const double det = detleft - detright;

  double detsum;
  if (detleft > 0.0) {
    if (detright <= 0.0) return (det > 0.0) - (det < 0.0);
    detsum = detleft + detright;
  } else if (detleft < 0.0) {
    if (detright >= 0.0) return (det > 0.0) - (det < 0.0);
    detsum = -detleft - detright;
  } else {
    return (det > 0.0) - (det < 0.0);
  }

  constexpr double eps = 1.1102230246251565e-16;  // 2^-53
  const double errbound = (3.0 + 16.0 * eps) * eps * detsum;
  if ((det >= errbound) || (-det >= errbound)) {
    return (det > 0.0) - (det < 0.0);
  }

  double abx, abx_t, aby, aby_t, acx, acx_t, acy, acy_t;
  two_diff(bx, ax, abx, abx_t);
  two_diff(by, ay, aby, aby_t);
  two_diff(cx, ax, acx, acx_t);
  two_diff(cy, ay, acy, acy_t);

  double e[16];
  int n = 0;
  auto add = [&](double b) {
    double q = b;
    for (int i = 0; i < n; ++i) {
      double s, h;
      two_sum(q, e[i], s, h);
      q = s;
      e[i] = h;
    }
    e[n++] = q;
  };

  const double l[2] = {abx, abx_t}, r[2] = {acy, acy_t};
  const double m[2] = {aby, aby_t}, k[2] = {acx, acx_t};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      double p, pt;
      two_prod(l[i], r[j], p, pt);
      add(p);
      add(pt);
      two_prod(m[i], k[j], p, pt);
      add(-p);
      add(-pt);
    }
  }

  for (int i = n - 1; i >= 0; --i) {
    if (e[i] > 0.0) return 1;
    if (e[i] < 0.0) return -1;
  }
  return 0;
}

/* -------------------------------------------------------------------------- */
/*  RNG                                                                       */
/* -------------------------------------------------------------------------- */

struct SplitMix64 {
  uint64_t state;
  uint64_t next() {
    uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
};

}  // namespace detail

/* -------------------------------------------------------------------------- */
/*  Context                                                                   */
/* -------------------------------------------------------------------------- */

/* Keeps the memory between two calls, which avoids the allocations when many
 * polygons are triangulated one after the other.
 * Not thread safe : use one context per thread. */
class Context {
 public:
  /* Triangulate a polygon with holes.
   *
   * @param num_contours          number of contours (the first one is the outer
   *                              boundary, the following ones are holes).
   * @param nvertices_per_contour number of vertices of each contour (>= 3,
   *                              the first point is NOT repeated at the end).
   * @param vertices              all the vertices, contour after contour.
   * @param triangles             output (cleared first), counter clockwise,
   *                              indices into `vertices`.
   *
   * On failure `triangles` is empty and the result tells what went wrong. */
  Result triangulate(
    const size_t num_contours,
    const uint32_t nvertices_per_contour[],
    const Vertex *vertices,
    TriangleBuffer &triangles
  ) {
    triangles.clear();
    if (init(num_contours, nvertices_per_contour, vertices) && run(triangles)) {
      return Result{};
    }
    triangles.clear();
    return Result{status_, error_contour_};
  }

  /* Contours of any point type (see PointTraits) : any container with size()
   * and operator[], nested twice (eg. std::vector<std::vector<P>>).
   * `indices` receives 3 values per triangle. Index can be any unsigned integer type. */
  template <typename Index = uint32_t, typename Contours>
  Result triangulate(const Contours &contours, std::vector<Index> &indices) {
    static_assert(std::is_unsigned<Index>::value, "Index must be an unsigned integer type");
    indices.clear();

    input_counts_.clear();
    input_vertices_.clear();

    for (size_t j = 0u; j < contours.size(); ++j) {
      const auto &contour = contours[j];
      const size_t len = contour.size();
      using Traits = PointTraits<std::decay_t<decltype(contour[0])>>;

      if (len < 3u) {
        return Result{Status::ContourTooSmall, static_cast<uint32_t>(j)};
      }
      if (input_vertices_.size() + len > static_cast<uint64_t>(std::numeric_limits<uint32_t>::max())) {
        return Result{Status::TooManyVertices};
      }

      const auto &a = contour[0];
      const auto &b = contour[len - 1u];
      if ((Traits::x(a) == Traits::x(b)) && (Traits::y(a) == Traits::y(b))) {
        POLYTRI_LOG("Issue : start and end vertices are similar. (%.2f %.2f)\n",
          Traits::x(a), Traits::y(a));
        return Result{Status::ClosedContour, static_cast<uint32_t>(j)};
      }

      for (size_t i = 0u; i < len; ++i) {
        const auto &v = contour[i];
        input_vertices_.emplace_back(Traits::x(v), Traits::y(v));
      }
      input_counts_.push_back(static_cast<uint32_t>(len));
    }

    if ((input_vertices_.size() > 0u)
     && ((input_vertices_.size() - 1u) > static_cast<uint64_t>(std::numeric_limits<Index>::max()))) {
      return Result{Status::TooManyVertices};
    }

    const auto result = triangulate(input_counts_.size(), input_counts_.data(),
                                    input_vertices_.data(), output_triangles_);
    if (!result) {
      return result;
    }

    indices.reserve(3u * output_triangles_.size());
    for (const auto &t : output_triangles_) {
      indices.push_back(static_cast<Index>(t.v0));
      indices.push_back(static_cast<Index>(t.v1));
      indices.push_back(static_cast<Index>(t.v2));
    }
    return result;
  }

 private:
  static constexpr uint32_t kInvalidIndex = UINT32_MAX;

  // -------------------------
  // Types
  // -------------------------

  struct Segment {
    uint32_t v0{};
    uint32_t v1{};
  };

  // Query structure (point location DAG).
  enum class NodeType : uint8_t {
    XNode,  // key = segment, left = left of the segment, right = right of it.
    YNode,  // key = vertex,  left = below the vertex,   right = above it.
    Sink    // key = trapezoid index.
  };

  struct QNode {
    NodeType type = NodeType::Sink;
    uint32_t key_index = kInvalidIndex;
    uint32_t left = kInvalidIndex;
    uint32_t right = kInvalidIndex;
  };

  struct Trapezoid {
    uint32_t max_y = kInvalidIndex;
    uint32_t min_y = kInvalidIndex;
    uint32_t left_segment = kInvalidIndex;
    uint32_t right_segment = kInvalidIndex;
    uint32_t sink = kInvalidIndex;
  };

  // -------------------------
  // Entry points
  // -------------------------

  // (Re)initialize with a new input, keeping the memory of the previous runs.
  bool init(
    const size_t num_contours,
    const uint32_t nvertices_per_contour[],
    const Vertex *vertices
  );

  // Appends the triangles. Returns false if the input is not a valid polygon.
  bool run(TriangleBuffer &triangles);

  [[nodiscard]] bool ok() const { return Status::Ok == status_; }

  void fail(Status status, uint32_t contour = Result::kNoContour) {
    if (ok()) {
      status_ = status;
      error_contour_ = contour;
    }
  }

  // -------------------------
  // Geometry
  // -------------------------

  // Sign of the orientation of (a, b, c) : +1 ccw, -1 cw, 0 collinear.
  [[nodiscard]]
  static int orientation(const Vertex &a, const Vertex &b, const Vertex &c);

  // Lexicographic (y, x) order, ties (identical points) are broken by index.
  [[nodiscard]]
  bool is_lower(uint32_t a, uint32_t b) const;

  // > 0 if the vertex v is on the left of the (upward) segment, < 0 on the right.
  [[nodiscard]]
  int side_of_segment(uint32_t segment_index, uint32_t v) const;

  [[nodiscard]]
  bool segment_has_vertex(uint32_t segment_index, uint32_t v) const {
    return (seg_max_[segment_index] == v) || (seg_min_[segment_index] == v);
  }

  // -------------------------
  // Trapezoidal Decomposition
  // -------------------------

  void trapezoidal_decomposition();

  // Order in which segments are inserted.
  void init_permutation_table();

  // Indices (in the permutation) ending each level of insertion.
  [[nodiscard]]
  std::vector<uint32_t> compute_level_bounds() const;

  uint32_t new_sink(uint32_t trapezoid_index);

  // Search the query structure from `node` and return the leaf node reached.
  //  - `v` is the point searched.
  //  - when the search crosses a segment having `v` as endpoint, `ref_vertex`
  //    (if valid) tells on which side to go, by the side of this vertex, else
  //    `forced_left` is used.
  //  - when `v` is exactly a y-node, goes below.
  [[nodiscard]]
  uint32_t locate_sink(
    uint32_t node, uint32_t v, uint32_t ref_vertex, bool forced_left,
    bool tie_above = false
  ) const;

  // Same as locate_sink, returns the trapezoid index.
  [[nodiscard]]
  uint32_t locate(
    uint32_t node, uint32_t v, uint32_t ref_vertex, bool forced_left,
    bool tie_above = false
  ) const;

  // Find the trapezoid of every vertex that is not inserted yet, by following
  // the contours from the vertices already inserted : each edge is traced
  // through the few trapezoids it crosses, instead of searching from the top.
  void update_vertex_roots();

  // Return the trapezoid containing the vertex `to`, which is not inserted yet,
  // by following the straight line from the vertex `from` through the
  // trapezoids. `trapezoid_index` is the trapezoid of `from` (the one next to
  // the line when `from` is an inserted vertex). Returns kInvalidIndex if the
  // line goes through a vertex (not a simple polygon).
  [[nodiscard]]
  uint32_t trace_trapezoid(uint32_t trapezoid_index, uint32_t from, uint32_t to) const;

  // Split horizontally the trapezoid containing the vertex.
  void add_endpoint_to_query_structure(uint32_t vertex_index);

  // Split / merge all the trapezoids crossed by a segment.
  void add_segment_to_query_structure(uint32_t segment_index);

  // -------------------------
  // Monotone Partitioning
  // -------------------------

  [[nodiscard]]
  bool is_inside(const Trapezoid &trapezoid) const;

  void monotone_partitioning();

  void build_vertex_adjacency();

  // -------------------------
  // Monotone Triangulation
  // -------------------------

  void triangulate_monotone_polygons(TriangleBuffer &triangles);

  // Triangulate a y-monotone, counter clockwise polygon.
  void triangulate_monotone_polygon(const std::vector<uint32_t> &polygon, TriangleBuffer &triangles);

  // -------------------------
  // Attributes
  // -------------------------

  std::vector<Vertex> input_vertices_{};
  std::vector<uint32_t> input_counts_{};
  TriangleBuffer output_triangles_{};

  const Vertex *vertices_{};
  Status status_{Status::Ok};

  uint32_t num_vertices_{};
  uint32_t num_segments_{};
  uint32_t num_holes_{};

  std::vector<uint32_t> contour_offsets_{};  // first vertex of each contour, + end
  uint32_t error_contour_{Result::kNoContour};

  std::vector<Segment> segments_{};
  std::vector<uint32_t> seg_max_{};
  std::vector<uint32_t> seg_min_{};

  // Randomized segments indices
  std::vector<uint32_t> permutation_{};

  // Query structure for trapezoidation
  std::vector<Trapezoid> trapezoids_{};
  std::vector<QNode> query_points_{};
  std::vector<uint32_t> vertex_ynodes_{};
  std::vector<uint32_t> vertex_roots_{};
  std::vector<uint32_t> crossed_{};

  // Data for monotonization
  std::vector<Segment> diagonals_{};
  std::vector<uint32_t> adj_start_{};
  std::vector<uint32_t> adj_{};
  std::vector<uint8_t> adj_used_{};

#if POLYTRI_STATS
  struct Stats {
    uint64_t visits_endpoint = 0;
    uint64_t visits_start = 0;
    uint64_t visits_walk = 0;
    uint64_t visits_roots = 0;
    uint64_t crossings = 0;
    uint32_t levels = 0;
  };
  mutable Stats stats_{};
  mutable uint64_t visits_{};
#endif
};

/* -------------------------------------------------------------------------- */
/*  Entry points                                                              */
/* -------------------------------------------------------------------------- */

inline bool Context::init(
  const size_t num_contours,
  const uint32_t nvertices_per_contour[],
  const Vertex *vertices
) {
  status_ = Status::Ok;
  error_contour_ = Result::kNoContour;
  vertices_ = vertices;
  num_vertices_ = num_segments_ = num_holes_ = 0u;
  contour_offsets_.clear();
  trapezoids_.clear();
  query_points_.clear();
  diagonals_.clear();
  crossed_.clear();
#if POLYTRI_STATS
  stats_ = Stats{};
  visits_ = 0u;
#endif

  if ((0u == num_contours) || (nullptr == nvertices_per_contour) || (nullptr == vertices)) {
    fail(Status::EmptyInput);
    return false;
  }

  uint64_t total = 0u;
  for (size_t i = 0u; i < num_contours; ++i) {
    if (nvertices_per_contour[i] < 3u) {
      fail(Status::ContourTooSmall, static_cast<uint32_t>(i));
      return false;
    }
    total += nvertices_per_contour[i];
  }
  if (total >= (static_cast<uint64_t>(kInvalidIndex) / 8u)) {
    fail(Status::TooManyVertices);
    return false;
  }

  num_vertices_ = static_cast<uint32_t>(total);
  num_segments_ = num_vertices_;
  num_holes_ = static_cast<uint32_t>(num_contours - 1u);

  segments_.resize(num_segments_);
  seg_max_.resize(num_segments_);
  seg_min_.resize(num_segments_);

  contour_offsets_.reserve(num_contours + 1u);

  uint32_t offset = 0u;
  for (size_t cid = 0u; cid < num_contours; ++cid) {
    const auto n = nvertices_per_contour[cid];
    contour_offsets_.push_back(offset);

    uint32_t lowest = 0u;
    for (uint32_t i = 1u; i < n; ++i) {
      if (is_lower(offset + i, offset + lowest)) {
        lowest = i;
      }
    }
    const auto &prev = vertices_[offset + (lowest + n - 1u) % n];
    const auto &curr = vertices_[offset + lowest];
    const auto &next = vertices_[offset + (lowest + 1u) % n];
    const int orient = orientation(prev, curr, next);
    if (0 == orient) {
      fail(Status::DegenerateContour, static_cast<uint32_t>(cid));
      return false;
    }

    const bool want_ccw = (0u == cid);
    const bool flip = ((orient > 0) != want_ccw);

    for (uint32_t i = 0u; i < n; ++i) {
      const auto a = offset + i;
      const auto b = offset + (i + 1u) % n;

      const auto index = a;
      segments_[index].v0 = flip ? b : a;
      segments_[index].v1 = flip ? a : b;

      if (is_lower(a, b)) {
        seg_max_[index] = b;
        seg_min_[index] = a;
      } else {
        seg_max_[index] = a;
        seg_min_[index] = b;
      }
    }

    offset += n;
  }
  contour_offsets_.push_back(offset);
  return true;
}

/* -------------------------------------------------------------------------- */

inline bool Context::run(TriangleBuffer &triangles)
{
  if (!ok()) {
    return false;
  }

  trapezoidal_decomposition();
  if (!ok()) {
    return false;
  }

  monotone_partitioning();
  const size_t expected = static_cast<size_t>(num_vertices_) - 2u + 2u * num_holes_;
  triangles.reserve(expected);

  triangulate_monotone_polygons(triangles);
  if (!ok()) {
    return false;
  }

  if (triangles.size() != expected) {
    fail(Status::InternalError);
    POLYTRI_LOG("Error : %zu triangles instead of %zu.\n", triangles.size(), expected);
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
/*  Geometry                                                                  */
/* -------------------------------------------------------------------------- */

inline int Context::orientation(const Vertex &a, const Vertex &b, const Vertex &c)
{
  return detail::orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
}

/* -------------------------------------------------------------------------- */

inline bool Context::is_lower(uint32_t a, uint32_t b) const
{
  const auto &A = vertices_[a];
  const auto &B = vertices_[b];
  if (A.y != B.y) return A.y < B.y;
  if (A.x != B.x) return A.x < B.x;
  return a < b;
}

/* -------------------------------------------------------------------------- */

inline int Context::side_of_segment(uint32_t segment_index, uint32_t v) const
{
  const auto &lo = vertices_[seg_min_[segment_index]];
  const auto &hi = vertices_[seg_max_[segment_index]];
  return orientation(lo, hi, vertices_[v]);
}

/* -------------------------------------------------------------------------- */
/*  Trapezoidal Decomposition                                                 */
/* -------------------------------------------------------------------------- */

inline void Context::trapezoidal_decomposition()
{
  init_permutation_table();

  vertex_ynodes_.assign(num_vertices_, kInvalidIndex);
  vertex_roots_.assign(num_vertices_, 0u);
  trapezoids_.reserve(2u * num_segments_ + 2u);
  query_points_.reserve(8u * num_segments_ + 8u);

  trapezoids_.emplace_back();
  new_sink(0u);

  const auto bounds = compute_level_bounds();
#if POLYTRI_STATS
  stats_.levels = static_cast<uint32_t>(bounds.size());
#endif

  uint32_t next = 0u;
  for (size_t level = 0u; level < bounds.size(); ++level) {
    for (; next < bounds[level]; ++next) {
      add_segment_to_query_structure(permutation_[next]);
      if (!ok()) {
        return;
      }
    }

    if (level + 1u < bounds.size()) {
#if POLYTRI_STATS
      const auto visits_before = visits_;
#endif
      update_vertex_roots();
#if POLYTRI_STATS
      stats_.visits_roots += visits_ - visits_before;
#endif
    }
  }
}

/* -------------------------------------------------------------------------- */

inline void Context::init_permutation_table()
{
  permutation_.resize(num_segments_);
  for (uint32_t i = 0u; i < num_segments_; ++i) {
    permutation_[i] = i;
  }
  detail::SplitMix64 rng{POLYTRI_RANDOM_SEED};
  for (uint32_t i = num_segments_; i > 1u; --i) {
    const auto j = static_cast<uint32_t>(rng.next() % i);
    std::swap(permutation_[i - 1u], permutation_[j]);
  }
}

/* -------------------------------------------------------------------------- */

inline std::vector<uint32_t> Context::compute_level_bounds() const
{
  std::vector<uint32_t> bounds;

#if POLYTRI_ENABLE_LOGSTAR
  const double n = static_cast<double>(num_segments_);
  double l = n;
  uint32_t last = 0u;
  for (;;) {
    l = std::log2(l);
    if (l <= 1.0) {
      break;
    }
    const double bound = std::ceil(n / l);
    if (bound >= n) {
      break;
    }
    const auto b = static_cast<uint32_t>(bound);
    if (b > last) {
      bounds.push_back(b);
      last = b;
    }
  }
#endif

#if POLYTRI_ENABLE_LOGSTAR && (POLYTRI_LOGSTAR_MAX_LEVELS > 0)
  // Optional cap on the number of levels (last one included).
  if (bounds.size() + 1u > POLYTRI_LOGSTAR_MAX_LEVELS) {
    bounds.resize(POLYTRI_LOGSTAR_MAX_LEVELS - 1u);
  }
#endif

  bounds.push_back(num_segments_);
  return bounds;
}

/* -------------------------------------------------------------------------- */

inline uint32_t Context::new_sink(uint32_t trapezoid_index)
{
  QNode node;
  node.type = NodeType::Sink;
  node.key_index = trapezoid_index;

  const auto node_index = static_cast<uint32_t>(query_points_.size());
  query_points_.push_back(node);
  trapezoids_[trapezoid_index].sink = node_index;
  return node_index;
}

/* -------------------------------------------------------------------------- */

inline uint32_t Context::locate_sink(
  uint32_t node,
  uint32_t v,
  uint32_t ref_vertex,
  bool forced_left,
  bool tie_above
) const {
  while (NodeType::Sink != query_points_[node].type) {
#if POLYTRI_STATS
    ++visits_;
#endif
    const auto &n = query_points_[node];

    if (NodeType::YNode == n.type) {
      const bool below = (n.key_index == v) ? !tie_above : is_lower(v, n.key_index);
      node = below ? n.left : n.right;
    } else {
      bool left;
      if (segment_has_vertex(n.key_index, v)) {
        left = (kInvalidIndex != ref_vertex)
             ? (side_of_segment(n.key_index, ref_vertex) > 0)
             : forced_left
             ;
      } else {
        left = (side_of_segment(n.key_index, v) > 0);
      }
      node = left ? n.left : n.right;
    }
  }
  return node;
}

/* -------------------------------------------------------------------------- */

inline uint32_t Context::locate(
  uint32_t node,
  uint32_t v,
  uint32_t ref_vertex,
  bool forced_left,
  bool tie_above
) const {
  return query_points_[
    locate_sink(node, v, ref_vertex, forced_left, tie_above)
  ].key_index;
}

/* -------------------------------------------------------------------------- */

inline void Context::update_vertex_roots()
{
  for (size_t c = 0u; c + 1u < contour_offsets_.size(); ++c) {
    const auto first = contour_offsets_[c];
    const auto count = contour_offsets_[c + 1u] - first;

    uint32_t start = 0u;
    while ((start < count) && (kInvalidIndex == vertex_ynodes_[first + start])) {
      ++start;
    }

    uint32_t prev = first + start;
    uint32_t prev_trapezoid = kInvalidIndex;

    if (start >= count) {
      // No vertex inserted : search the first one from its previous root.
      start = 0u;
      prev = first;
      const auto sink = locate_sink(vertex_roots_[prev], prev, kInvalidIndex, false);
      vertex_roots_[prev] = sink;
      prev_trapezoid = query_points_[sink].key_index;
    }

    // Visit the other vertices of the contour once, in order.
    for (uint32_t k = 1u; k < count; ++k) {
      const auto w = first + (start + k) % count;
      if (kInvalidIndex != vertex_ynodes_[w]) {
        prev = w;
        prev_trapezoid = kInvalidIndex;
        continue;
      }

      uint32_t t;
      if (kInvalidIndex == prev_trapezoid) {
        const bool down = is_lower(w, prev);
        t = locate(vertex_ynodes_[prev], prev, w, false, !down);
      } else {
        t = prev_trapezoid;
      }

      t = trace_trapezoid(t, prev, w);
      if (kInvalidIndex == t) {
        fail(Status::NotSimple);
        return;
      }

      vertex_roots_[w] = trapezoids_[t].sink;
      prev = w;
      prev_trapezoid = t;
    }
  }
}

/* -------------------------------------------------------------------------- */

inline uint32_t Context::trace_trapezoid(
  uint32_t trapezoid_index,
  uint32_t from,
  uint32_t to
) const {
  const bool down = is_lower(to, from);
  const auto lower = down ? to : from;
  const auto upper = down ? from : to;

  auto t = trapezoid_index;
  for (size_t guard = 0u; guard <= trapezoids_.size(); ++guard) {
    const auto &trapezoid = trapezoids_[t];
    const auto next_vertex = down ? trapezoid.min_y : trapezoid.max_y;

    if (kInvalidIndex == next_vertex) {
      return t;
    }
    if (down ? !is_lower(to, next_vertex) : !is_lower(next_vertex, to)) {
      return t;
    }

    const int side = orientation(
      vertices_[lower], vertices_[upper], vertices_[next_vertex]
    );
    if (0 == side) {
      return kInvalidIndex;
    }
    t = locate(
      vertex_ynodes_[next_vertex], next_vertex, kInvalidIndex, (side < 0), !down
    );
  }
  return kInvalidIndex;
}

/* -------------------------------------------------------------------------- */

inline void Context::add_endpoint_to_query_structure(uint32_t vertex_index)
{
  if (kInvalidIndex != vertex_ynodes_[vertex_index]) {
    return;
  }

#if POLYTRI_STATS
  const auto visits_before = visits_;
#endif
  const auto top_index = locate(
    vertex_roots_[vertex_index], vertex_index, kInvalidIndex, false
  );
#if POLYTRI_STATS
  stats_.visits_endpoint += visits_ - visits_before;
#endif
  const auto btm_index = static_cast<uint32_t>(trapezoids_.size());

  // new trapezoid is the bottom part, existing one keeps the top part.
  const Trapezoid old = trapezoids_[top_index];
  trapezoids_.push_back(old);
  trapezoids_[btm_index].max_y = vertex_index;
  trapezoids_[top_index].min_y = vertex_index;

  // old sink becomes the y-node.
  const auto y_node = old.sink;
  const auto top_sink = new_sink(top_index);
  const auto btm_sink = new_sink(btm_index);

  auto &node = query_points_[y_node];
  node.type = NodeType::YNode;
  node.key_index = vertex_index;
  node.left = btm_sink;
  node.right = top_sink;

  vertex_ynodes_[vertex_index] = y_node;

  POLYTRI_LOG("split Y : top %u / bottom %u (at v[%u].y = %.3f).\n",
    top_index, btm_index, vertex_index, vertices_[vertex_index].y);
}

/* -------------------------------------------------------------------------- */

inline void Context::add_segment_to_query_structure(uint32_t segment_index)
{
  const auto top = seg_max_[segment_index];
  const auto bottom = seg_min_[segment_index];

  add_endpoint_to_query_structure(top);
  add_endpoint_to_query_structure(bottom);

  // Find all the trapezoids crossed by the segment.
  crossed_.clear();
#if POLYTRI_STATS
  auto visits_mark = visits_;
#endif
  auto trapezoid_index = locate(vertex_ynodes_[top], top, bottom, false);
#if POLYTRI_STATS
  stats_.visits_start += visits_ - visits_mark;
  visits_mark = visits_;
#endif

  for (;;) {
    crossed_.push_back(trapezoid_index);

    const auto next_vertex = trapezoids_[trapezoid_index].min_y;
    if (next_vertex == bottom) {
      break;
    }
    if (kInvalidIndex == next_vertex) {
      fail(Status::NotSimple);
      return;
    }

    const int side = side_of_segment(segment_index, next_vertex);
    if ((0 == side) || (crossed_.size() > trapezoids_.size())) {
      fail(Status::NotSimple);
      return;
    }

    trapezoid_index = locate(
      vertex_ynodes_[next_vertex], next_vertex, kInvalidIndex, (side < 0)
    );
#if POLYTRI_STATS
    stats_.visits_walk += visits_ - visits_mark;
    visits_mark = visits_;
#endif
  }
#if POLYTRI_STATS
  stats_.crossings += crossed_.size();
#endif

  // Split each crossed trapezoid in a left part and a right part.
  uint32_t left_trap = kInvalidIndex;
  uint32_t right_trap = kInvalidIndex;
  for (size_t i = 0u; i < crossed_.size(); ++i) {
    const auto t = crossed_[i];
    const Trapezoid old = trapezoids_[t];

    uint32_t left_sink, right_sink;

    if (0u == i) {
      const auto r = static_cast<uint32_t>(trapezoids_.size());

      Trapezoid right;
      right.max_y = old.max_y;
      right.min_y = old.min_y;
      right.left_segment = segment_index;
      right.right_segment = old.right_segment;
      trapezoids_.push_back(right);

      trapezoids_[t].right_segment = segment_index;

      left_sink = new_sink(t);
      right_sink = new_sink(r);
      left_trap = t;
      right_trap = r;
    } else if (side_of_segment(segment_index, old.max_y) < 0) {
      if (trapezoids_[left_trap].left_segment != old.left_segment) {
        fail(Status::NotSimple);
        return;
      }
      trapezoids_[left_trap].min_y = old.min_y;
      left_sink = trapezoids_[left_trap].sink;

      Trapezoid right;
      right.max_y = old.max_y;
      right.min_y = old.min_y;
      right.left_segment = segment_index;
      right.right_segment = old.right_segment;
      trapezoids_[t] = right;

      right_sink = new_sink(t);
      right_trap = t;
    } else {
      if (trapezoids_[right_trap].right_segment != old.right_segment) {
        fail(Status::NotSimple);
        return;
      }
      trapezoids_[right_trap].min_y = old.min_y;
      right_sink = trapezoids_[right_trap].sink;

      Trapezoid left;
      left.max_y = old.max_y;
      left.min_y = old.min_y;
      left.left_segment = old.left_segment;
      left.right_segment = segment_index;
      trapezoids_[t] = left;

      left_sink = new_sink(t);
      left_trap = t;
    }

    // The old sink becomes an x-node
    auto &node = query_points_[old.sink];
    node.type = NodeType::XNode;
    node.key_index = segment_index;
    node.left = left_sink;
    node.right = right_sink;
  }
}

/* -------------------------------------------------------------------------- */
/*  Monotone Partitioning                                                     */
/* -------------------------------------------------------------------------- */

inline bool Context::is_inside(const Trapezoid &trapezoid) const
{
  if ((kInvalidIndex == trapezoid.left_segment)
   || (kInvalidIndex == trapezoid.right_segment)) {
    return false;
  }

  const auto right = trapezoid.right_segment;
  const bool inside = (seg_max_[right] == segments_[right].v1);

#if POLYTRI_DEBUG_INFO
  const auto left = trapezoid.left_segment;
  assert(inside == (seg_max_[left] == segments_[left].v0));
#endif

  return inside;
}

/* -------------------------------------------------------------------------- */

inline void Context::monotone_partitioning()
{
  POLYTRI_LOG("\n((%s))\n", __FUNCTION__);

  diagonals_.clear();

  for (const auto &trapezoid : trapezoids_) {
    if (!is_inside(trapezoid)) {
      continue;
    }

    const auto top = trapezoid.max_y;
    const auto bottom = trapezoid.min_y;
    assert(kInvalidIndex != top);
    assert(kInvalidIndex != bottom);

    const auto l = trapezoid.left_segment;
    const auto r = trapezoid.right_segment;

    const bool top_middle = !segment_has_vertex(l, top) && !segment_has_vertex(r, top);
    const bool btm_middle = !segment_has_vertex(l, bottom) && !segment_has_vertex(r, bottom);

    if (top_middle || btm_middle) {
      Segment diagonal;
      diagonal.v0 = top;
      diagonal.v1 = bottom;
      diagonals_.push_back(diagonal);
    }
  }

  build_vertex_adjacency();
}

/* -------------------------------------------------------------------------- */

inline void Context::build_vertex_adjacency()
{
  // Compressed adjacency lists of the planar graph made of the polygon edges
  // and the diagonals. A vertex has at most a few diagonals, so sorting its
  // neighbors around it costs O(1).

  adj_start_.assign(num_vertices_ + 1u, 0u);

  for (const auto &s : segments_) {
    ++adj_start_[s.v0 + 1u];
    ++adj_start_[s.v1 + 1u];
  }
  for (const auto &d : diagonals_) {
    ++adj_start_[d.v0 + 1u];
    ++adj_start_[d.v1 + 1u];
  }
  for (uint32_t i = 0u; i < num_vertices_; ++i) {
    adj_start_[i + 1u] += adj_start_[i];
  }

  adj_.assign(adj_start_[num_vertices_], kInvalidIndex);
  adj_used_.assign(adj_.size(), 0u);

  std::vector<uint32_t> fill(adj_start_.begin(), adj_start_.end() - 1);
  auto link = [&](uint32_t a, uint32_t b) {
    adj_[fill[a]++] = b;
    adj_[fill[b]++] = a;
  };
  for (const auto &s : segments_) link(s.v0, s.v1);
  for (const auto &d : diagonals_) link(d.v0, d.v1);

  // Sort the neighbors of each vertex counter clockwise.
  for (uint32_t v = 0u; v < num_vertices_; ++v) {
    const auto &V = vertices_[v];
    auto first = adj_.begin() + adj_start_[v];
    auto last = adj_.begin() + adj_start_[v + 1u];

    auto upper_half = [&](uint32_t a) {
      const auto &A = vertices_[a];
      return (A.y > V.y) || ((A.y == V.y) && (A.x > V.x)) ? 0 : 1;
    };

    std::sort(first, last, [&](uint32_t a, uint32_t b) {
      const int ha = upper_half(a);
      const int hb = upper_half(b);
      if (ha != hb) return ha < hb;
      return orientation(V, vertices_[a], vertices_[b]) > 0;
    });
  }

  // Half edges going against the polygon orientation are on the outside.
  for (const auto &s : segments_) {
    for (auto i = adj_start_[s.v1]; i < adj_start_[s.v1 + 1u]; ++i) {
      if (adj_[i] == s.v0) {
        adj_used_[i] = 1u;
        break;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
/*  Monotone Triangulation                                                    */
/* -------------------------------------------------------------------------- */

inline void Context::triangulate_monotone_polygons(TriangleBuffer &triangles)
{
  POLYTRI_LOG("\n((%s))\n", __FUNCTION__);

  std::vector<uint32_t> face{};
  for (uint32_t u = 0u; u < num_vertices_; ++u) {
    for (uint32_t j = adj_start_[u]; j < adj_start_[u + 1u]; ++j) {
      if (adj_used_[j]) {
        continue;
      }

      face.clear();
      uint32_t cu = u;
      uint32_t cj = j;
      do {
        adj_used_[cj] = 1u;
        face.push_back(cu);

        const auto v = adj_[cj];
        const auto begin = adj_start_[v];
        const auto end = adj_start_[v + 1u];
        auto pos = begin;
        while ((pos < end) && (adj_[pos] != cu)) {
          ++pos;
        }
        assert(pos < end);
        cj = (pos == begin) ? (end - 1u) : (pos - 1u);
        cu = v;

        if (face.size() > adj_.size()) {
          fail(Status::NotSimple);
          return;
        }
      } while (!adj_used_[cj]);

      if ((cu != u) || (cj != j) || (face.size() < 3u)) {
        fail(Status::NotSimple);
        return;
      }

      triangulate_monotone_polygon(face, triangles);
      if (!ok()) {
        return;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */

inline void Context::triangulate_monotone_polygon(
  const std::vector<uint32_t> &polygon,
  TriangleBuffer &triangles
) {
  const auto m = static_cast<uint32_t>(polygon.size());

  if (3u == m) {
    triangles.emplace_back(polygon[0], polygon[1], polygon[2]);
    return;
  }

  // Topmost and bottommost vertices.
  uint32_t top = 0u, bottom = 0u;
  for (uint32_t i = 1u; i < m; ++i) {
    if (is_lower(polygon[top], polygon[i])) top = i;
    if (is_lower(polygon[i], polygon[bottom])) bottom = i;
  }

  // Merge the left (top -> bottom, counter clockwise) and right chains in
  // decreasing order. chain : 0 = left, 1 = right, 2 = extremities.
  struct Item { uint32_t vertex; uint8_t chain; };
  std::vector<Item> order;
  order.reserve(m);
  order.push_back({polygon[top], 2u});

  uint32_t li = (top + 1u) % m;
  uint32_t ri = (top + m - 1u) % m;
  while ((li != bottom) || (ri != bottom)) {
    bool take_left;
    if (li == bottom)      take_left = false;
    else if (ri == bottom) take_left = true;
    else                   take_left = !is_lower(polygon[li], polygon[ri]);

    if (take_left) {
      order.push_back({polygon[li], 0u});
      li = (li + 1u) % m;
    } else {
      order.push_back({polygon[ri], 1u});
      ri = (ri + m - 1u) % m;
    }
  }
  order.push_back({polygon[bottom], 2u});

  if (order.size() != m) {
    fail(Status::NotSimple);
    return;
  }

  auto emit_ccw = [&](uint32_t a, uint32_t b, uint32_t c) {
    const int o = orientation(vertices_[a], vertices_[b], vertices_[c]);
    if (o >= 0) triangles.emplace_back(a, b, c);
    else        triangles.emplace_back(a, c, b);
  };

  std::vector<Item> stack;
  stack.reserve(m);
  stack.push_back(order[0]);
  stack.push_back(order[1]);

  for (uint32_t j = 2u; j + 1u < m; ++j) {
    const Item u = order[j];

    if (u.chain != stack.back().chain) {
      // Opposite chain : u sees all the vertices of the stack.
      for (size_t i = stack.size() - 1u; i > 0u; --i) {
        if (0u == u.chain) {
          triangles.emplace_back(u.vertex, stack[i].vertex, stack[i - 1u].vertex);
        } else {
          triangles.emplace_back(u.vertex, stack[i - 1u].vertex, stack[i].vertex);
        }
      }
      const Item last = stack.back();
      stack.clear();
      stack.push_back(last);
      stack.push_back(u);
    } else {
      // Same chain : cut the convex corners.
      Item last = stack.back();
      stack.pop_back();

      while (!stack.empty()) {
        const auto &prev = stack.back();
        const int o = orientation(
          vertices_[prev.vertex], vertices_[last.vertex], vertices_[u.vertex]
        );
        if (0u == u.chain) {
          if (o <= 0) break;
          triangles.emplace_back(prev.vertex, last.vertex, u.vertex);
        } else {
          if (o >= 0) break;
          triangles.emplace_back(u.vertex, last.vertex, prev.vertex);
        }
        last = prev;
        stack.pop_back();
      }

      stack.push_back(last);
      stack.push_back(u);
    }
  }

  // The last vertex sees all the remaining ones.
  const auto last_vertex = order[m - 1u].vertex;
  for (size_t i = stack.size() - 1u; i > 0u; --i) {
    emit_ccw(last_vertex, stack[i].vertex, stack[i - 1u].vertex);
  }
}

/* -------------------------------------------------------------------------- */
/*  Single Use Functions                                                      */
/* -------------------------------------------------------------------------- */

inline Result triangulate(
  const size_t num_contours,
  const uint32_t nvertices_per_contour[],
  const Vertex *vertices,
  TriangleBuffer &triangles
) {
  Context context;
  return context.triangulate(num_contours, nvertices_per_contour, vertices, triangles);
}

template <typename Index = uint32_t, typename Contours>
Result triangulate(const Contours &contours, std::vector<Index> &indices) {
  Context context;
  return context.triangulate(contours, indices);
}

template <typename Index = uint32_t, typename Contours>
std::vector<Index> triangulate(const Contours &contours) {
  std::vector<Index> indices;
  if (!triangulate(contours, indices)) {
    indices.clear();
  }
  return indices;
}

}  // namespace polytri
