#pragma once

#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <chrono>
#include <iostream>
#include <limits>
#include <list>
#include <random>
#include <vector>
#include <algorithm>

// ----------------------------------------------------------------------------

#ifndef POLYTRI_ENABLE_PERMUTATION
#define POLYTRI_ENABLE_PERMUTATION 0
#endif

#define POLYTRI_DEBUG_INFO 0

#if POLYTRI_DEBUG_INFO
#define POLYTRI_LOG(...)  fprintf(stderr, __VA_ARGS__)
#else
#define POLYTRI_LOG(...)
#endif

// ----------------------------------------------------------------------------

class PolyTri {
 public:
  struct vertex_t {
    vertex_t() = default;
    vertex_t(double _x, double _y) : x(_x), y(_y) {}
    double x{};
    double y{};
  };

  struct segment_t {
    uint32_t v0{};
    uint32_t v1{};
  };

  struct triangle_t {
    triangle_t() = default;
    triangle_t(uint32_t _v0, uint32_t _v1, uint32_t _v2)
      : v0(_v0), v1(_v1), v2(_v2)
    {}
    uint32_t v0{};
    uint32_t v1{};
    uint32_t v2{};
  };

 public:
  using TriangleBuffer_t = std::vector<triangle_t>;

  /**
   * num_contours : number of contours and size of the nvertices_per_contour array.
   * nvertices_per_contour : array of num_contour, each cell contain the number of
   *         vertices in its contour.
   * vertices : a set of sequentially ordered 2d floating point coordinates to triangulate.
   * triangles : a buffer of indices where the triangles faces will be output.
   */
  static void Triangulate(
    const size_t num_contours,
    const uint32_t nvertices_per_contour[],
    const vertex_t *vertices,
    TriangleBuffer_t &triangles
  );

  /* mapbox/earcut type interface */
  template </*typename N = uint32_t,*/ typename P>
  static std::vector<uint32_t> Triangulate(P const& polys) {
    size_t const NPOLY = polys.size();

    std::vector<uint32_t> contour_lengths;
    uint32_t totalLength = 0;
    for (size_t j = 0; j < NPOLY; ++j) {
      auto const& p = polys[j];
      auto const clen = static_cast<uint32_t>(p.size());
      totalLength += clen;
      contour_lengths.push_back(clen);
    }

    std::vector<vertex_t> vertices{};
    vertices.reserve(totalLength);

    for (size_t j = 0; j < NPOLY; ++j) {
      auto const& p = polys[j];

      for (size_t i = 0; i < p.size(); ++i) {
        auto const& v = p[i];
        vertices.emplace_back(v.x, v.y);
      }

      auto const&a = p[0];
      auto const&b = p[p.size()-1];
      if ((fabs(a.x-b.x) < DBL_EPSILON) && (fabs(a.y-b.y) < DBL_EPSILON)) {
        POLYTRI_LOG("Issue : start and end meet. (%.2f %.2f)\n", a.x, a.y);
        vertices.resize(vertices.size()-1);
        contour_lengths[j] -= 1;
      }
    }

    TriangleBuffer_t triangles{};
    Triangulate(
      contour_lengths.size(),
      contour_lengths.data(),
      vertices.data(),
      triangles
    );

    std::vector<uint32_t> indices{};
    indices.reserve(3 * triangles.size());
    for (auto const& t : triangles) {
      indices.insert(indices.end(), {t.v0, t.v1, t.v2}); //
    }
    return indices;
  }

 private:
  static const uint32_t kInvalidIndex = UINT32_MAX;

  enum QNodeType_t {
    X_NODE,
    Y_NODE,
    SINK
  };

  enum MergeSide_t {
    MergeLeft,
    MergeRight,
    MergeEnd
  };

  struct QNode_t {
    QNodeType_t type = SINK;
    uint32_t key_index = kInvalidIndex;
    QNode_t *left = nullptr;
    QNode_t *right = nullptr;
    QNode_t *parent = nullptr;
  };

  struct Trapezoid_t {
    uint32_t max_y = kInvalidIndex;
    uint32_t min_y = kInvalidIndex;
    uint32_t left_segment = kInvalidIndex;
    uint32_t right_segment = kInvalidIndex;
    uint32_t above1 = kInvalidIndex;
    uint32_t above2 = kInvalidIndex;
    uint32_t below1 = kInvalidIndex;
    uint32_t below2 = kInvalidIndex;
    QNode_t *sink = nullptr;
  };

  using Chain_t = std::list<uint32_t>;
  using ChainIterator_t = Chain_t::iterator;

  enum InsertionSide_t {
    InsertLeft,
    InsertRight
  };

  struct Monochain_t {
    InsertionSide_t insertion_side;
    Chain_t list;
  };

  static
  InsertionSide_t GetIntersectionSide(
    const bool min_is_right,
    const bool go_down
  );

 private:
  PolyTri(
    const size_t num_contours,
    const uint32_t nvertices_per_contour[],
    const vertex_t *vertices
  );

  void trapezoidal_decomposition();

  void monotone_partitioning();

  void triangulate_monotone_polygons(TriangleBuffer_t &triangles);

  // -------------------------
  // Trapezoidal Decomposition
  // -------------------------

  // Return true if trapezoid is a top inside triangle.
  [[nodiscard]]
  bool is_top_inside_triangle(const Trapezoid_t &trapezoid) const;

  // Return true if trapezoid is a top triangle.
  [[nodiscard]]
  bool is_top_triangle(const Trapezoid_t &trapezoid) const;

  // Return true if trapezoid is a bottom triangle.
  [[nodiscard]]
  bool is_bottom_triangle(const Trapezoid_t &trapezoid) const;

  // Return a random segment index
  [[nodiscard]]
  uint32_t new_random_segment_index();

  // Compute the signed distance of point to a segment, to know its side.
  [[nodiscard]]
  double distance_from_segment(const vertex_t &v, const segment_t &segment);

  // Return the trapezoid index containing the vertex.
  [[nodiscard]]
  uint32_t search_trapezoid_index(const vertex_t &v, const QNode_t *node);

  [[nodiscard]]
  inline uint32_t search_trapezoid_index(const vertex_t &v) {
    return search_trapezoid_index(v, root_);
  }

  // Return the index of the next available trapezoid to be initialized.
  [[nodiscard]]
  uint32_t get_new_trapezoid_index();

  // Return a pointer to the next available node to be used, initialized with default values.
  QNode_t* create_node(QNodeType_t type, QNode_t *n, uint32_t key_index);

  // Link a sink node to its trapezoid and vice versa.
  void link_sink_node_and_trapezoid(QNode_t *node, uint32_t trapezoid_index);

  // Determine the max and min vertex indices of segment s.
  void get_max_min_y_indices(const segment_t& s,
                             uint32_t &max_y_index,
                             uint32_t &min_y_index) const;

  // Update trapezoid neighbors info after a y-split.
  void update_ysplit_trapezoid_neighbors(const uint32_t trapezoid_index);

  // Add an endpoint to the query structure, creating a y-node and a new trapezoid.
  void add_endpoint_to_query_structure(const uint32_t vertex_index);

  // Add a segment to the query structure.
  void add_segment_to_query_structure(const uint32_t segment_index);

  // Return the top_sink fusionned with btm_sink.
  [[nodiscard]]
  QNode_t* fusion_sinks(QNode_t *top_sink, QNode_t *btm_sink);

  // Update below's above neighbors.
  void update_trapezoid_aboves(
    const uint32_t trapezoid_index,
    Trapezoid_t &below
  );

  // Update a trapezoid neighbors during a X-split.
  void update_xsplit_trapezoid_neighbors(const MergeSide_t side,
                                         const uint32_t left_trap_index,
                                         const uint32_t right_trap_index);

  // Update a node, its parent and its previous parent.
  void update_node_parent(QNode_t *new_parent, QNode_t *node);

  // Recursively split / merge trapezoids until the last endpoint.
  void split_merge_trapezoids(const uint32_t segment_index,
                              const uint32_t end_y_index,
                              const uint32_t trapezoid_index,
                              QNode_t *left_fusion_node,
                              QNode_t *right_fusion_node);

  // Compute a slighty offset vertex to find the first trapezoid of the segment.
  void compute_offset_vertex(const uint32_t max_y_index,
                             const uint32_t min_y_index,
                             vertex_t &offset) const;

  // Connect two endpoints by split / merging intermittent trapezoids.
  void thread_endpoints(const uint32_t segment_index,
                        const uint32_t top_index,
                        const uint32_t bottom_index);

  // Create a randomized table of numsegment indices.
  void init_permutation_table();

  // Initialize the query structure by adding a first random segment.
  void init_query_structure();


  // -------------------------
  // Monotone Partitioning
  // -------------------------

  [[nodiscard]]
  uint32_t find_top_inside_trapezoid_index() const;

  void add_vertex_to_monochain(
    const Trapezoid_t &trapezoid,
    const bool go_down,
    Monochain_t *monochain
  );

  [[nodiscard]]
  Monochain_t* create_monochain(
    const uint32_t first_index,
    const uint32_t second_index,
    const InsertionSide_t side
  );

  void select_monotone_path(
    const uint32_t trapezoid_index,
    const bool go_down,
    const bool come_from_left
  );

  void build_monotone_chains(
    Monochain_t *monochain,
    const uint32_t trapezoid_index,
    const uint32_t from_index,
    const bool go_down
  );

  // -------------------------
  // Monochain Triangulation
  // -------------------------

  // Triangulate a direct order monochain.
  void triangulate_monochain(
    Monochain_t &monochain,
    const ChainIterator_t &first,
    TriangleBuffer_t &triangles
  );

  // Return true if an angle is convex.
  [[nodiscard]]
  bool is_angle_convex(uint32_t v0, uint32_t v1, uint32_t v2) const;

  // -------------------------
  // Attributes
  // -------------------------

  uint32_t num_segments_{};
  const vertex_t *vertices_{};

  std::vector<segment_t> segments_{};

  // Randomized segments indices
  std::vector<uint32_t> permutation_{};

  // Query structure for trapezoidation
  std::vector<Trapezoid_t> trapezoids_{};
  std::vector<QNode_t> query_points_{};
  QNode_t *root_{};
  std::vector<QNode_t*> vertex_ynodes_{};
  uint32_t used_trapezoid_count_{};
  uint32_t used_node_count_{};

  // Data for monotonization
  std::vector<bool> visited_trapezoids_{};
  std::list<Monochain_t> monochains_{};

  // Output triangles
  //std::vector<triangle_t> triangles_{};
};

// ----------------------------------------------------------------------------

#ifdef POLYTRI_IMPLEMENTATION

#include "polytri/polygon_triangulation.inl"
#include "polytri/monotone_partitioning.inl"
#include "polytri/trapezoidal_decomposition.inl"

#endif // POLYTRI_IMPLEMENTATION

// ----------------------------------------------------------------------------
