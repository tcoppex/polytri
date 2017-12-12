#include "polygon_triangulation.h"

#include <cstring>
#include <algorithm>
#include <iostream>

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::Triangulate(const size_t num_contours,
                                       const uint32_t nvertices_per_contour[],
                                       const vertex_t vertices[],
                                       TriangleBuffer_t &triangles)
{
  assert(0u != num_contours);
  assert(nullptr != nvertices_per_contour);
  assert(nullptr != vertices);

  PolygonTriangulation tri(num_contours, nvertices_per_contour, vertices);
  tri.trapezoidal_decomposition();
  tri.monotone_partitioning();
  tri.triangulate_monotone_polygons(triangles);
}

/* -------------------------------------------------------------------------- */

PolygonTriangulation::PolygonTriangulation(const size_t num_contours,
                                         const uint32_t nvertices_per_contour[],
                                         const vertex_t *vertices) :
  vertices_(vertices)
{
  // Retrieve the total number of segments.
  num_segments_ = 0u;
  for (auto i = 0u; i < num_contours; ++i) {
    num_segments_ += nvertices_per_contour[i];
  }

  // Initialize the list of segments.
  segments_.resize(num_segments_);

  uint32_t vid = 0u;
  for (auto cid = 0u; cid < num_contours; ++cid) {
    const auto nverts = nvertices_per_contour[cid];

    // special case for first segment : save first id.
    uint32_t first_vid = vid;
    segments_[vid].v0 = first_vid;
    segments_[vid].v1 = vid+1u;
    ++vid;
    for (auto i = 1u; i < nverts; ++i) {
      segments_[vid].v0 = segments_[vid-1u].v1;
      segments_[vid].v1 = vid+1u;
      ++vid;
    }
    // special case for last segment : change last vertex id.
    segments_[vid-1u].v1 = first_vid;
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::trapezoidal_decomposition()
{
  /// Note :
  /// Seidel's is a bit differents than that,
  /// it use two loops to construct the trapezoidation in log*(n) with some tricks.

  init_permutation_table();
  init_query_structure();

  while (!permutation_.empty()) {
    add_segment_to_query_structure(new_random_segment_index());
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::monotone_partitioning()
{
  // keep track of visited trapezoids.
  visited_trapezoids_.resize(trapezoids_.size(), false);

  // we start partitioning on a top triangle.
  const auto start_tr = find_top_inside_trapezoid_index();
  assert(kInvalidIndex != start_tr);
  visited_trapezoids_[start_tr] = true;

  // recursively build monotone chain.
  const auto &trapezoid = trapezoids_[start_tr];
  const auto tr_min = trapezoid.min_y;
  const auto tr_max = trapezoid.max_y;

  if (kInvalidIndex != trapezoid.below2) {
    auto *monochain = create_monochain(tr_max, tr_min, InsertRight);
    build_monotone_chains(monochain, trapezoid.below1, start_tr, true);

    monochain = create_monochain(tr_min, tr_max, InsertLeft);
    build_monotone_chains(monochain, trapezoid.below2, start_tr, true);
  } else {
    const auto& right_segment = segments_[trapezoid.right_segment];
    uint32_t max_y, min_y;
    get_max_min_y_indices(right_segment, max_y, min_y);

    const auto right = (tr_min == min_y) ? tr_min : tr_max;
    const auto left  = (tr_min == min_y) ? tr_max : tr_min;
    const auto side  = (tr_min == min_y) ? InsertRight : InsertLeft;
    auto *monochain = create_monochain(left, right, side);
    build_monotone_chains(monochain, trapezoid.below1, start_tr, true);
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::triangulate_monotone_polygons(TriangleBuffer_t &triangles)
{
  // Comparator struct ot find top and bottom most vertices in the list.
  struct MinMaxComparator {
    MinMaxComparator(const vertex_t vertices[]) :
      data(vertices)
    {}
    bool operator() (const uint32_t &a, const uint32_t &b) {
      return data[a].y < data[b].y;
    }
    vertex_t const* data;
  } cmp(vertices_);

  // Triangulate each monochains.
  for (auto& m : monochains_) {
    // find topmost vertex of the monochain
    const auto min_max = std::minmax_element(m.list.begin(), m.list.end(), cmp);

    // first vertex to use depends on the main edge side (opposite to insertion side).
    // Bottommost if insertion is on the left, topmost otherwise.
    ChainIterator_t start_it;
    if (m.insertion_side == InsertLeft) {
      start_it = min_max.first;
    } else {
      start_it = min_max.second;
    }
    triangulate_monochain(m, start_it, triangles);
  }
}

/* -------------------------------------------------------------------------- */

bool PolygonTriangulation::is_angle_convex(uint32_t v0, uint32_t v1, uint32_t v2) const
{
  const auto& A = vertices_[v0];
  const auto& B = vertices_[v1];
  const auto& C = vertices_[v2];

  const auto det = (B.x - A.x) * (C.y - B.y) - (B.y - A.y) * (C.x - B.x);
  return det <= 0.0;
}

/* -------------------------------------------------------------------------- */

namespace {

// Return the next or previous iterator of a double linked list
// without its end marker.
template<typename T>
T next(const T &it, const T &end)
{
  const auto &n = std::next(it);
  return (n == end) ? std::next(n) : n;
}

template<typename T>
T prev(const T &it, const T &end)
{
  const auto &n = std::prev(it);
  return (n == end) ? std::prev(n) : n;
}

}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::triangulate_monochain(Monochain_t &monochain,
                                                 const ChainIterator_t &first,
                                                 TriangleBuffer_t &triangles)
{
  const auto &end = monochain.list.end();

  for (auto current = next(first, end); monochain.list.size() >= 3u;) {
    const auto v0 = *prev(current, end);
    const auto v1 = *current;
    const auto v2 = *next(current, end);

    if (is_angle_convex(v0, v1, v2)) {
      triangles.push_back(triangle_t(v0, v1, v2));
      // remove current vertex from the chain and update position.
      const auto &save = prev(current, end);
      monochain.list.erase(current);
      current = (first == save) ? next(first, end) : save;
    } else {
      current = next(current, end);
    }
  }
}

/* -------------------------------------------------------------------------- */
