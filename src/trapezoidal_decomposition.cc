#include "polygon_triangulation.h"

#include <cmath>
#include <cfloat>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <random>
#include <limits>

#define DEBUG_INFO 0

/* -------------------------------------------------------------------------- */

namespace {

bool is_vertex_lower(const vertex_t &a, const vertex_t &b) {
  assert((fabs(a.x-b.x) > DBL_EPSILON) || (fabs(a.y-b.y) > DBL_EPSILON));
  return ((a.y < b.y) || ((fabs(a.y-b.y) < DBL_EPSILON) && (a.x < b.x)));
}

}

/* -------------------------------------------------------------------------- */

bool PolygonTriangulation::is_top_inside_triangle(const Trapezoid_t &trapezoid) const
{
  // Check if border segments are in direct order.
  if (is_top_triangle(trapezoid)) {
    return (trapezoid.left_segment + 1u) % segments_.size() == trapezoid.right_segment;
  }
  return false;
}

/* -------------------------------------------------------------------------- */

bool PolygonTriangulation::is_top_triangle(const Trapezoid_t &trapezoid) const
{
  if (   (kInvalidIndex == trapezoid.left_segment)
      || (kInvalidIndex == trapezoid.right_segment)) {
    return false;
  }

  const auto& s1 = segments_[trapezoid.left_segment];
  const auto& s2 = segments_[trapezoid.right_segment];

  uint32_t maxy, miny;
  get_max_min_y_indices(s1, maxy, miny);

  if (   (maxy == s2.v0)
      || (maxy == s2.v1)) {
    return true;
  }

  return false;
}

/* -------------------------------------------------------------------------- */

bool PolygonTriangulation::is_bottom_triangle(const Trapezoid_t &trapezoid) const
{
  if (   (kInvalidIndex == trapezoid.left_segment)
      || (kInvalidIndex == trapezoid.right_segment)) {
    return false;
  }

  const auto& s1 = segments_[trapezoid.left_segment];
  const auto& s2 = segments_[trapezoid.right_segment];

  uint32_t maxy, miny;
  get_max_min_y_indices(s1, maxy, miny);

  if (   (miny == s2.v0)
      || (miny == s2.v1)) {
    return true;
  }

  return false;
}

/* -------------------------------------------------------------------------- */

uint32_t PolygonTriangulation::new_random_segment_index()
{
  const uint32_t index = permutation_.back();
  permutation_.pop_back();
  return index;
}

/* -------------------------------------------------------------------------- */

double PolygonTriangulation::distance_from_segment(const vertex_t &v, const segment_t &segment)
{
  uint32_t max_y_index, min_y_index;
  get_max_min_y_indices(segment, max_y_index, min_y_index);

  const auto &A = vertices_[max_y_index];
  const auto &B = vertices_[min_y_index];

  vector_t AB;
  AB.x = B.x - A.x;
  AB.y = B.y - A.y;
  const auto det = -(AB.y * v.x - AB.x * v.y + B.x*A.y - B.y*A.x);

  return det;
}

/* -------------------------------------------------------------------------- */

uint32_t PolygonTriangulation::search_trapezoid_index(const vertex_t &v, const QNode_t *node)
{
  assert(nullptr != node);
  switch (node->type) {
    case X_NODE: {
      return (distance_from_segment(v, segments_[node->key_index]) <= 0.0) ?
            search_trapezoid_index(v, node->left)  :
            search_trapezoid_index(v, node->right) ;
    }

    case Y_NODE: {
      return is_vertex_lower(v, vertices_[node->key_index]) ?
            search_trapezoid_index(v, node->left)  :
            search_trapezoid_index(v, node->right) ;
    }

    case SINK: {
      return node->key_index;
    }
  }

  return kInvalidIndex;
}

/* -------------------------------------------------------------------------- */

uint32_t PolygonTriangulation::get_new_trapezoid_index()
{
  return used_trapezoid_count_++;
}

/* -------------------------------------------------------------------------- */

PolygonTriangulation::QNode_t* PolygonTriangulation::create_node(QNodeType_t type,
                                                                 QNode_t *parent,
                                                                 uint32_t key_index)
{
  const uint32_t node_index = used_node_count_++;
  QNode_t *node = &query_points_[node_index];
  node->type = type;
  node->parent = parent;
  node->key_index = key_index;

  if (SINK == type) {
    link_sink_node_and_trapezoid(node, key_index);
  }

  if (nullptr == parent) {
    root_ = node;
  }

  return node;
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::link_sink_node_and_trapezoid(QNode_t *node, uint32_t trapezoid_index)
{
  assert(node);
  assert(SINK == node->type);
  node->key_index = trapezoid_index;
  trapezoids_[trapezoid_index].sink = node;
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::update_ysplit_trapezoid_neighbors(const uint32_t trapezoid_index)
{
  const auto &trapezoid = trapezoids_[trapezoid_index];

  if (kInvalidIndex != trapezoid.below1) {
    auto &below_trap = trapezoids_[trapezoid.below1];
    below_trap.above1 = (trapezoid.above1 == below_trap.above1) ? trapezoid_index
                                                                : below_trap.above1;
    if (kInvalidIndex != trapezoid.above1)
    below_trap.above2 = (trapezoid.above1 == below_trap.above2) ? trapezoid_index
                                                                : below_trap.above2;
  }
  if (kInvalidIndex != trapezoid.below2) {
    auto &below_trap = trapezoids_[trapezoid.below2];
    below_trap.above1 = (trapezoid.above1 == below_trap.above1) ? trapezoid_index
                                                                : below_trap.above1;
    if (kInvalidIndex != trapezoid.above1)
    below_trap.above2 = (trapezoid.above1 == below_trap.above2) ? trapezoid_index
                                                                : below_trap.above2;
  }
  if (kInvalidIndex != trapezoid.above1) {
    auto &above_trap = trapezoids_[trapezoid.above1];
    above_trap.below1 = (trapezoid.below1 == above_trap.below1) ? trapezoid_index
                                                                : above_trap.below1;
    if (kInvalidIndex != trapezoid.below1)
    above_trap.below2 = (trapezoid.below1 == above_trap.below2) ? trapezoid_index
                                                                : above_trap.below2;
  }
  if (kInvalidIndex != trapezoid.above2) {
    auto &above_trap = trapezoids_[trapezoid.above2];
    above_trap.below1 = (trapezoid.below1 == above_trap.below1) ? trapezoid_index
                                                                : above_trap.below1;
    if (kInvalidIndex != trapezoid.below1)
    above_trap.below2 = (trapezoid.below1 == above_trap.below2) ? trapezoid_index
                                                                : above_trap.below2;
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::add_endpoint_to_query_structure(const uint32_t vertex_index)
{
  if (nullptr != vertex_ynodes_[vertex_index]) {
    return;
  }

  const auto top_trap_index = search_trapezoid_index(vertices_[vertex_index]);
  const auto btm_trap_index = get_new_trapezoid_index();

  auto &top_trapezoid = trapezoids_[top_trap_index];
  auto &btm_trapezoid = trapezoids_[btm_trap_index];
  btm_trapezoid = top_trapezoid;

  QNode_t *sink = top_trapezoid.sink;
  QNode_t *y_node = create_node(Y_NODE, sink->parent, vertex_index);
  y_node->left = create_node(SINK, y_node, btm_trap_index);
  y_node->right = sink;

  if (sink->parent) {
    if (sink->parent->left == sink) {
      sink->parent->left = y_node;
    } else if (sink->parent->right == sink) {
      sink->parent->right = y_node;
    }
  }

  sink->parent = y_node;

  // Quick access to the new y-node.
  vertex_ynodes_[vertex_index] = y_node; //

  // Update trapezoids.
  top_trapezoid.min_y = vertex_index;
  top_trapezoid.below1 = btm_trap_index;
  top_trapezoid.below2 = kInvalidIndex;

  btm_trapezoid.max_y = vertex_index;
  btm_trapezoid.above1 = top_trap_index;
  btm_trapezoid.above2 = kInvalidIndex;

  // The bottom trapezoid is newly created, so we must updated its neighbors as well.
  update_ysplit_trapezoid_neighbors(btm_trap_index);

#if DEBUG_INFO
  fprintf(stderr, "split Y : top %d / bottom %d (at %d = %f).\n",
          top_trap_index, btm_trap_index,
          vertex_index, vertices_[vertex_index].y);
#endif
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::get_max_min_y_indices(const segment_t& s,
                                                 uint32_t &max_y_index,
                                                 uint32_t &min_y_index) const
{
  if (is_vertex_lower(vertices_[s.v0], vertices_[s.v1])) {
    max_y_index = s.v1;
    min_y_index = s.v0;
  } else {
    max_y_index = s.v0;
    min_y_index = s.v1;
  }
}

/* -------------------------------------------------------------------------- */

PolygonTriangulation::QNode_t* PolygonTriangulation::fusion_sinks(QNode_t *top_sink,
                                                                  QNode_t *btm_sink)
{
  auto &top_trap = trapezoids_[top_sink->key_index];
  const auto &btm_trap = trapezoids_[btm_sink->key_index];

  top_trap.min_y = btm_trap.min_y;

  // used with the second version of update_xsplit
  top_trap.below1 = btm_trap.below1;
  top_trap.below2 = btm_trap.below2;

  //top_trap.above1 = (kInvalidIndex != top_trap.above2) ? top_trap.above2 : top_trap.above1;
  //top_trap.above2 = kInvalidIndex; //

  return top_sink;
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::update_trapezoid_aboves(const uint32_t trapezoid_index,
                                                   Trapezoid_t &below) {
  below.above1 = (trapezoid_index == below.above1) ? kInvalidIndex
                                                    : below.above1;
  below.above2 = (trapezoid_index == below.above2) ? kInvalidIndex
                                                    : below.above2;
  /*
  if (kInvalidIndex == below.above1) {
    below.above1 = below.above2;
    below.above2 = kInvalidIndex;
  }
  */
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::update_xsplit_trapezoid_neighbors(const MergeSide_t side,
                                                             const uint32_t left_trap_index,
                                                             const uint32_t right_trap_index)
{
  auto &left_trap = trapezoids_[left_trap_index];
  auto &right_trap = trapezoids_[right_trap_index];

  // We need 'belows' to compute trapezoids and for monotonization,
  // we need 'aboves' only for monotonization.

  if (side == MergeLeft) {

    update_trapezoid_aboves(left_trap_index, trapezoids_[left_trap.below1]);
    if (kInvalidIndex != left_trap.below2) {
      update_trapezoid_aboves(left_trap_index, trapezoids_[left_trap.below2]);
    }

    // ---------

    right_trap.below1 = left_trap.below1;
    right_trap.below2 = left_trap.below2;
    left_trap.below2 = kInvalidIndex;

    auto &right_below1 = trapezoids_[right_trap.below1];
    right_below1.above1 = right_trap_index;

    if (kInvalidIndex != right_trap.below2) {
      trapezoids_[right_trap.below2].above1 = right_trap_index;
    }
  } else if (side == MergeRight) {

    update_trapezoid_aboves(right_trap_index, trapezoids_[right_trap.below1]);
    if (kInvalidIndex != right_trap.below2) {
      update_trapezoid_aboves(right_trap_index, trapezoids_[right_trap.below2]);
    }

    // ---------

    right_trap.below1 = (kInvalidIndex != left_trap.below2) ? left_trap.below2 : left_trap.below1;
    right_trap.below2 = kInvalidIndex;

    auto &below1 = trapezoids_[left_trap.below1];
    if (below1.above1 == kInvalidIndex) {
      below1.above1 = (left_trap_index != below1.above2) ? left_trap_index : below1.above1; //
    } else {
      below1.above2 = left_trap_index;
    }

    if (kInvalidIndex != left_trap.below2) {
      trapezoids_[left_trap.below2].above1 = left_trap_index;
    }
  } else {
    right_trap.below1 = (kInvalidIndex != right_trap.below2) ? right_trap.below2 : right_trap.below1;
    left_trap.below2 = kInvalidIndex;
    right_trap.below2 = kInvalidIndex;

    // When we close the last trapezoid the below trapezoid has 2 aboves,
    // unless the vertex is already segmented.
    if (left_trap.below1 == right_trap.below1) {
      // 1) The below trapezoid is unsegmented.

      // close the trapezoid
      auto &below = trapezoids_[left_trap.below1];
      if (is_bottom_triangle(left_trap)) {
        left_trap.below1 = kInvalidIndex;
        left_trap.below2 = kInvalidIndex;
        below.above2 = right_trap_index;
      } else if (is_bottom_triangle(right_trap)) {
        right_trap.below1 = kInvalidIndex;
        right_trap.below2 = kInvalidIndex;
        below.above1 = left_trap_index;
        below.above2 = (right_trap_index != below.above2) ? below.above2 : kInvalidIndex;
      } else {
        below.above1 = left_trap_index;
        below.above2 = right_trap_index;
      }
    } else {
      // 2) The below trapezoid is segmented.

      auto &left_below = trapezoids_[left_trap.below1];
      left_below.above1 = left_trap_index;
      left_below.above2 = kInvalidIndex;

      auto &right_below = trapezoids_[right_trap.below1];
      right_below.above1 = right_trap_index;
      right_below.above2 = kInvalidIndex;
    }
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::update_node_parent(QNode_t *new_parent, QNode_t *node) {
  if (node->parent->left == node) {
    node->parent->left = new_parent;
  } else if (node->parent->right == node) {
    node->parent->right = new_parent;
  }
  node->parent = new_parent;
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::split_merge_trapezoids(
    const uint32_t segment_index,
    const uint32_t end_y_index,
    const uint32_t trapezoid_index,
    QNode_t *left_fusion_node,
    QNode_t *right_fusion_node
) {
  auto &trapezoid = trapezoids_[trapezoid_index];
  assert(kInvalidIndex != trapezoid_index);
  assert(end_y_index != trapezoid.max_y);

  // Create a new X-node.
  auto *sink = trapezoid.sink;
  auto *parent = sink->parent;
  auto *x_node = create_node(X_NODE, parent, segment_index);

  // Set left / right sink, potentially with fusion.
  x_node->left  = ( left_fusion_node) ? fusion_sinks( left_fusion_node, sink) : sink;
  x_node->right = (right_fusion_node) ? fusion_sinks(right_fusion_node, sink) : sink;
  update_node_parent(x_node, x_node->left);
  update_node_parent(x_node, x_node->right);

  //-----------------------------------------

  // Update x-node sub trapezoids segments.
  const auto left_trap_index = x_node->left->key_index;
  const auto right_trap_index = x_node->right->key_index;
  auto &left_trap = trapezoids_[left_trap_index];
  auto &right_trap = trapezoids_[right_trap_index];

  left_trap.right_segment = segment_index;
  right_trap.left_segment = segment_index;

#if DEBUG_INFO
  fprintf(stderr, "sub-split X : left %d / right %d (trap %d)\n", left_trap_index, right_trap_index, trapezoid_index);
#endif
  assert(kInvalidIndex != trapezoid.min_y);

  // Determine the next fusion side.
  const auto &vertex = vertices_[trapezoid.min_y];
  const auto &segment = segments_[segment_index];
  const auto vertex_distance = distance_from_segment(vertex, segment);
  const auto side = (vertex_distance > +DBL_EPSILON) ? MergeLeft :
                    (vertex_distance < -DBL_EPSILON) ? MergeRight :
                                                        MergeEnd;

  // Update neighborhood depending on the next fusion side.
  update_xsplit_trapezoid_neighbors(side, left_trap_index, right_trap_index);

  // Recursively split merge successive trapezoids.
  // The recursion ends on the final vertex (y_side == 0.0).
  if (side == MergeLeft) {
    split_merge_trapezoids(segment_index, end_y_index, left_trap.below1, x_node->left, nullptr);
  } else if (side == MergeRight) {
    split_merge_trapezoids(segment_index, end_y_index, right_trap.below1, nullptr, x_node->right);
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::compute_offset_vertex(
    const uint32_t max_y_index,
    const uint32_t min_y_index,
    vertex_t &offset
) const {
  const auto eps = 1.0e-3;

  const auto &A = vertices_[max_y_index];
  const auto &B = vertices_[min_y_index];
  offset.x = B.x - A.x;
  offset.y = B.y - A.y;
  const auto invlen = 1.0 / sqrt(offset.x*offset.x + offset.y*offset.y);
  offset.x *= invlen * eps;
  offset.y *= invlen * eps;

  offset.x = A.x + offset.x;
  offset.y = A.y + offset.y;
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::thread_endpoints(
    const uint32_t segment_index,
    const uint32_t max_y_index,
    const uint32_t min_y_index
) {
  // Recursively split and merge trapezoids intersecting the segment.

  // To find the top trapezoid we need to offset the first vertex in the direction
  // of the segment, to distinct between left/right trapezoids neighbors.
  vertex_t v;
  compute_offset_vertex(max_y_index, min_y_index, v);

  /// @note could be constant instead, by storing two below sinks and a segment for each Y.
  /// furthermore this will prevent a bug when trapzeoid collapse (on same Ys).
  const auto top_trapezoid_index = search_trapezoid_index(v, vertex_ynodes_[max_y_index]); //
  const auto new_trapezoid_index = get_new_trapezoid_index();

  auto &top_trapezoid = trapezoids_[top_trapezoid_index];
  auto &new_trapezoid = trapezoids_[new_trapezoid_index];

  // Copy top trapezoid attributes to the new one.
  new_trapezoid = top_trapezoid;

  // Link first splitted trapezoid to the one above it.
  if (kInvalidIndex != top_trapezoid.above2) {
    trapezoids_[top_trapezoid.above2].below1 = new_trapezoid_index;
    new_trapezoid.above1 = top_trapezoid.above2;
  } else {
    auto &above_trapezoid = trapezoids_[top_trapezoid.above1];

    if (above_trapezoid.below2 == kInvalidIndex) {
      // case 1 : new empty trapezoid
      above_trapezoid.below2 = new_trapezoid_index;
    } else if (   (top_trapezoid_index == above_trapezoid.below2)
               && (kInvalidIndex != top_trapezoid.left_segment)) {
      // case 2 : old right trap will become a triangle
      above_trapezoid.below2 = new_trapezoid_index;
    }
  }
  top_trapezoid.above2 = kInvalidIndex;
  new_trapezoid.above2 = kInvalidIndex;

  // Every threading operation create one new sink, used as the right fusion sink
  // by default.
  auto *new_sink = create_node(SINK, root_, new_trapezoid_index);

#if DEBUG_INFO
  fprintf(stderr, "> split X : %d %d\n", max_y_index, min_y_index);
#endif
  split_merge_trapezoids(segment_index, min_y_index, top_trapezoid_index, nullptr, new_sink);

  // Close connections with above trapezoid when one of the start trapezoids
  // (left or right) is a triangle.
  if (is_top_triangle(top_trapezoid)) {
    top_trapezoid.above1 = kInvalidIndex;
  } else if (is_top_triangle(new_trapezoid)) {
    new_trapezoid.above1 = kInvalidIndex;
  }
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::add_segment_to_query_structure(const uint32_t segment_index)
{
  const auto &segment = segments_[segment_index];

  uint32_t min_y_index, max_y_index;
  get_max_min_y_indices(segment, max_y_index, min_y_index);

  add_endpoint_to_query_structure(max_y_index);
  add_endpoint_to_query_structure(min_y_index);
  thread_endpoints(segment_index, max_y_index, min_y_index);
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::init_permutation_table()
{
  permutation_.resize(num_segments_);
  for (auto i = 0u; i < permutation_.size(); ++i) {
    permutation_[i] = num_segments_-i-1;
  }
#if 0
  const auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  fprintf(stderr, "seed used : %lu\n", seed);
  std::shuffle(permutation_.begin(), permutation_.end(), std::default_random_engine(seed));
#endif
}

/* -------------------------------------------------------------------------- */

void PolygonTriangulation::init_query_structure()
{
  vertex_ynodes_.resize(num_segments_ + 1u, nullptr);
  query_points_.resize(8u * num_segments_);
  trapezoids_.resize(4u * num_segments_);

  // Default empty trapezoid.
  create_node(SINK, nullptr, get_new_trapezoid_index());
}
