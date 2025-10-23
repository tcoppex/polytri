
/* -------------------------------------------------------------------------- */

namespace {

bool is_vertex_lower(
  const PolyTri::vertex_t &a,
  const PolyTri::vertex_t &b
) {
#if POLYTRI_DEBUG_INFO
  // fprintf(stderr, "a %.3f %.3f / b %.3f %.3f\n", a.x, a.y, b.x, b.y);
#endif

  // assert((fabs(a.x-b.x) > DBL_EPSILON) || (fabs(a.y-b.y) > DBL_EPSILON));
  if ((fabs(a.x-b.x) <= DBL_EPSILON) && (fabs(a.y-b.y) <= DBL_EPSILON)) {
    fprintf(stderr, "a %.3f %.3f / b %.3f %.3f\n", a.x, a.y, b.x, b.y);
    assert(0 && "Two vertices are the same");
  }

  return ((a.y < b.y) || ((fabs(a.y-b.y) <= DBL_EPSILON) && (a.x < b.x)));
}

}

/* -------------------------------------------------------------------------- */

bool PolyTri::is_top_inside_triangle(const Trapezoid_t &trapezoid) const
{
  POLYTRI_LOG("(%s)\n", __FUNCTION__);

  // Check if border segments are in direct order.
  if (is_top_triangle(trapezoid)) {
    auto const side0 = (trapezoid.left_segment + num_segments_ - 1u) % num_segments_;
    auto const side1 = (trapezoid.right_segment + 1u) % num_segments_;

    POLYTRI_LOG("> is top, side0 = %d, side1 = %d (L = %u, R = %u)\n",
      side0, side1, trapezoid.left_segment, trapezoid.right_segment
    );

    return (side0 == trapezoid.right_segment)
        && (side1 == trapezoid.left_segment)
        ;
  }
  return false;
}

/* -------------------------------------------------------------------------- */

bool PolyTri::is_top_triangle(const Trapezoid_t &trapezoid) const
{
  if ((kInvalidIndex == trapezoid.left_segment)
   || (kInvalidIndex == trapezoid.right_segment)) {
    // POLYTRI_LOG("%s left or right segment of the trap non existent\n", __FUNCTION__);
    return false;
  }

  const auto& left = segments_[trapezoid.left_segment];
  const auto& right = segments_[trapezoid.right_segment];

  uint32_t left_maxy{}, _{};
  get_max_min_y_indices(left, left_maxy, _);

  if ((left_maxy == right.v0)
   || (left_maxy == right.v1)) {
    POLYTRI_LOG("%s: right segment has left max y\n", __FUNCTION__);
    return true;
  }

  return false;
}

/* -------------------------------------------------------------------------- */

bool PolyTri::is_bottom_triangle(const Trapezoid_t &trapezoid) const
{
  if ((kInvalidIndex == trapezoid.left_segment)
   || (kInvalidIndex == trapezoid.right_segment)) {
    // POLYTRI_LOG("%s left or right segment of the trap non existent\n", __FUNCTION__);
    return false;
  }

  const auto& left = segments_[trapezoid.left_segment];
  const auto& right = segments_[trapezoid.right_segment];

  uint32_t _{}, left_miny{};
  get_max_min_y_indices(left, _, left_miny);

  if ((left_miny == right.v0)
   || (left_miny == right.v1)) {
    POLYTRI_LOG("%s right segment has left min y\n", __FUNCTION__);
    return true;
  }

  return false;
}

/* -------------------------------------------------------------------------- */

uint32_t PolyTri::new_random_segment_index()
{
  const uint32_t index = permutation_.back();
  permutation_.pop_back();
  return index;
}

/* -------------------------------------------------------------------------- */

double PolyTri::distance_from_segment(const vertex_t &v, const segment_t &segment)
{
  uint32_t max_y_index{}, min_y_index{};
  get_max_min_y_indices(segment, max_y_index, min_y_index);

  const auto &A = vertices_[max_y_index];
  const auto &B = vertices_[min_y_index];

  vertex_t AB;
  AB.x = B.x - A.x;
  AB.y = B.y - A.y;

  double d = -(AB.y * v.x - AB.x * v.y + B.x*A.y - B.y*A.x);

  // POLYTRI_LOG("%f\n", d);

  return d;
}

/* -------------------------------------------------------------------------- */

uint32_t PolyTri::search_trapezoid_index(const vertex_t &v, const QNode_t *node)
{
  assert(nullptr != node);

  POLYTRI_LOG("[node %d](key id %d) v(%3f, %.3f)\n", node->type, node->key_index, v.x, v.y);

  switch (node->type) {
    case X_NODE: {
      auto const d = distance_from_segment(v, segments_[node->key_index]);
      return (d <= DBL_EPSILON) ?
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

uint32_t PolyTri::get_new_trapezoid_index() {
  return used_trapezoid_count_++;
}

/* -------------------------------------------------------------------------- */

PolyTri::QNode_t* PolyTri::create_node(
  QNodeType_t type,
  QNode_t *parent,
  uint32_t key_index
) {
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

void PolyTri::link_sink_node_and_trapezoid(QNode_t *node, uint32_t trapezoid_index)
{
  // POLYTRI_LOG("%s %p %d\n", __FUNCTION__, (void*)node, trapezoid_index);

  assert(node);
  assert(SINK == node->type);

  node->key_index = trapezoid_index;
  trapezoids_[trapezoid_index].sink = node;
}

/* -------------------------------------------------------------------------- */

void PolyTri::update_ysplit_trapezoid_neighbors(const uint32_t trapezoid_index)
{
  // POLYTRI_LOG("%s %d\n", __FUNCTION__, trapezoid_index);

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

void PolyTri::add_endpoint_to_query_structure(const uint32_t vertex_index)
{
  // POLYTRI_LOG("%s %d\n", __FUNCTION__, vertex_index);

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

  POLYTRI_LOG("split Y : top %d / bottom %d (at v[%d].y = %.3f).\n",
          top_trap_index, btm_trap_index,
          vertex_index, vertices_[vertex_index].y);
}

/* -------------------------------------------------------------------------- */

void PolyTri::get_max_min_y_indices(
  const segment_t& s,
  uint32_t &max_y_index,
  uint32_t &min_y_index
) const {
  if (is_vertex_lower(vertices_[s.v0], vertices_[s.v1])) {
    max_y_index = s.v1;
    min_y_index = s.v0;
  } else {
    max_y_index = s.v0;
    min_y_index = s.v1;
  }
}

/* -------------------------------------------------------------------------- */

PolyTri::QNode_t* PolyTri::fusion_sinks(
  QNode_t *top_sink,
  QNode_t *btm_sink
) {
  // POLYTRI_LOG("%s %p %p\n", __FUNCTION__, (void*)top_sink, (void*)btm_sink);

  auto &top_trap = trapezoids_[top_sink->key_index];
  const auto &btm_trap = trapezoids_[btm_sink->key_index];

  top_trap.min_y = btm_trap.min_y;

  // used with the second version of update_xsplit
  top_trap.below1 = btm_trap.below1;
  top_trap.below2 = btm_trap.below2;

  // top_trap.above1 = (kInvalidIndex != top_trap.above2) ? top_trap.above2 : top_trap.above1;
  // top_trap.above2 = kInvalidIndex; //

  return top_sink;
}

/* -------------------------------------------------------------------------- */

void PolyTri::update_trapezoid_aboves(
  const uint32_t trapezoid_index,
  Trapezoid_t &below
) {
  // POLYTRI_LOG("%s %d\n", __FUNCTION__, trapezoid_index);

  if (below.above1 == trapezoid_index) {
    below.above1 = kInvalidIndex;
  }

  if (below.above2 == trapezoid_index) {
    below.above2 = kInvalidIndex;
  }

#if 0
  if (kInvalidIndex == below.above1) {
    below.above1 = below.above2;
    below.above2 = kInvalidIndex;
  }
#endif
}

/* -------------------------------------------------------------------------- */

void PolyTri::update_xsplit_trapezoid_neighbors(
  const MergeSide_t side,
  const uint32_t left_trap_index,
  const uint32_t right_trap_index
) {
  POLYTRI_LOG("%s %d %d %d\n", __FUNCTION__, side, left_trap_index, right_trap_index);

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

    auto &below1 = trapezoids_[right_trap.below1];
    below1.above1 = right_trap_index;

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
    // END TRAPEZOID

    right_trap.below1 = (kInvalidIndex != right_trap.below2) ? right_trap.below2
                                                             : right_trap.below1
                                                             ;
    right_trap.below2 = kInvalidIndex;
    left_trap.below2 = kInvalidIndex;

    // When we close the last trapezoid the below trapezoid has 2 aboves,
    // unless the vertex is already segmented.
    if (left_trap.below1 == right_trap.below1) {
      // 1) The below trapezoid is unsegmented.

      // close the trapezoid
      auto &below = trapezoids_[left_trap.below1];
      if (is_bottom_triangle(left_trap)) {
        POLYTRI_LOG("> left trap is bottom triangle\n");
        left_trap.below1 = kInvalidIndex;
        left_trap.below2 = kInvalidIndex;
        below.above2 = right_trap_index;
      } else if (is_bottom_triangle(right_trap)) {
        POLYTRI_LOG("> right trap is bottom triangle\n");
        right_trap.below1 = kInvalidIndex;
        right_trap.below2 = kInvalidIndex;
        below.above1 = left_trap_index;
        below.above2 = (right_trap_index != below.above2) ? below.above2
                                                          : kInvalidIndex
                                                          ;
      } else {
        POLYTRI_LOG("> neither traps are bottom triangle\n");
        below.above1 = left_trap_index;
        below.above2 = right_trap_index;
      }
    } else {
      // 2) The below trapezoid is segmented.
        POLYTRI_LOG("> below trap is segmented\n");

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

void PolyTri::update_node_parent(QNode_t *new_parent, QNode_t *node) {
  if (node->parent->left == node) {
    node->parent->left = new_parent;
  } else if (node->parent->right == node) {
    node->parent->right = new_parent;
  }
  node->parent = new_parent;
}

/* -------------------------------------------------------------------------- */

void PolyTri::split_merge_trapezoids(
  const uint32_t segment_index,
  const uint32_t end_y_index,
  const uint32_t trapezoid_index,
  QNode_t *left_fusion_node,
  QNode_t *right_fusion_node
) {
  // POLYTRI_LOG(
  //   "%s %d %d %d %p %p\n", __FUNCTION__,
  //   segment_index, end_y_index, trapezoid_index,
  //   (void*)left_fusion_node, (void*)right_fusion_node
  // );

  auto &trapezoid = trapezoids_[trapezoid_index];
  assert(kInvalidIndex != trapezoid_index);
  assert(end_y_index != trapezoid.max_y);

  // Create a new X-node.
  auto *sink = trapezoid.sink;
  auto *parent = sink->parent;
  auto *x_node = create_node(X_NODE, parent, segment_index);

  // Set left / right sink, potentially with fusion.
  x_node->left  = ( left_fusion_node) ? fusion_sinks( left_fusion_node, sink)
                                      : sink;
  x_node->right = (right_fusion_node) ? fusion_sinks(right_fusion_node, sink)
                                      : sink;
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

  assert(kInvalidIndex != trapezoid.min_y);

  // Determine the next fusion side.
  const auto &vertex = vertices_[trapezoid.min_y];
  const auto &segment = segments_[segment_index];

  const auto vertex_distance = distance_from_segment(vertex, segment);

  const auto side = (vertex_distance > +DBL_EPSILON) ? MergeLeft
                  : (vertex_distance < -DBL_EPSILON) ? MergeRight
                                                     : MergeEnd
                                                     ;

  POLYTRI_LOG("sub-split X (trap %d) : left-right trap  %d / %d || side %d\n",
    trapezoid_index, left_trap_index, right_trap_index, side
  );

  // Update neighborhood depending on the next fusion side.
  update_xsplit_trapezoid_neighbors(side, left_trap_index, right_trap_index);

  // Recursively split merge successive trapezoids.
  // The recursion ends on the final vertex (y_side == 0.0).
  if (side == MergeLeft) {
    split_merge_trapezoids(
      segment_index, end_y_index, left_trap.below1, x_node->left, nullptr
    );
  } else if (side == MergeRight) {
    split_merge_trapezoids(
      segment_index, end_y_index, right_trap.below1, nullptr, x_node->right
    );
  }
}

/* -------------------------------------------------------------------------- */

void PolyTri::compute_offset_vertex(
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

void PolyTri::thread_endpoints(
    const uint32_t segment_index,
    const uint32_t max_y_index,
    const uint32_t min_y_index
) {
  // Recursively split and merge trapezoids intersecting the segment.

  // To find the top trapezoid we need to offset the first vertex in the direction
  // of the segment, to distinct between left/right trapezoids neighbors.
  vertex_t v{};
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

  POLYTRI_LOG("> split X, max-min y index : %d %d\n", max_y_index, min_y_index);

  split_merge_trapezoids(
    segment_index, min_y_index, top_trapezoid_index, nullptr, new_sink
  );

  // Close connections with above trapezoid when one of the start trapezoids
  // (left or right) is a triangle.
  if (is_top_triangle(top_trapezoid)) {
    POLYTRI_LOG("> top trap is top\n");
    top_trapezoid.above1 = kInvalidIndex;
  } else if (is_top_triangle(new_trapezoid)) {
    POLYTRI_LOG("> new trap is top\n");
    new_trapezoid.above1 = kInvalidIndex;
  }
}

/* -------------------------------------------------------------------------- */

void PolyTri::add_segment_to_query_structure(const uint32_t segment_index)
{
  const auto &segment = segments_[segment_index];

  uint32_t min_y_index{}, max_y_index{};
  get_max_min_y_indices(segment, max_y_index, min_y_index);

  add_endpoint_to_query_structure(max_y_index);
  add_endpoint_to_query_structure(min_y_index);
  thread_endpoints(segment_index, max_y_index, min_y_index);
}

/* -------------------------------------------------------------------------- */

void PolyTri::init_permutation_table()
{
  // assert(num_segments_ == segments_.size());
  POLYTRI_LOG("%s %d %lu\n", __FUNCTION__, num_segments_, segments_.size());

  permutation_.resize(num_segments_);
  for (uint32_t i = 0u; i < num_segments_; ++i) {
    permutation_[i] = (num_segments_ - 1) - i;
  }

#if POLYTRI_ENABLE_PERMUTATION
  // 1761133065998441995
  const auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  POLYTRI_LOG("seed used : %lu\n", seed);
  std::shuffle(
    permutation_.begin(), permutation_.end(), std::default_random_engine(seed)
  );
#endif
}

/* -------------------------------------------------------------------------- */

void PolyTri::init_query_structure()
{
  vertex_ynodes_.resize(num_segments_ + 1u, nullptr);
  query_points_.resize(8u * num_segments_);
  trapezoids_.resize(4u * num_segments_);

  // Default empty trapezoid.
  create_node(SINK, nullptr, get_new_trapezoid_index());
}

/* -------------------------------------------------------------------------- */
