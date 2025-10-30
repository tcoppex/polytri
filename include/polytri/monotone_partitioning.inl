
/* -------------------------------------------------------------------------- */

PolyTri::InsertionSide_t PolyTri::GetIntersectionSide(
  const bool min_is_right,
  const bool go_down
) {
  return (min_is_right == go_down) ? PolyTri::InsertRight
                                   : PolyTri::InsertLeft
                                   ;
}

/* -------------------------------------------------------------------------- */

uint32_t PolyTri::find_top_inside_trapezoid_index() const {
  POLYTRI_LOG("%s\n", __FUNCTION__);

  for (uint32_t i = 0u; i < trapezoids_.size(); ++i) {
    if (is_top_inside_triangle(i)) {
      return i;
    }
  }
  return kInvalidIndex;
}

/* -------------------------------------------------------------------------- */

void PolyTri::add_vertex_to_monochain(
  const Trapezoid_t &trapezoid,
  const bool go_down,
  Monochain_t *monochain
) {
  /// * Insertion order depends on the side :
  /// When is left : up pushed back, down pushed front
  /// When is right : down pushed back, up pushed front

  const auto vertex_index = (go_down) ? trapezoid.min_y
                                      : trapezoid.max_y
                                      ;

  /// To insert vertices in direct order we look for the direction.
  if ((InsertRight == monochain->insertion_side) == go_down) {
    POLYTRI_LOG("> %s: push vertex %u back\n", __FUNCTION__, vertex_index);
    monochain->list.push_back(vertex_index);
  } else {
    POLYTRI_LOG("> %s: push vertex %u front\n", __FUNCTION__, vertex_index);
    monochain->list.push_front(vertex_index);
  }
}

/* -------------------------------------------------------------------------- */

PolyTri::Monochain_t* PolyTri::create_monochain(
    const uint32_t first_index,
    const uint32_t second_index,
    const InsertionSide_t side
) {
  POLYTRI_LOG("! %s((first vertices) %u -> %u, (insertion side) %s)\n", __FUNCTION__,
    first_index, second_index, (side == InsertLeft) ? "left" : "right"
  );

  /// * For a monochain, vertices are always added to the same side, left or right.
  /// This is due to the nature of monochain and the vertical trapezoidation.
  ///
  /// * We need to know the left or right position only when pushing the first
  /// two vertices, For this we check if vertices belong to left / right segment,
  /// if they both does not belong to it (2 middles vertices), we took the min
  /// and max values.
  ///
  /// * When entering a new monochain we know the side of insertion by looking
  /// at the entering side edge value and comparing it to the trapezoid values
  /// (ie. by entering on the right, the right edges must include both the max and
  /// min trapezoid y index to be the inserting side).
  ///  If both diagonal extremities are middle vertices, the main edge is the
  /// opposite one (of previous SIDE), otherwise it MUST contains one of the
  /// trapezoid vertex (min / max)
  ///
  /// * [ the real special case is diagonals with middle vertices, all other cases
  /// follow the sames rules ]
  ///

  monochains_.emplace_back(Monochain_t(side, {first_index, second_index}));
  return &monochains_.back();
}

/* -------------------------------------------------------------------------- */

void PolyTri::select_monotone_path(
  const uint32_t trapezoid_index,
  const bool go_down,
  const bool come_from_left
) {
  const auto &trapezoid = trapezoids_[trapezoid_index];

  POLYTRI_LOG("%s((trap_id) %u)\n", __FUNCTION__, trapezoid_index);

  /* When both the left & right segments are invalid we are outside
   * the polygon and the monotone partitionning failed. */
  assert(kInvalidIndex != trapezoid.left_segment);
  assert(kInvalidIndex != trapezoid.right_segment);

  const auto tr_max = trapezoid.max_y;
  const auto tr_min = trapezoid.min_y;

  bool top_left{}, btm_left{};
  if (kInvalidIndex != trapezoid.left_segment) {
    uint32_t left_max_y{}, left_min_y{};
    const auto &left_segment = segments_[trapezoid.left_segment];
    get_max_min_y_indices(left_segment, left_max_y, left_min_y);
    top_left = (tr_max == left_max_y);
    btm_left = (tr_min == left_min_y);
  }

  bool top_right{}, btm_right{};
  if (kInvalidIndex != trapezoid.right_segment) {
    uint32_t right_max_y{}, right_min_y{};
    const auto &right_segment = segments_[trapezoid.right_segment];
    get_max_min_y_indices(right_segment, right_max_y, right_min_y);
    top_right = (tr_max == right_max_y);
    btm_right = (tr_min == right_min_y);
  }

  POLYTRI_LOG("> %d%d%d%d\n", top_left, btm_left, top_right, btm_right);

  const bool top_middle = (!top_left && !top_right);  //  0x0x
  const bool btm_middle = (!btm_left && !btm_right);  //  x0x0
  const bool top_flat = (top_left && top_right);      //  1x1x
  const bool btm_flat = (btm_left && btm_right);      //  x1x1

  if (!((top_left && btm_left) || (top_right && btm_right))) {
    if (top_flat || btm_flat) {
      // (should not happens)
    } else if (top_middle && btm_middle) {
      if (come_from_left) {
        auto *new_monochain = create_monochain(tr_min, tr_max, InsertLeft);
        build_monotone_chains(new_monochain, trapezoid.above2, trapezoid_index, false);
        build_monotone_chains(new_monochain, trapezoid.below2, trapezoid_index, true);
      } else {
        auto *new_monochain = create_monochain(tr_max, tr_min, InsertRight);
        build_monotone_chains(new_monochain, trapezoid.below1, trapezoid_index, true);
        build_monotone_chains(new_monochain, trapezoid.above1, trapezoid_index, false);
      }
    } else {
      const auto left = (top_left || (top_middle && btm_right)) ? tr_max : tr_min;
      const auto right = (left == tr_min) ? tr_max : tr_min;
      const bool min_is_right = (right == tr_min);

      if (!top_middle && !btm_middle) {
        if (go_down) {
          auto *new_monochain = create_monochain(left, right, GetIntersectionSide(min_is_right, true));
          build_monotone_chains(new_monochain, trapezoid.below1, trapezoid_index, true);
        } else {
          auto *new_monochain = create_monochain(right, left, GetIntersectionSide(min_is_right, false));
          build_monotone_chains(new_monochain, trapezoid.above1, trapezoid_index, false);
        }
      } else if (btm_middle) {
        if (top_left) {
          if (!go_down && come_from_left) {
            auto *new_monochain = create_monochain(right, left, InsertLeft);
            build_monotone_chains(new_monochain, trapezoid.below2, trapezoid_index, true);
            build_monotone_chains(new_monochain, trapezoid.above1, trapezoid_index, false);
          } else {
            auto *new_monochain = create_monochain(left, right, GetIntersectionSide(min_is_right, true));
            build_monotone_chains(new_monochain, trapezoid.below1, trapezoid_index, true);
          }
        } else if (top_right) {
          if (!go_down && !come_from_left) {
            auto *new_monochain = create_monochain(right, left, InsertRight);
            build_monotone_chains(new_monochain, trapezoid.below1, trapezoid_index, true);
            build_monotone_chains(new_monochain, trapezoid.above1, trapezoid_index, false);
          } else {
            auto *new_monochain = create_monochain(left, right, GetIntersectionSide(min_is_right, true));
            build_monotone_chains(new_monochain, trapezoid.below2, trapezoid_index, true);
          }
        }
      } else if (top_middle) {
        if (btm_left) {
          if (go_down && come_from_left) {
            auto *new_monochain = create_monochain(left, right, InsertLeft);
            build_monotone_chains(new_monochain, trapezoid.below1, trapezoid_index, true);
            build_monotone_chains(new_monochain, trapezoid.above2, trapezoid_index, false);
          } else {
            auto *new_monochain = create_monochain(right, left, GetIntersectionSide(min_is_right, false));
            build_monotone_chains(new_monochain, trapezoid.above1, trapezoid_index, false);
          }
        } else if (btm_right) {
          if (go_down && !come_from_left) {
            auto *new_monochain = create_monochain(left, right, InsertRight);
            build_monotone_chains(new_monochain, trapezoid.below1, trapezoid_index, true);
            build_monotone_chains(new_monochain, trapezoid.above1, trapezoid_index, false);
          } else {
            auto *new_monochain = create_monochain(right, left, GetIntersectionSide(min_is_right, false));
            build_monotone_chains(new_monochain, trapezoid.above2, trapezoid_index, false);
          }
        }
      }
    }
  } else {
    POLYTRI_LOG("> Monotone path stopped (%d%d%d%d)\n",
      top_left, btm_left, top_right, btm_right
    );
  }
}

/* -------------------------------------------------------------------------- */

void PolyTri::build_monotone_chains(
  Monochain_t *monochain,
  const uint32_t trapezoid_index,
  const uint32_t from_index,
  const bool go_down
) {
  if (kInvalidIndex == trapezoid_index) {
    return;
  }
  POLYTRI_LOG("%s((trap_id) %u, (from_id) %u, %s)\n", __FUNCTION__,
    trapezoid_index, from_index, go_down ? "go down" : "go up");

  const auto &trapezoid = trapezoids_[trapezoid_index];

  if (visited_trapezoids_[trapezoid_index]) {
    POLYTRI_LOG("%s((trap_id) %u) already visited, stop.\n",
      __FUNCTION__, trapezoid_index);
    return;
  }
  visited_trapezoids_[trapezoid_index] = true;

  add_vertex_to_monochain(trapezoid, go_down, monochain);

  // Determine if the current search come from the first children of the trapezoids.
  const bool come_from_left = (from_index == trapezoid.above1)
                           || (from_index == trapezoid.below1)
                           ;
  select_monotone_path(trapezoid_index, go_down, come_from_left);

  // Continue in the same direction.
  if (go_down) {
    build_monotone_chains(monochain, trapezoid.below1, trapezoid_index, true);
    build_monotone_chains(monochain, trapezoid.below2, trapezoid_index, true);
  } else {
    build_monotone_chains(monochain, trapezoid.above1, trapezoid_index, false);
    build_monotone_chains(monochain, trapezoid.above2, trapezoid_index, false);
  }
}

/* -------------------------------------------------------------------------- */
