#pragma once

#include "point_traits.hpp"

namespace pcp {
namespace traits {

/**
 * @brief
 * The PointMap concept requires PointMap to be a callable type which takes a parameter
 * of type Key and returns an object whose type satisfies the Point concept.
 * @tparam PointMap
 * @tparam Key
 */
template <class PointMap, class Key>
static constexpr bool is_point_map_v = is_point_v<std::invoke_result_t<PointMap, Key>>;

/**
 * @brief
 * The PointViewMap concept requires PointViewMap to be a callable type which takes a parameter
 * of type Key and returns an object whose type satisfies the PointView concept.
 * @tparam PointViewMap
 * @tparam Key
 */
template <class PointViewMap, class Key>
static constexpr bool is_point_view_map_v =
    is_point_view_v<std::invoke_result_t<PointViewMap, Key>>;

} // namespace traits
} // namespace pcp