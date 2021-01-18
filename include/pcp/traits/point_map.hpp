#ifndef PCP_TRAITS_POINT_MAP_HPP
#define PCP_TRAITS_POINT_MAP_HPP

/**
 * @file
 * @ingroup traits
 */

#include "point_traits.hpp"

namespace pcp {
namespace traits {

/**
 * @ingroup traits-property-maps
 * @brief
 * Compile-time check for PointMap concept.
 * The PointMap concept requires PointMap to be a callable type which takes a parameter
 * of type Key and returns an object whose type satisfies the Point concept.
 * @tparam PointMap
 * @tparam Key
 */
template <class PointMap, class Key>
static constexpr bool is_point_map_v = is_point_v<std::invoke_result_t<PointMap, Key>>;

/**
 * @ingroup traits-property-maps
 * @brief
 * Compile-time check for PointViewMap concept.
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

#endif // PCP_TRAITS_POINT_MAP_HPP