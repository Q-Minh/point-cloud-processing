#ifndef PCP_TRAITS_SIGNED_DISTANCE_MAP_HPP
#define PCP_TRAITS_SIGNED_DISTANCE_MAP_HPP

/**
 * @file
 * @ingroup traits
 */

#include <type_traits>

namespace pcp {
namespace traits {

/**
 * @ingroup traits
 * @brief
 * Compile-time check for SignedDistanceMap concept.
 * The SignedDistanceMap concept requires SignedDistanceMap to be a callable type which takes a
 * parameter of type Key and returns an arithmetic type (most often float or double).
 * @tparam SignedDistanceMap
 * @tparam Key
 */
template <class SignedDistanceMap, class Key>
static constexpr bool is_signed_distance_map_v =
    std::is_arithmetic_v<std::invoke_result_t<SignedDistanceMap, Key>>;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_SIGNED_DISTANCE_MAP_HPP