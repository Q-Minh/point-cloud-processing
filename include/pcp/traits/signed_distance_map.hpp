#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

/**
 * @brief
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