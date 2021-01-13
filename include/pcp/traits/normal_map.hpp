#pragma once

#include "normal_traits.hpp"

namespace pcp {
namespace traits {

/**
 * @brief
 * The NormalMap concept requires NormalMap to be a callable type which takes a parameter
 * of type Key and returns an object whose type satisfies the Normal concept.
 * @tparam NormalMap
 * @tparam Key
 */
template <class NormalMap, class Key>
static constexpr bool is_normal_map_v = is_normal_v<std::invoke_result_t<NormalMap, Key>>;

} // namespace traits
} // namespace pcp