#ifndef PCP_TRAITS_INDEX_MAP_HPP
#define PCP_TRAITS_INDEX_MAP_HPP

/**
 * @file
 * @ingroup traits
 */

#include <type_traits>

namespace pcp {
namespace traits {

/**
 * @ingroup traits-property-maps
 * @brief
 * Compile-time check for IndexMap concept.
 * The IndexMap concept requires IndexMap to be a callable type which takes a parameter
 * of type Key and returns an integral type (short, int, unsigned int, long, etc.)
 * @tparam IndexMap
 * @tparam Key
 */
template <class IndexMap, class Key>
static constexpr bool is_index_map_v = std::is_integral_v<std::invoke_result_t<IndexMap, Key>>;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_INDEX_MAP_HPP