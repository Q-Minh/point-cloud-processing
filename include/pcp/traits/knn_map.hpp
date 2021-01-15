#ifndef PCP_TRAITS_KNN_MAP_HPP
#define PCP_TRAITS_KNN_MAP_HPP

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
 * The KnnMap concept requires KnnMap to be a callable type which takes a parameter
 * of type Key and returns a range with begin and end iterators.
 * @tparam KnnMap
 * @tparam Key
 */
template <class KnnMap, class Key, class = void>
struct is_knn_map : std::false_type
{
};

template <class KnnMap, class Key>
struct is_knn_map<
    KnnMap,
    Key,
    std::void_t<
        decltype(std::declval<std::invoke_result_t<KnnMap, Key>&>().begin()),
        decltype(std::declval<std::invoke_result_t<KnnMap, Key>&>().end())>> : std::true_type
{
};

/**
 * @ingroup traits-property-maps
 * @brief
 * Compile-time check for KnnMap concept
 * @tparam KnnMap
 * @tparam Key
 */
template <class KnnMap, class Key>
static constexpr bool is_knn_map_v = is_knn_map<KnnMap, Key>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_KNN_MAP_HPP