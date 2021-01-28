#ifndef PCP_TRAITS_IDENTITY_MAP_HPP
#define PCP_TRAITS_IDENTITY_MAP_HPP

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
 * The IdentityMap concept requires IdentityMap to be a callable type which takes a parameter
 * of type Key and returns an instance of a type that is equality & inequality comparable.
 * @tparam IdentityMap
 * @tparam Key
 */
template <class IdentityMap, class Key, class = void>
struct is_identity_map : std::false_type
{
};

template <class IdentityMap, class Key>
struct is_identity_map<
    IdentityMap,
    Key,
    std::void_t<
        decltype(
            std::declval<std::invoke_result_t<IdentityMap, Key>&>() ==
            std::declval<std::invoke_result_t<IdentityMap, Key>&>()),
        decltype(
            std::declval<std::invoke_result_t<IdentityMap, Key>&>() ==
            std::declval<std::invoke_result_t<IdentityMap, Key>&>())>> : std::true_type
{
};

/**
 * @ingroup traits-property-maps
 * @brief
 * Compile-time check for IdentityMap concept
 * @tparam IdentityMap
 * @tparam Key
 */
template <class IdentityMap, class Key>
static constexpr bool is_identity_map_v = !std::is_void_v<std::invoke_result_t<IdentityMap, Key>> &&
                                          is_identity_map<IdentityMap, Key>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_IDENTITY_MAP_HPP