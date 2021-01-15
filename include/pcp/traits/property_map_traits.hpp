#ifndef PCP_TRAITS_PROPERTY_MAP_TRAITS_HPP
#define PCP_TRAITS_PROPERTY_MAP_TRAITS_HPP

/**
 * @file
 * @ingroup traits
 */

#include <tuple>
#include <type_traits>

namespace pcp {
namespace traits {

/**
 * @ingroup traits
 * @brief
 * Compile-time type inspection for property maps.
 * @tparam PropertyMap
 * @tparam ...Args
 */
template <class PropertyMap, class... Args>
struct property_map_traits
{
    using value_type =
        std::invoke_result_t<PropertyMap, Args...>; //!< Return type of the property map
    using key_type =
        typename std::tuple_element<0, std::tuple<Args...>>::type; //!< Type of the first parameter
                                                                   //!< of the property map

    template <std::size_t N>
    using parameter_type =
        typename std::tuple_element<N, std::tuple<Args...>>::type; //!< Type of the n-th parameter
                                                                   //!< of the property map

    static constexpr int parameter_size =
        std::tuple_size<std::tuple<Args...>>::value; //!< Number of parameters of the property map
};

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_PROPERTY_MAP_TRAITS_HPP