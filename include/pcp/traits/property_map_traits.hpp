#pragma once

#include <tuple>
#include <type_traits>

namespace pcp {
namespace traits {

template <class PropertyMap, class... Args>
struct property_map_traits
{
    using value_type = std::invoke_result_t<PropertyMap, Args...>;
    using key_type = typename std::tuple_element<0, std::tuple<Args...>>::type;

    template <std::size_t N>
    using parameter_type = typename std::tuple_element<N, std::tuple<Args...>>::type;

    static constexpr int parameter_size = std::tuple_size<std::tuple<Args...>>::value;
};

} // namespace traits
} // namespace pcp