#ifndef PCP_TRAITS_COORDINATE_MAP_HPP
#define PCP_TRAITS_COORDINATE_MAP_HPP

#include <array>
#include <type_traits>

namespace pcp {
namespace traits {

template <class CoordinateMap, class Key, class CoordinateType, std::size_t Dims>
static constexpr bool is_coordinate_map_v =
    std::is_same_v<std::invoke_result_t<CoordinateMap, Key>, std::array<CoordinateType, Dims>>;

template <class CoordinateMap, class Key>
static constexpr std::size_t dimensionality =
    std::tuple_size<std::invoke_result_t<CoordinateMap, Key>>::value;

template <class CoordinateMap, class Key>
using coordinate_type = typename std::invoke_result_t<CoordinateMap, Key>::value_type;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_COORDINATE_MAP_HPP
