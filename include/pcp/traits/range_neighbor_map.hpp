#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class RangeNeighborMap, class Key, class Range, class = void>
struct is_range_neighbor_map : std::false_type
{
};

template <class RangeNeighborMap, class Key, class Range>
struct is_range_neighbor_map<
    RangeNeighborMap,
    Key,
    Range,
    std::void_t<
        decltype(std::declval<std::invoke_result_t<RangeNeighborMap, Key, Range>&>().begin()),
        decltype(std::declval<std::invoke_result_t<RangeNeighborMap, Key, Range>&>().end())>>
    : std::true_type
{
};

/**
 * @brief
 * The RangeNeighborMap concept requires RangeNeighborMap to be a callable type which takes a
 * parameter of type Key and a parameter of type Range and returns a range with begin and end
 * iterators.
 * @tparam RangeNeighborMap
 * @tparam Key
 * @tparam Range
 * Must satisfy Range concept, which means it must support geometric intersection tests.
 */
template <class RangeNeighborMap, class Key, class Range>
static constexpr bool is_range_neighbor_map_v =
    is_range_neighbor_map<RangeNeighborMap, Key, Range>::value;

} // namespace traits
} // namespace pcp