#ifndef PCP_TRAITS_GRAPH_VERTEX_TRAITS_HPP
#define PCP_TRAITS_GRAPH_VERTEX_TRAITS_HPP

#include <type_traits>

namespace pcp {
namespace traits {

template <class GraphVertex, class = void>
struct is_graph_vertex : std::false_type
{
};

/**
 * @brief
 * GraphVertex requirements:
 * - type member named id_type which must be an integral type
 * - method id_type GraphVertex::id()
 * - method void GraphVertex::id(id_type)
 * @tparam GraphVertex Type to inspect
 */
template <class GraphVertex>
struct is_graph_vertex<
    GraphVertex,
    std::void_t<
        typename GraphVertex::id_type,
        decltype(std::declval<GraphVertex&>().id()),
        decltype(std::declval<GraphVertex&>().id(std::declval<typename GraphVertex::id_type&>()))>>
    : std::true_type
{
    static_assert(
        std::is_same_v<typename GraphVertex::id_type, decltype(std::declval<GraphVertex&>().id())>,
        "id_type type member must be return type of id() method");
    static_assert(
        std::is_integral_v<typename GraphVertex::id_type>,
        "id_type type member must be integral type");
};

template <class GraphVertex>
static constexpr bool is_graph_vertex_v = is_graph_vertex<GraphVertex>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_GRAPH_VERTEX_TRAITS_HPP