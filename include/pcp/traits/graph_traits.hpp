#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class DirectedGraph, class = void>
struct is_directed_graph : std::false_type
{
};

/**
 * DirectedGraph concept:
 *
 * Notation used:
 * - vbegin, vend are iterators of type Graph::vertex_iterator_type
 * - ebegin, eend are iterators of type Graph::edge_iterator_type
 * - g is an instance of Graph
 * - u, v are vertices of type Graph::vertex_type
 *
 * Requirements:
 * - return type of g.vertices() is Graph::vertex_iterator_range
 * - return type of g.edges() is Graph::edge_iterator_range
 * - return type of g.out_edges_of(v) is Graph::edge_iterator_range
 *
 * Valid expressions:
 * - auto [vbegin, vend] = g.vertices();
 * - auto [ebegin, eend] = g.edges();
 * - auto [u, v] = *eit;
 */
template <class DirectedGraph>
struct is_directed_graph<
    DirectedGraph,
    std::void_t<
        typename DirectedGraph::vertex_type,
        typename DirectedGraph::vertex_iterator_type,
        typename DirectedGraph::edge_iterator_type,
        typename DirectedGraph::vertex_iterator_range,
        typename DirectedGraph::edge_iterator_range,
        decltype(std::declval<DirectedGraph&>().vertices()),
        decltype(std::declval<DirectedGraph&>().edges()),
        decltype(std::declval<DirectedGraph&>().out_edges_of(
            std::declval<typename DirectedGraph::vertex_iterator_type&>()))>> : std::true_type
{
};

template <class DirectedGraph>
static constexpr bool is_directed_graph_v = is_directed_graph<DirectedGraph>::value;

} // namespace traits
} // namespace pcp