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
 * - vbegin, vend are iterators of type DirectedGraph::vertex_iterator_type
 * - ebegin, eend are iterators of type DirectedGraph::edge_iterator_type
 * - g is an instance of DirectedGraph
 * - uit, vit are iterators of type DirectedGraph::vertex_iterator_type
 * - u, v are vertices of type DirectedGraph::vertex_type
 *
 * Requirements:
 * - return type of g.vertices() is DirectedGraph::vertex_iterator_range
 * - return type of g.edges() is DirectedGraph::edge_iterator_range
 * - return type of g.out_edges_of(v) is DirectedGraph::edge_iterator_range
 *
 * Valid expressions:
 * - auto [vbegin, vend] = g.vertices();
 * - auto [ebegin, eend] = g.edges();
 * - auto [uit, vit] = *eit;
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
        decltype(std::declval<DirectedGraph&>().vertex_count()),
        decltype(std::declval<DirectedGraph&>().edge_count()),
        decltype(std::declval<DirectedGraph&>().vertices()),
        decltype(std::declval<DirectedGraph&>().edges()),
        decltype(std::declval<DirectedGraph&>().out_edges_of(
            std::declval<typename DirectedGraph::vertex_iterator_type&>()))>> : std::true_type
{
};

template <class DirectedGraph>
static constexpr bool is_directed_graph_v = is_directed_graph<DirectedGraph>::value;

template <class MutableDirectedGraph, class = void>
struct is_mutable_directed_graph : std::false_type
{
};

/**
 * MutableDirectedGraph concept (refines DirectedGraph):
 *
 * Notation used:
 * - vbegin, vend are iterators of type MutableDirectedGraph::vertex_iterator_type
 * - ebegin, eend are iterators of type MutableDirectedGraph::edge_iterator_type
 * - g is an instance of MutableDirectedGraph
 * - uit, vit are iterators of type MutableDirectedGraph::vertex_iterator_type
 * - u, v are vertices of type MutableDirectedGraph::vertex_type
 *
 * Valid expressions:
 * - g.add_vertex(v);
 * - g.add_edge(uit, vit);
 * - g.remove_vertex(uit);
 * - g.remove_edge(uit, vit);
 */
template <class MutableDirectedGraph>
struct is_mutable_directed_graph<
    MutableDirectedGraph,
    std::void_t<
        decltype(std::declval<MutableDirectedGraph&>().add_vertex(
            std::declval<typename MutableDirectedGraph::vertex_type&>())),
        decltype(std::declval<MutableDirectedGraph&>().add_edge(
            std::declval<typename MutableDirectedGraph::vertex_iterator_type&>(),
            std::declval<typename MutableDirectedGraph::vertex_iterator_type&>())),
        decltype(std::declval<MutableDirectedGraph&>().remove_vertex(
            std::declval<typename MutableDirectedGraph::vertex_iterator_type&>())),
        decltype(std::declval<MutableDirectedGraph&>().remove_edge(
            std::declval<typename MutableDirectedGraph::edge_iterator_type&>()))>>
    : is_directed_graph<MutableDirectedGraph>
{
    static_assert(
        std::is_same_v<
            std::remove_cv_t<decltype(std::declval<MutableDirectedGraph&>().add_vertex(
                std::declval<typename MutableDirectedGraph::vertex_type&>()))>,
            typename MutableDirectedGraph::vertex_iterator_type>,
        "MutableDirectedGraph's add_vertex() member function must return iterator to the added "
        "vertex");
    static_assert(
        std::is_same_v<
            std::remove_cv_t<decltype(std::declval<MutableDirectedGraph&>().add_edge(
                std::declval<typename MutableDirectedGraph::vertex_iterator_type&>(),
                std::declval<typename MutableDirectedGraph::vertex_iterator_type&>()))>,
            typename MutableDirectedGraph::edge_iterator_type>,
        "MutableDirectedGraph's add_edge() member function must return iterator to the added "
        "edge");
};

template <class MutableDirectedGraph>
static constexpr bool is_mutable_directed_graph_v =
    is_mutable_directed_graph<MutableDirectedGraph>::value;

} // namespace traits
} // namespace pcp