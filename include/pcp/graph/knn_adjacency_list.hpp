#ifndef PCP_GRAPH_UNDIRECTED_KNN_ADJACENCY_LIST_HPP
#define PCP_GRAPH_UNDIRECTED_KNN_ADJACENCY_LIST_HPP

/**
 * @file
 * @ingroup graph
 */

#include "directed_adjacency_list.hpp"
#include "pcp/traits/knn_search_traits.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

/**
 * @ingroup graph-structures-types
 * @brief
 * Constructs a graph of k-nearest-neighbors of the given vertices.
 * The k nearest neighbor querying implementation is user-provided.
 * The function assumes that the given vertices have been assigned
 * unique identifiers from [0...N-1] where N is the number of given
 * vertices (in other words N == std::distance(begin, end) is true).
 * The returned graph is undirected in the sense that we add
 * two edges (for both directions) for each k neighbor of each vertex.
 * This means that for a given vertex v having one of its k neighbors u,
 * the graph will contain edges (v, u) and (u, v).
 *
 * The sequence [begin, end] must be sorted by its identifiers ranging from [0, N-1].
 *
 * @tparam ForwardIter Iterator to a type convertible to GraphVertex
 * @tparam KnnSearcher Callable satisfying KnnSearcher concept
 * @tparam IndexMap Type satisfying IndexMap concept
 * @tparam GraphVertex Vertex type
 * @param begin Iterator to the start of the sequence
 * @param end Iterator to one past the end of the sequence
 * @param knn Callable object implementing the knn searches for each vertex
 * @param index_map The index map property map
 * @return The undirected graph of k nearest neighbors of each of its vertices
 */
template <
    class ForwardIter,
    class KnnSearcher,
    class IndexMap,
    class GraphVertex = typename std::iterator_traits<ForwardIter>::value_type>
auto undirected_knn_graph(
    ForwardIter begin,
    ForwardIter end,
    KnnSearcher knn,
    IndexMap const& index_map)
    -> directed_adjacency_list_t<typename std::iterator_traits<ForwardIter>::value_type, IndexMap>
{
    using vertex_type = GraphVertex;

    /**
     * Note:
     * Should we have a template parameter for the user to decide
     * on the type of graph he wants to use?
     * directed_adjacency_list_t can still be used as the default.
     */
    using graph_type = directed_adjacency_list_t<GraphVertex, IndexMap>;

    static_assert(
        std::is_convertible_v<typename std::iterator_traits<ForwardIter>::value_type, vertex_type>,
        "ForwardIter must be iterator to a type convertible to GraphVertex");
    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, GraphVertex>,
        "knn must satisfy KnnSearcher concept");

    // adds the vertices to the graph in sequential order
    // such that the vertices will be sorted by their id
    graph_type g(begin, end, index_map);
    auto const [vbegin, vend] = g.vertices();
    using difference_type     = decltype(std::distance(vbegin, vend));
    for (auto it = vbegin; it != vend; ++it)
    {
        auto const& neighbors = knn(*it);
        for (auto const& neighbor : neighbors)
        {
            auto const offset      = static_cast<difference_type>(index_map(neighbor));
            auto const neighbor_it = std::next(vbegin, offset);
            // add edges in both directions so that the graph
            // is undirected
            g.add_edge(it, neighbor_it);
            g.add_edge(neighbor_it, it);
        }
    }

    return g;
}

/**
 * @ingroup graph-structures-types
 * @brief
 * Constructs a graph of k-nearest-neighbors of the given vertices.
 * The k nearest neighbor querying implementation is user-provided.
 * The function assumes that the given vertices have been assigned
 * unique identifiers from [0...N-1] where N is the number of given
 * vertices (in other words N == std::distance(begin, end) is true).
 * The sequence [begin, end] must be sorted.
 *
 * @tparam ForwardIter Iterator to a type convertible to GraphVertex
 * @tparam KnnSearcher Callable satisfying KnnSearcher concept
 * @tparam GraphVertex Vertex type
 * @param begin Iterator to the start of the sequence
 * @param end Iterator to one past the end of the sequence
 * @param knn Callable object implementing the knn searches for each vertex
 * @param index_map The index map property map
 * @return The directed graph of k nearest neighbors of each of its vertices
 */
template <
    class ForwardIter,
    class KnnSearcher,
    class IndexMap,
    class GraphVertex = typename std::iterator_traits<ForwardIter>::value_type>
auto directed_knn_graph(
    ForwardIter begin,
    ForwardIter end,
    KnnSearcher knn,
    IndexMap const& index_map)
    -> directed_adjacency_list_t<typename std::iterator_traits<ForwardIter>::value_type, IndexMap>
{
    using vertex_type = GraphVertex;

    /**
     * Note:
     * Should we have a template parameter for the user to decide
     * on the type of graph he wants to use?
     * directed_adjacency_list_t can still be used as the default.
     */
    using graph_type = directed_adjacency_list_t<GraphVertex, IndexMap>;

    static_assert(
        std::is_convertible_v<typename std::iterator_traits<ForwardIter>::value_type, vertex_type>,
        "ForwardIter must be iterator to a type convertible to GraphVertex");
    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, GraphVertex>,
        "knn must satisfy KnnSearcher concept");

    // adds the vertices to the graph in sequential order
    // such that the vertices will be sorted by their id
    graph_type g{begin, end, index_map};
    auto const [vbegin, vend] = g.vertices();
    using difference_type     = decltype(std::distance(vbegin, vend));
    for (auto it = vbegin; it != vend; ++it)
    {
        auto const& neighbors = knn(*it);
        for (auto const& neighbor : neighbors)
        {
            auto const offset      = static_cast<difference_type>(index_map(neighbor));
            auto const neighbor_it = std::next(vbegin, offset);
            g.add_edge(it, neighbor_it);
        }
    }

    return g;
}

} // namespace graph
} // namespace pcp

#endif // PCP_GRAPH_UNDIRECTED_KNN_ADJACENCY_LIST_HPP