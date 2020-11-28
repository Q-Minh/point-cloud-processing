#pragma once

#include "directed_adjacency_list.hpp"
#include "pcp/traits/knn_search_traits.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

/**
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
 * @tparam ForwardIter Iterator to a type convertible to GraphVertex
 * @tparam KnnSearcher Callable satisfying KnnSearcher concept
 * @tparam GraphVertex Vertex type satisfying GraphVertex concept
 * @param begin
 * @param end
 * @param knn Callable object implementing the knn searches for each vertex
 * @param k The number of neighbors to consider in the knn search for each vertex
 * @return The undirected graph of k nearest neighbors of each of its vertices
 */
template <
    class ForwardIter,
    class KnnSearcher,
    class GraphVertex = typename std::iterator_traits<ForwardIter>::value_type>
auto undirected_knn_graph(ForwardIter begin, ForwardIter end, KnnSearcher knn, std::uint64_t k = 5)
    -> directed_adjacency_list_t<typename std::iterator_traits<ForwardIter>::value_type>
{
    using vertex_type = GraphVertex;

    /**
     * Note:
     * Should we have a template parameter for the user to decide
     * on the type of graph he wants to use?
     * directed_adjacency_list_t can still be used as the default.
     */
    using graph_type = directed_adjacency_list_t<GraphVertex>;

    static_assert(
        traits::is_graph_vertex_v<GraphVertex>,
        "GraphVertex must satisfy GraphVertex concept");
    static_assert(
        std::is_convertible_v<typename std::iterator_traits<ForwardIter>::value_type, vertex_type>,
        "RandomAccessIter must be iterator to a type convertible to GraphVertex");
    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, GraphVertex, std::uint64_t>,
        "knn must satisfy KnnSearcher concept");

    std::vector<vertex_type> sorted_vertices(begin, end);
    // Note: Could use radix sort to sort in O(N)
    std::sort(sorted_vertices.begin(), sorted_vertices.end(), [](auto const& v1, auto const& v2) {
        return v1.id() < v2.id();
    });

    // adds the vertices to the graph in sequential order
    // such that the vertices will be sorted by their id
    graph_type g(sorted_vertices.cbegin(), sorted_vertices.cend());
    auto const [vbegin, vend] = g.vertices();
    for (auto it = vbegin; it != vend; ++it)
    {
        auto const& neighbors = knn(*it, k);
        for (auto const& neighbor : neighbors)
        {
            auto const neighbor_it = std::next(vbegin, neighbor.id());
            // add edges in both directions so that the graph
            // is undirected
            g.add_edge(it, neighbor_it);
            g.add_edge(neighbor_it, it);
        }
    }

    return g;
}

} // namespace graph
} // namespace pcp