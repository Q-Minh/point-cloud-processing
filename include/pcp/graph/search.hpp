#ifndef PCP_GRAPH_SEARCH_HPP
#define PCP_GRAPH_SEARCH_HPP

/**
 * @file
 * @ingroup graph
 */

#include "pcp/traits/graph_traits.hpp"
#include "pcp/traits/graph_vertex_traits.hpp"

#include <queue>
#include <stack>
#include <type_traits>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

/**
 * @ingroup graph-algorithms
 * @brief
 * Visit a graph in breadth first order, visiting each vertex along
 * with its source vertex with a call to op(source_vertex, destination_vertex).
 * Starts from vertex begin.
 * @tparam DirectedGraph Type of graph to traverse
 * @tparam IndexMap Type satisfying IndexMap concept
 * @tparam BinaryOp Type of callable to call on source and destination vertices
 * @tparam GraphIterator Type of iterator used to traverse the graph
 * @param graph The graph to traverse
 * @param begin Iterator to the root vertex of the traversal
 * @param index_map The index map property map
 * @param op The binary operation to apply once at each vertex traversed. Takes two parameters of
 * the type of vertex stored in the graph.
 */
template <
    class DirectedGraph,
    class IndexMap,
    class BinaryOp,
    class GraphIterator = typename DirectedGraph::vertex_iterator_type>
void breadth_first_search(
    DirectedGraph& graph,
    GraphIterator begin,
    IndexMap const& index_map,
    BinaryOp&& op)
{
    using vertex_iterator_type = GraphIterator;
    using vertex_type          = typename std::iterator_traits<vertex_iterator_type>::value_type;
    using size_type            = typename DirectedGraph::size_type;

    static_assert(
        traits::is_directed_graph_v<DirectedGraph>,
        "graph must satisfy DirectedGraph concept");
    static_assert(
        std::is_invocable_v<BinaryOp, vertex_type, vertex_type>,
        "op must be callable by op(GraphIterator, GraphIterator)");

    auto const [vbegin, vend] = graph.vertices();
    auto const n              = std::distance(vbegin, vend);
    auto const count          = static_cast<size_type>(n);
    std::vector<bool> visited(count, false);

    std::queue<vertex_iterator_type> bfs_queue;
    bfs_queue.push(begin);
    while (!bfs_queue.empty())
    {
        auto source = bfs_queue.front();
        bfs_queue.pop();
        auto [edge_begin, edge_end] = graph.out_edges_of(source);
        for (auto edge = edge_begin; edge != edge_end; ++edge)
        {
            auto [u, v] = *edge;

            auto const destination_id = index_map(*v);
            if (visited[destination_id])
                continue;

            op(*u, *v);
            visited[destination_id] = true;
            bfs_queue.push(v);
        }
        auto const source_id = index_map(*source);
        visited[source_id]   = true;
    }
}

/**
 * @ingroup graph-algorithms
 * @brief
 * Visit a graph in depth first order, visiting each vertex along
 * with its source vertex with a call to op(source_vertex, destination_vertex).
 * Starts from vertex begin.
 * @tparam DirectedGraph Type of graph to traverse
 * @tparam IndexMap Type satisfying IndexMap concept
 * @tparam BinaryOp Type of callable to call on source and destination vertices
 * @tparam GraphIterator Type of iterator used to traverse the graph
 * @param graph The graph to traverse
 * @param begin Iterator to the root vertex of the traversal
 * @param index_map The index map property map
 * @param op The binary operation to apply at edge traversed vertex having signature 
 * f(vertex_type, vertex_type)
 */
template <
    class DirectedGraph,
    class IndexMap,
    class BinaryOp,
    class GraphIterator = typename DirectedGraph::vertex_iterator_type>
void depth_first_search(DirectedGraph& graph, GraphIterator begin, IndexMap index_map, BinaryOp&& op)
{
    using vertex_iterator_type = GraphIterator;
    using vertex_type          = typename std::iterator_traits<vertex_iterator_type>::value_type;
    using size_type            = typename DirectedGraph::size_type;

    static_assert(
        traits::is_graph_vertex_v<vertex_type>,
        "GraphIterator must be dereferenceable to a type satisfying GraphVertex concept");
    static_assert(
        traits::is_directed_graph_v<DirectedGraph>,
        "graph must satisfy DirectedGraph concept");
    static_assert(
        std::is_invocable_v<BinaryOp, vertex_type, vertex_type>,
        "op must be callable by op(GraphIterator, GraphIterator)");

    auto const [vbegin, vend] = graph.vertices();
    auto const n              = std::distance(vbegin, vend);
    auto const count          = static_cast<size_type>(n);
    std::vector<bool> visited(count, false);

    using dfs_element_type = std::pair<vertex_iterator_type, vertex_iterator_type>;
    std::stack<dfs_element_type> dfs_stack;
    dfs_stack.push({begin, begin});
    while (!dfs_stack.empty())
    {
        auto [parent, source] = dfs_stack.top();
        dfs_stack.pop();

        auto const source_id = index_map(*source);

        if (!visited[source_id])
        {
            if (parent != source)
                op(*parent, *source);

            visited[source_id] = true;
        }

        auto [edge_begin, edge_end] = graph.out_edges_of(source);
        for (auto edge = edge_begin; edge != edge_end; ++edge)
        {
            auto [u, v] = *edge;

            auto const destination_id = index_map(*v);
            if (visited[destination_id])
                continue;

            dfs_stack.push({u, v});
        }
    }
}

} // namespace graph
} // namespace pcp

#endif // PCP_GRAPH_SEARCH_HPP