#pragma once

#include <pcp/traits/graph_traits.hpp>
#include <pcp/traits/graph_vertex_traits.hpp>
#include <queue>
#include <stack>
#include <type_traits>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

/**
 * @brief
 * Visit a graph in breadth first order, visiting each vertex along
 * with its source vertex with a call to op(source_vertex, destination_vertex).
 * Starts from vertex begin.
 * @tparam DirectedGraph Type of graph to traverse
 * @tparam GraphIterator Type of iterator used to traverse the graph
 * @tparam BinaryOp Type of callable to call on source and destination vertices
 * @param graph 
 * @param op
 * @param begin
 */
template <
    class DirectedGraph,
    class BinaryOp,
    class GraphIterator = typename DirectedGraph::vertex_iterator_type>
void breadth_first_search(DirectedGraph& graph, GraphIterator begin, BinaryOp&& op)
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

            auto const destination_id = v->id();
            if (visited[destination_id])
                continue;

            op(*u, *v);
            visited[destination_id] = true;
            bfs_queue.push(v);
        }
        auto const source_id = source->id();
        visited[source_id]   = true;
    }
}

/**
 * @brief
 * Visit a graph in depth first order, visiting each vertex along
 * with its source vertex with a call to op(source_vertex, destination_vertex).
 * Starts from vertex begin.
 * @tparam DirectedGraph Type of graph to traverse
 * @tparam GraphIterator Type of iterator used to traverse the graph
 * @tparam BinaryOp Type of callable to call on source and destination vertices
 * @param graph
 * @param op
 * @param begin
 */
template <
    class DirectedGraph,
    class BinaryOp,
    class GraphIterator = typename DirectedGraph::vertex_iterator_type>
void depth_first_search(DirectedGraph& graph, GraphIterator begin, BinaryOp&& op)
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

        auto const source_id = source->id();

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

            auto const destination_id = v->id();
            if (visited[destination_id])
                continue;

            dfs_stack.push({u, v});
        }
    }
}

} // namespace graph
} // namespace pcp