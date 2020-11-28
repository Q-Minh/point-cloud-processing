#pragma once

#include "pcp/traits/graph_traits.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>

namespace pcp {
namespace graph {

/**
 * @brief Implementation of prim's minimum spanning tree algorithm.
 *
 * The implementation uses std::next(begin, vertex_iterator)
 * three times in its main loop over vertices of the graph G.
 * std::distance(begin, vertex_iterator) is also
 * used many times in the main loop.
 *
 * If the vertex_iterator type is random access, the complexity
 * is O(|E+V|logV). Otherwise, expect O(|V|^2).
 *
 * @tparam DirectedGraph Type satisfying DirectedGraph concept
 * @tparam CostFunc Callable type with signature R(typename DirectedGraph::vertex_iterator_type,
 * typename DirectedGraph::vertex_iterator_type)
 * @tparam MutableDirectedGraph Type satisfying MutableDirectedGraph concept
 * @param G        The directed graph from which we compute the minimum spanning tree
 * @param get_cost The cost function used to add weight to edges between connected vertices
 * @param root     The starting point/vertex of the algorithm. This will be the root of the minimum
 * spanning tree returned
 * @return         The minimum spanning tree of graph G with root root using edge costs determined
 * by get_cost. Return type is a pair<graph, root>
 */
template <class DirectedGraph, class MutableDirectedGraph, class CostFunc>
auto prim_minimum_spanning_tree(
    DirectedGraph const& G,
    CostFunc get_cost,
    typename DirectedGraph::vertex_iterator_type root)
    -> std::pair<
        MutableDirectedGraph,
        std::function<typename MutableDirectedGraph::vertex_iterator_type(MutableDirectedGraph&)>>
{
    using const_vertex_iterator_type = typename DirectedGraph::const_vertex_iterator_type;
    using edge_iterator_type         = typename DirectedGraph::edge_iterator_type;
    using vertex_type                = typename DirectedGraph::vertex_type;
    using size_type                  = typename DirectedGraph::size_type;
    using cost_type                  = std::invoke_result_t<CostFunc, vertex_type, vertex_type>;

    static_assert(
        traits::is_directed_graph_v<DirectedGraph>,
        "G must satisfy DirectedGraph concept");
    static_assert(
        traits::is_mutable_directed_graph_v<MutableDirectedGraph>,
        "Return type MutableDirectedGraph must satisfy MutableDirectedGraph concept");
    static_assert(
        std::is_convertible_v<vertex_type, typename MutableDirectedGraph::vertex_type>,
        "MutableDirectedGraph::vertex_type must be convertible to DirectedGraph::vertex_type");
    static_assert(
        std::is_invocable_v<CostFunc, vertex_type, vertex_type>,
        "get_cost must satisfy signature R(typename DirectedGraph::vertex_type, "
        "DirectedGraph::vertex_type)");

    auto const [vbegin, vend] = G.vertices();
    auto const vertex_count   = G.vertex_count();

    auto const key_of = [vbegin = vbegin](const_vertex_iterator_type it) -> size_t {
        return static_cast<size_t>(std::distance<const_vertex_iterator_type>(vbegin, it));
    };

    MutableDirectedGraph MST;
    std::for_each(vbegin, vend, [&MST](vertex_type const& v) { MST.add_vertex(v); });

    /**
     * We use indices to refer to offsets of elements in the range
     * [vbegin, vend), so indices[0] is the offset used to recover
     * the vertex iterator vit = std::next(vbegin, indices[0]),
     * which in this case will return an iterator to the first
     * vertex because of the std::iota call.
     */
    std::vector<size_t> indices(vertex_count);
    std::iota(indices.begin(), indices.end(), size_t{0u});

    /**
     * Bookkeep the best costs found yet in the algorithm using
     * a vector of costs, where cost[key_of(vertex_iterator)]
     * represents the minimum cost currently known from edges
     * going to vertex_iterator
     */
    std::vector<cost_type> cost(vertex_count, std::numeric_limits<cost_type>::max());

    /**
     * The root vertex specified will start off with a zero-initialized cost,
     * so that it is the first vertex to be popped off the heap
     */
    cost[key_of(root)] = cost_type{};

    /**
     * Bookkeep the minimum cost edges found yet in the algorithm
     * using a vector of optional edge_iterators so that it is
     * possible to have null edges (optional will have no value
     * in that case). edge[key_of(vertex_iterator)] returns the
     * current minimum cost edge of edges going to vertex_iterator.
     */
    std::vector<std::optional<edge_iterator_type>> edge(vertex_count);

    /**
     * Use a boolean array to keep track of vertices that are
     * still not added to the Minimum Spanning Tree.
     * if contains[key_of(vertex_iterator)] is true, then the
     * vertex referenced by vertex_iterator is still in the
     * graph G, which means that it is not yet added to the
     * Minimum Spanning Tree.
     */
    std::vector<bool> contains(vertex_count, true);

    auto const compare = [&cost](size_type const id1, size_type const id2) {
        return cost[id1] > cost[id2];
    };

    using difference_type = typename decltype(cost)::difference_type;
    using compare_type    = decltype(compare);
    using min_heap_type   = std::priority_queue<size_type, decltype(indices), compare_type>;

    auto const [mst_vbegin, mst_vend] = MST.vertices();
    min_heap_type heap{indices.cbegin(), indices.cend(), compare};
    while (!heap.empty())
    {
        auto const vid     = heap.top();
        auto const voffset = static_cast<difference_type>(vid);

        // remove vertex from the graph G
        contains[vid] = false;

        if (edge[vid].has_value())
        {
            auto const [e1, e2] = *(edge[vid].value());
            auto const id1      = key_of(e1);
            auto const id2      = key_of(e2);
            auto const vit1     = std::next(mst_vbegin, static_cast<difference_type>(id1));
            auto const vit2     = std::next(mst_vbegin, static_cast<difference_type>(id2));
            MST.add_edge(vit1, vit2);
        }

        auto const vit                              = std::next(vbegin, voffset);
        auto const [out_edges_begin, out_edges_end] = G.out_edges_of(vit);
        for (edge_iterator_type it = out_edges_begin; it != out_edges_end; ++it)
        {
            auto const [e1, e2] = *it;
            auto const wid      = key_of(e2);
            auto const weight   = get_cost(*e1, *e2);
            if (contains[wid] && weight < cost[wid])
            {
                cost[wid] = weight;
                edge[wid] = it;
            }
        }
        // remove heap element at the end so that reordering
        // of the heap is coherent with the latest changes
        // from the cost array
        heap.pop();
    }

    auto const key_of_root = key_of(root);
    return {
        MST,
        [key_of_root](MutableDirectedGraph& tree) ->
        typename MutableDirectedGraph::vertex_iterator_type {
            auto [tree_begin, tree_end] = tree.vertices();
            return std::next(tree_begin, static_cast<difference_type>(key_of_root));
        }};
}

} // namespace graph
} // namespace pcp