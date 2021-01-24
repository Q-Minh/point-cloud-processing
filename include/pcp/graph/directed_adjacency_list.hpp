#ifndef PCP_GRAPH_DIRECTED_ADJACENCY_LIST_HPP
#define PCP_GRAPH_DIRECTED_ADJACENCY_LIST_HPP

/**
 * @file
 * @ingroup graph
 */

#include "pcp/traits/index_map.hpp"
#include "pcp/traits/property_map_traits.hpp"

#include <iterator>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

/**
 * @ingroup graph-structures-types
 * @brief Implementation of an adjacency list based graph.
 *
 * Uses a vector of vertices to stores the vertices and
 * uses a vector of vectors of indices of neighbors to
 * store connectivity information of the graph.
 *
 * Space complexity is O(V * sizeof(Vertex) + E * sizeof(uint64_t))
 *
 * Satisfies MutableDirectedGraph concept.
 *
 * @tparam Element Any type
 */
template <class Element, class IndexMap>
class directed_adjacency_list_t
{
  public:
    using self_type             = directed_adjacency_list_t<Element, IndexMap>;
    using vertex_type           = Element;
    using vertices_type         = std::vector<vertex_type>;
    using vertex_iterator_type  = vertex_type*;
    using vertex_iterator_range = std::pair<vertex_type*, vertex_type*>;
    using index_type = typename traits::property_map_traits<IndexMap, vertex_type>::value_type;

    struct hash_fn
    {
        hash_fn(IndexMap const& index) : index_(index) {}

        index_type operator()(vertex_type const* v) const { return index_(*v); }

        IndexMap index_;
    };

    struct key_equal_fn
    {
        key_equal_fn(IndexMap const& index) : index_(index) {}

        bool operator()(vertex_type const* v1, vertex_type const* v2) const
        {
            return index_(*v1) == index_(*v2);
        }

        IndexMap index_;
    };

    using edges_type = std::unordered_multimap<vertex_type*, vertex_type*, hash_fn, key_equal_fn>;
    using edge_type  = typename edges_type::value_type;
    using edge_iterator_type  = typename edges_type::iterator;
    using edge_iterator_range = std::pair<edge_iterator_type, edge_iterator_type>;

    using difference_type = typename vertices_type::difference_type;
    using size_type       = typename vertices_type::size_type;

    directed_adjacency_list_t(self_type const&) = default;
    directed_adjacency_list_t(self_type&&)      = default;
    self_type& operator=(self_type const&) = default;
    self_type& operator=(self_type&&) = default;

    directed_adjacency_list_t(IndexMap const& idmap, std::size_t initial_capacity = 4'096)
        : hash_{idmap},
          key_equal_{idmap}, edges_{initial_capacity, hash_, key_equal_}, vertices_{}
    {
        vertices_.reserve(initial_capacity);
    }

    /**
     * @brief Constructs adjacency list using a range of elements convertible to Vertex
     * @tparam ForwardIter Iterator type of the range
     * @param begin
     * @param end
     */
    template <class ForwardIter>
    directed_adjacency_list_t(ForwardIter begin, ForwardIter end, IndexMap const& idmap)
        : hash_{idmap},
          key_equal_{idmap},
          edges_{static_cast<size_type>(std::distance(begin, end)), hash_, key_equal_},
          vertices_{}
    {
        static_assert(
            std::is_convertible_v<
                typename std::iterator_traits<ForwardIter>::value_type,
                vertex_type>,
            "begin, end must be iterators to vertex_type");

        vertices_.assign(begin, end);
    }

    /**
     * @brief Get number of vertices
     * @return
     */
    size_type vertex_count() const { return vertices_.size(); }

    /**
     * @brief Check if any vertices are in the graph
     * @return True if graph has no vertices
     */
    bool empty() const { return vertices_.empty(); }

    /**
     * @brief Get the number of edges in the graph.
     *
     * Complexity is linear in the number of vertices.
     *
     * @return Number of edges
     */
    size_type edge_count() const { return edges_.size(); }

    /**
     * @brief Returns a range over all vertices of this graph.
     * @return Range of all vertices
     */
    vertex_iterator_range vertices()
    {
        auto begin = std::addressof(vertices_.front());
        auto end   = std::addressof(vertices_.back()) + 1;
        return std::make_pair(begin, end);
    }

    /**
     * @brief Returns a range over all edges of this graph.
     * @return The range 'auto range = [first, last]' of the edges
     */
    edge_iterator_range edges() { return std::make_pair(edges_.begin(), edges_.end()); }

    /**
     * @brief Get all outgoing edges of the vertex referenced by vit.
     * Complexity is O(1).
     * @param vit Iterator to the vertex from which we want to get the outgoing edges
     * @return A range over the outgoing edges of vit as a pair of iterators
     */
    edge_iterator_range out_edges_of(vertex_iterator_type vit) { return edges_.equal_range(vit); }

    /**
     * @brief Add a vertex to the graph.
     * @param v
     * @return Iterator to the newly added vertex
     */
    vertex_iterator_type add_vertex(vertex_type const& v)
    {
        auto const offset = vertices_.size();
        vertices_.push_back(v);
        return std::addressof(vertices_.front()) + offset;
    }

    /**
     * @brief Add edge (uit, vit) to the graph
     * @param uit Source vertex
     * @param vit Destination vertex
     * @return Iterator to the created edge
     */
    edge_iterator_type add_edge(vertex_iterator_type uit, vertex_iterator_type vit)
    {
        return edges_.insert(std::make_pair(uit, vit));
    }

    /**
     * @brief Removes an edge from the graph
     * @param eit Iterator to the edge to remove
     * @return Iterator to the next edge in the sequence [first, last]
     */
    edge_iterator_type remove_edge(edge_iterator_type eit) { return edges_.erase(eit); }

    /**
     * @brief Clear all vertices and edges from the graph
     */
    void clear()
    {
        vertices_.clear();
        edges_.clear();
    }

  private:
    hash_fn hash_;           ///< Functor for hashing vertex pointers
    key_equal_fn key_equal_; ///< Functor for comparing vertex keys
    edges_type edges_;       ///< hash multimap of neighbors of vertices
    vertices_type vertices_; ///< vector of the elements to be stored
};

} // namespace graph
} // namespace pcp

#endif // PCP_GRAPH_DIRECTED_ADJACENCY_LIST_HPP