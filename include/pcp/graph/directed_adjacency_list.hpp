#ifndef PCP_GRAPH_DIRECTED_ADJACENCY_LIST_HPP
#define PCP_GRAPH_DIRECTED_ADJACENCY_LIST_HPP

/**
 * @file
 * @ingroup graph
 */

#include "pcp/traits/graph_vertex_traits.hpp"

#include <iterator>
#include <numeric>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

template <class GraphVertex>
class adjacency_list_edge_iterator_t;

/**
 * @ingroup graph
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
 * @tparam GraphVertex Any type can be a vertex
 */
template <class GraphVertex>
class directed_adjacency_list_t
{
    static_assert(
        traits::is_graph_vertex_v<GraphVertex>,
        "GraphVertex must satisfy GraphVertex concept");

  public:
    using self_type                  = directed_adjacency_list_t<GraphVertex>;
    using vertex_type                = GraphVertex;
    using vertices_type              = std::vector<vertex_type>;
    using vertex_iterator_type       = typename vertices_type::iterator;
    using const_vertex_iterator_type = typename vertices_type::const_iterator;
    using vertex_iterator_range      = std::pair<vertex_iterator_type, vertex_iterator_type>;
    using const_vertex_iterator_range =
        std::pair<const_vertex_iterator_type, const_vertex_iterator_type>;
    using edge_iterator_type  = adjacency_list_edge_iterator_t<GraphVertex>;
    using edge_iterator_range = std::pair<edge_iterator_type, edge_iterator_type>;

    using difference_type   = typename vertices_type::difference_type;
    using size_type         = typename vertices_type::size_type;
    using connectivity_type = std::vector<std::vector<size_type>>;

    friend class adjacency_list_edge_iterator_t<GraphVertex>;

    directed_adjacency_list_t()                 = default;
    directed_adjacency_list_t(self_type const&) = default;
    directed_adjacency_list_t(self_type&&)      = default;
    self_type& operator=(self_type const&) = default;
    self_type& operator=(self_type&&) = default;

    /**
     * @brief Constructs adjacency list using a range of elements convertible to Vertex
     * @tparam ForwardIter Iterator type of the range
     * @param begin
     * @param end
     */
    template <class ForwardIter>
    directed_adjacency_list_t(ForwardIter begin, ForwardIter end)
    {
        static_assert(
            std::is_convertible_v<
                typename std::iterator_traits<ForwardIter>::value_type,
                vertex_type>,
            "begin, end must be iterators to vertex_type");

        auto const vertex_count = static_cast<size_type>(std::distance(begin, end));
        vertices_.reserve(vertex_count);
        connectivity_.reserve(vertex_count);
        for (auto it = begin; it != end; ++it)
            add_vertex(*it);
    }

    /**
     * @brief Get number of vertices
     * @return
     */
    size_type vertex_count() const { return vertices_.size(); }

    /**
     * @brief Check if any vertices are in the graph
     * @return
     */
    bool empty() const { return vertices_.empty(); }

    /**
     * @brief Get the number of edges in the graph.
     *
     * Complexity is linear in the number of vertices.
     *
     * @return Number of edges
     */
    size_type edge_count() const
    {
        return std::accumulate(
            std::cbegin(connectivity_),
            std::cend(connectivity_),
            size_type{},
            [](size_type const count, std::vector<size_type> const& edges) {
                return count + edges.size();
            });
    }

    /**
     * @brief Returns a range over all vertices of this graph.
     * @return The range 'auto range = [first, last]' of the vertices
     */
    const_vertex_iterator_range vertices() const
    {
        return {std::cbegin(vertices_), std::cend(vertices_)};
    }
    /**
     * @brief Returns a range over all vertices of this graph.
     * @return The range 'auto range = [first, last]' of the vertices
     */
    vertex_iterator_range vertices() { return {std::begin(vertices_), std::end(vertices_)}; }
    /**
     * @brief Returns a range over all edges of this graph.
     * @return The range 'auto range = [first, last]' of the edges
     */
    edge_iterator_range edges() const
    {
        return {
            edge_iterator_type{const_cast<self_type*>(this)},
            edge_iterator_type{const_cast<self_type*>(this), connectivity_.size()}};
    }
    /**
     * @brief Get all outgoing edges of the vertex referenced by vit.
     * Complexity is O(1).
     * @param vit Iterator to the vertex from which we want to get the outgoing edges
     * @return A range over the outgoing edges of vit
     */
    edge_iterator_range out_edges_of(const_vertex_iterator_type vit) const
    {
        auto const begin   = std::cbegin(vertices_);
        auto const voffset = std::distance<const_vertex_iterator_type>(begin, vit);
        auto const vidx    = static_cast<size_type>(voffset);
        if (connectivity_[vidx].empty())
        {
            auto const end = edge_iterator_type{const_cast<self_type*>(this), connectivity_.size()};
            return {end, end};
        }
        auto edge_begin = edge_iterator_type{const_cast<self_type*>(this), vidx};
        auto edge_end   = edge_iterator_type{const_cast<self_type*>(this), vidx + 1u};
        return {edge_begin, edge_end};
    }

    /**
     * @brief Add a vertex to the graph.
     * @param v
     * @return Iterator to the newly added vertex
     */
    vertex_iterator_type add_vertex(vertex_type const& v)
    {
        auto const idx    = vertices_.size();
        auto const offset = static_cast<difference_type>(idx);
        vertices_.push_back(v);
        connectivity_.push_back({});
        return std::begin(vertices_) + offset;
    }

    /**
     * @brief Remove a vertex from the graph.
     *
     * Complexity is O(E + V)
     *
     * @param vit Iterator to the vertex to remove
     * @return Iterator to the next vertex in the range of vertices [first, last] before removing
     * vit
     */
    vertex_iterator_type remove_vertex(vertex_iterator_type const& vit)
    {
        difference_type const offset = std::distance(std::begin(vertices_), vit);
        size_type const idx          = static_cast<size_type>(offset);
        auto next                    = vertices_.erase(vit);
        connectivity_.erase(std::begin(connectivity_) + offset);
        for (auto& neighbors : connectivity_)
        {
            if (neighbors.empty())
                continue;

            auto const it = std::remove(std::begin(neighbors), std::end(neighbors), idx);
            neighbors.erase(it, std::end(neighbors));
            std::transform(
                std::begin(neighbors),
                std::end(neighbors),
                std::begin(neighbors),
                [idx](auto const neighbor_index) {
                    bool const should_be_shifted = neighbor_index > idx;
                    return should_be_shifted ? neighbor_index - 1 : neighbor_index;
                });
        }
        return next;
    }

    /**
     * @brief Add edge (uit, vit) to the graph
     * @param uit Source vertex
     * @param vit Destination vertex
     * @return Iterator to the created edge
     */
    edge_iterator_type add_edge(vertex_iterator_type const& uit, vertex_iterator_type const& vit)
    {
        auto const begin   = std::begin(vertices_);
        auto const uoffset = std::distance(begin, uit);
        auto const voffset = std::distance(begin, vit);
        auto const uidx    = static_cast<size_type>(uoffset);
        auto const vidx    = static_cast<size_type>(voffset);
        auto const i       = uidx;
        auto const j       = connectivity_[uidx].size();
        connectivity_[uidx].push_back(vidx);
        return edge_iterator_type{this, i, j};
    }

    /**
     * @brief Removes an edge from the graph
     * @param eit Iterator to the edge to remove
     * @return Iterator to the next edge in the sequence [first, last]
     */
    edge_iterator_type remove_edge(edge_iterator_type const& eit)
    {
        auto next     = eit;
        auto const i  = eit.i();
        auto const j  = eit.j();
        auto const it = std::begin(connectivity_[i]) + j;
        connectivity_[i].erase(it);
        if (j >= connectivity_[i].size())
            ++next;
        return next;
    }

    /**
     * @brief Clear all vertices and edges from the graph
     */
    void clear()
    {
        vertices_.clear();
        connectivity_.clear();
    }

  private:
    vertices_type vertices_;
    connectivity_type connectivity_;
};

/**
 * @ingroup graph
 * @brief
 * Iterator to the edges of a directed_adjacency_list_t.
 * The edges returned by dereferencing this iterator aren't
 * the actual edges as stored in the graph's implementation.
 * The edges are considered to be pairs of the graph's
 * vertex iterators.
 * @tparam GraphVertex Vertex type satisfying GraphVertex concept
 */
template <class GraphVertex>
class adjacency_list_edge_iterator_t
{
  public:
    using self_type            = adjacency_list_edge_iterator_t<GraphVertex>;
    using graph_type           = directed_adjacency_list_t<GraphVertex>;
    using vertices_type        = typename graph_type::vertices_type;
    using vertex_iterator_type = typename graph_type::vertex_iterator_type;
    using connectivity_type    = typename graph_type::connectivity_type;

    using value_type        = typename std::pair<vertex_iterator_type, vertex_iterator_type>;
    using reference         = value_type;
    using const_reference   = value_type const;
    using pointer           = value_type*;
    using const_pointer     = value_type const*;
    using iterator_category = std::random_access_iterator_tag;
    using difference_type   = typename connectivity_type::difference_type;
    using size_type         = typename connectivity_type::size_type;

    adjacency_list_edge_iterator_t(graph_type* graph) : graph_(graph), i_(), j_()
    {
        try_next_valid_edge();
    }
    adjacency_list_edge_iterator_t(graph_type* graph, size_type i) : graph_(graph), i_(i), j_()
    {
        try_next_valid_edge();
    }
    adjacency_list_edge_iterator_t(graph_type* graph, size_type i, size_type j)
        : graph_(graph), i_(i), j_(j)
    {
        try_next_valid_edge();
    }

    adjacency_list_edge_iterator_t(self_type const&) = default;
    self_type& operator=(self_type const&) = default;

    reference operator*() const
    {
        auto const ioffset         = static_cast<difference_type>(i_);
        auto u                     = std::begin(graph_->vertices_) + ioffset;
        auto const neighbor_offset = static_cast<difference_type>(graph_->connectivity_[i_][j_]);
        auto v                     = std::begin(graph_->vertices_) + neighbor_offset;
        return {u, v};
    }

    self_type& operator++()
    {
        if (++j_ >= graph_->connectivity_[i_].size())
        {
            do
            {
                j_ = 0u;
                ++i_;
            } while (i_ < graph_->connectivity_.size() && graph_->connectivity_[i_].empty());
        }
        return *this;
    }

    self_type operator++(int)
    {
        self_type copy{*this};
        this->operator++();
        return copy;
    }

    self_type& operator--()
    {
        if (j_ == 0)
        {
            do
            {
                --i_;
                j_ = graph_->connectivity_[i_].size();
            } while (i_ >= 0u && graph_->connectivity_[i_].empty());
        }
        --j_;
        return *this;
    }

    /**
     * Preconditions: n must be >= 0
     */
    self_type& operator+=(difference_type n)
    {
        *this = (*this + n);
        return *this;
    }

    /**
     * Preconditions: n must be >= 0
     */
    self_type& operator-=(difference_type n)
    {
        *this = (*this - n);
        return *this;
    }

    self_type operator+(difference_type n) const
    {
        auto i                  = i_;
        auto j                  = j_;
        auto const vertex_count = graph_->vertex_count();
        while (n > 0 && i < vertex_count)
        {
            auto const remaining_increments =
                static_cast<difference_type>(graph_->connectivity_[i].size() - j);
            if (remaining_increments > n)
            {
                j += static_cast<size_type>(n);
                break;
            }
            n -= remaining_increments;
            do
            {
                ++i;
                j = 0u;
            } while (i < graph_->connectivity_.size() && graph_->connectivity_[i].empty());
        }
        return self_type{graph_, i, j};
    }

    difference_type operator-(self_type const& other) const
    {
        auto const reduce_op = [](auto const s, auto const& edges) {
            return s + static_cast<difference_type>(edges.size());
        };
        auto const iself  = static_cast<difference_type>(i());
        auto const jself  = static_cast<difference_type>(j());
        auto const iother = static_cast<difference_type>(other.i());
        auto const jother = static_cast<difference_type>(other.j());
        auto const begin  = std::cbegin(graph_->connectivity_);
        auto const end1   = begin + iself;
        auto const sum1   = std::accumulate(begin, end1, difference_type{}, reduce_op) + jself;
        auto const end2   = begin + iother;
        auto const sum2   = std::accumulate(begin, end2, difference_type{}, reduce_op) + jother;
        return sum1 - sum2;
    }
    self_type operator-(difference_type n) const
    {
        auto i = i_;
        auto j = j_;
        while (n > 0 && i >= 0u)
        {
            auto const remaining_decrements = j + 1u;
            if (remaining_decrements > n)
            {
                j -= n;
                break;
            }
            n -= remaining_decrements;
            do
            {
                --i;
                j = graph_->connectivity_[i].size() - 1;
            } while (i >= 0u && graph_->connectivity_[i].empty());
        }
        return self_type{graph_, i, j};
    }
    friend self_type operator+(difference_type n, self_type const& self) { return self + n; }

    reference operator[](difference_type n) const { return *(*this + n); }

    bool operator==(self_type const& other) const
    {
        return (graph_ == other.graph_) && (i_ == other.i_) && (j_ == other.j_);
    }

    bool operator!=(self_type const& other) const { return !(*this == other); }
    bool operator<(self_type const& other) const
    {
        return i_ < other.i_ || (i_ == other.i_ && j_ < other.j_);
    }
    bool operator<=(self_type const& other) const { return (*this < other) || (*this == other); }
    bool operator>(self_type const& other) const { return !(*this <= other); }
    bool operator>=(self_type const& other) const { return !(*this < other); }

    size_type i() const { return i_; }
    size_type j() const { return j_; }

  private:
    void try_next_valid_edge()
    {
        if (i_ >= graph_->connectivity_.size())
            return;

        if (j_ >= graph_->connectivity_[i_].size())
        {
            ++(*this);
            return;
        }
        if (graph_->connectivity_[i_].empty())
        {
            ++(*this);
            return;
        }
    }

    graph_type* graph_;
    size_type i_;
    size_type j_;
};

} // namespace graph
} // namespace pcp

#endif // PCP_GRAPH_DIRECTED_ADJACENCY_LIST_HPP