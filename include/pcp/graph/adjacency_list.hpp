#pragma once

#include <iterator>
#include <numeric>
#include <utility>
#include <vector>

namespace pcp {
namespace graph {

template <class Vertex>
class adjacency_list_edge_iterator_t;

template <class Vertex>
class adjacency_list_t
{
  public:
    using self_type                  = adjacency_list_t<Vertex>;
    using vertex_type                = Vertex;
    using vertices_type              = std::vector<vertex_type>;
    using vertex_iterator_type       = typename vertices_type::iterator;
    using const_vertex_iterator_type = typename vertices_type::const_iterator;
    using vertex_iterator_range      = std::pair<vertex_iterator_type, vertex_iterator_type>;
    using const_vertex_iterator_range =
        std::pair<const_vertex_iterator_type, const_vertex_iterator_type>;
    using edge_iterator_type  = adjacency_list_edge_iterator_t<Vertex>;
    using edge_iterator_range = std::pair<edge_iterator_type, edge_iterator_type>;

    using difference_type   = typename vertices_type::difference_type;
    using size_type         = typename vertices_type::size_type;
    using connectivity_type = std::vector<std::vector<size_type>>;

    friend class adjacency_list_edge_iterator_t<Vertex>;

    adjacency_list_t()                 = default;
    adjacency_list_t(self_type const&) = default;
    adjacency_list_t(self_type&&)      = default;
    self_type& operator=(self_type const&) = default;
    self_type& operator=(self_type&&) = default;

    template <class ForwardIter>
    adjacency_list_t(ForwardIter begin, ForwardIter end)
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

    size_type vertex_count() const { return vertices_.size(); }
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

    const_vertex_iterator_range vertices() const
    {
        return {std::cbegin(vertices_), std::cend(vertices_)};
    }

    vertex_iterator_range vertices() { return {std::begin(vertices_), std::end(vertices_)}; }

    edge_iterator_range edges() const
    {
        return {
            edge_iterator_type{const_cast<self_type&>(*this)},
            edge_iterator_type{const_cast<self_type&>(*this), connectivity_.size()}};
    }

    edge_iterator_range out_edges_of(const_vertex_iterator_type vit) const
    {
        auto const begin   = std::begin(vertices_);
        auto const voffset = std::distance(begin, vit);
        auto const vidx    = static_cast<size_type>(voffset);
        auto edge_begin    = edge_iterator_type{const_cast<self_type&>(*this), vidx};
        auto edge_end      = edge_iterator_type{const_cast<self_type&>(*this), vidx + 1u};
        if (connectivity_[vidx + 1u].empty())
            ++edge_end;
        return {edge_begin, edge_end};
    }

    vertex_iterator_type add_vertex(vertex_type const& v)
    {
        auto const idx    = vertices_.size();
        auto const offset = static_cast<difference_type>(idx);
        vertices_.push_back(v);
        connectivity_.push_back({});
        return std::begin(vertices_) + offset;
    }

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

    edge_iterator_type add_edge(const_vertex_iterator_type uit, const_vertex_iterator_type vit)
    {
        auto const begin   = std::cbegin(vertices_);
        auto const uoffset = std::distance(begin, uit);
        auto const voffset = std::distance(begin, vit);
        auto const uidx    = static_cast<size_type>(uoffset);
        auto const vidx    = static_cast<size_type>(voffset);
        auto const i       = uidx;
        auto const j       = connectivity_[uidx].size();
        connectivity_[uidx].push_back(vidx);
        return edge_iterator_type{*this, i, j};
    }

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

    void clear()
    {
        vertices_.clear();
        connectivity_.clear();
    }

  private:
    vertices_type vertices_;
    connectivity_type connectivity_;
};

template <class Vertex>
class adjacency_list_edge_iterator_t
{
  public:
    using self_type            = adjacency_list_edge_iterator_t<Vertex>;
    using graph_type           = adjacency_list_t<Vertex>;
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

    adjacency_list_edge_iterator_t(graph_type& graph) : graph_(graph), i_(), j_() {}
    adjacency_list_edge_iterator_t(graph_type& graph, size_type i) : graph_(graph), i_(i), j_() {}
    adjacency_list_edge_iterator_t(graph_type& graph, size_type i, size_type j)
        : graph_(graph), i_(i), j_(j)
    {
    }

    adjacency_list_edge_iterator_t(self_type const&)     = default;
    adjacency_list_edge_iterator_t(self_type&&) noexcept = default;
    self_type& operator=(self_type const&) = default;
    self_type& operator=(self_type&&) noexcept = default;

    reference operator*() const
    {
        auto const ioffset         = static_cast<difference_type>(i_);
        auto u                     = std::begin(graph_.vertices_) + ioffset;
        auto const neighbor_offset = static_cast<difference_type>(graph_.connectivity_[i_][j_]);
        auto v                     = std::begin(graph_.vertices_) + neighbor_offset;
        return {u, v};
    }

    self_type& operator++()
    {
        if (++j_ >= graph_.connectivity_[i_].size())
        {
            do
            {
                j_ = 0u;
                ++i_;
            } while (i_ < graph_.connectivity_.size() && graph_.connectivity_[i_].empty());
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
                j_ = graph_.connectivity_[i_].size();
            } while (i_ >= 0u && graph_.connectivity_[i_].empty());
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
        auto const vertex_count = graph_.vertex_count();
        while (n > 0 && i < vertex_count)
        {
            auto const remaining_increments =
                static_cast<difference_type>(graph_.connectivity_[i].size() - j);
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
            } while (i < graph_.connectivity_.size() && graph_.connectivity_[i].empty());
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
        auto const begin  = std::cbegin(graph_.connectivity_);
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
                j = graph_.connectivity_[i].size() - 1;
            } while (i >= 0u && graph_.connectivity_[i].empty());
        }
        return self_type{graph_, i, j};
    }
    friend self_type operator+(difference_type n, self_type const& self) { return self + n; }

    reference operator[](difference_type n) const { return *(*this + n); }

    bool operator==(self_type const& other) const
    {
        return (&graph_ == &(other.graph_)) && (i_ == other.i_) && (j_ == other.j_);
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
    graph_type& graph_;
    size_type i_;
    size_type j_;
};

} // namespace graph
} // namespace pcp
