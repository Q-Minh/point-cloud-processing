#pragma once

#include "pcp/traits/knn_search_traits.hpp"

#include <utility>
#include <vector>

namespace pcp {
namespace graph {

template <class Vertex>
class knn_adjacency_list_edge_iterator_t;

template <class Vertex>
class static_knn_adjacency_list_t
{
    friend class knn_adjacency_list_edge_iterator_t<Vertex>;

  public:
    using vertex_type               = std::remove_cv_t<Vertex>;
    using self_type                 = static_knn_adjacency_list_t<Vertex>;
    using k_type                    = std::uint32_t;
    using vertices_type             = std::vector<Vertex>;
    using vertex_neighborhoods_type = std::vector<Vertex>;
    using vertex_iterator_type      = typename vertices_type::const_iterator;
    using vertex_iterator_range     = std::pair<vertex_iterator_type, vertex_iterator_type>;
    using edge_iterator_type        = knn_adjacency_list_edge_iterator_t<Vertex>;
    using edge_iterator_range       = std::pair<edge_iterator_type, edge_iterator_type>;
    using size_type                 = typename vertices_type::size_type;

    static_knn_adjacency_list_t() noexcept                       = default;
    static_knn_adjacency_list_t(self_type const& other) noexcept = default;
    static_knn_adjacency_list_t(self_type&& other) noexcept      = default;

    template <class ForwardIter, class KnnSearcher>
    static_knn_adjacency_list_t(ForwardIter begin, ForwardIter end, k_type k, KnnSearcher&& knn)
        : k_(k), vertices_(), vertex_neighborhoods_()
    {
        static_assert(
            std::is_convertible_v<std::remove_cv_t<typename ForwardIter::value_type>, vertex_type>,
            "ForwardIter::value_type must be convertible to Vertex");
        static_assert(
            traits::is_knn_searcher_v<KnnSearcher, vertex_type, k_type>,
            "KnnSearcher must satisfy KnnSearcher concept");
        build_from(begin, end, std::forward<KnnSearcher>(knn));
    }

    bool empty() const { return vertices_.empty(); }
    size_type vertex_count() const { return vertices_.size(); }
    size_type edge_count() const { return vertices_.size() * k_; }

    vertex_iterator_range vertices() const { return {std::begin(vertices_), std::end(vertices_)}; };

    edge_iterator_range edges() const
    {
        using diff_t = typename edge_iterator_type::difference_type;
        return {
            edge_iterator_type{const_cast<self_type&>(*this)},
            edge_iterator_type{
                const_cast<self_type&>(*this),
                static_cast<diff_t>(vertex_neighborhoods_.size())}};
    }

    edge_iterator_range out_edges_of(vertex_iterator_type const& v) const
    {
        auto const n      = std::distance<vertex_iterator_type>(std::begin(vertices_), v);
        auto const offset = n * k_;
        auto const end    = offset + k_;
        return {
            edge_iterator_type(const_cast<self_type&>(*this), offset),
            edge_iterator_type(const_cast<self_type&>(*this), end)};
    }

    k_type const& k() const { return k_; }
    void k(k_type k) { k_ = k; }

    template <class ForwardIter, class KnnSearcher>
    void build_from(ForwardIter begin, ForwardIter end, KnnSearcher&& knn)
    {
        vertices_.clear();
        vertex_neighborhoods_.clear();

        auto const vertex_count      = std::distance(begin, end);
        auto const neighborhood_size = vertex_count * k_;
        vertices_.reserve(vertex_count);
        vertex_neighborhoods_.reserve(neighborhood_size);
        auto it = begin;
        for (size_t i = 0; i < vertex_count; ++i, ++it)
        {
            vertex_type const& vertex      = *it;
            auto const neighbors           = knn(vertex, k_);
            auto const neighborhood_offset = i * k_;
            vertices_.push_back(vertex);
            std::copy(
                std::begin(neighbors),
                std::end(neighbors),
                std::back_inserter(vertex_neighborhoods_));
        }
    }

  private:
    std::uint32_t k_;
    vertices_type vertices_;
    vertex_neighborhoods_type vertex_neighborhoods_;
};

template <class Vertex>
class knn_adjacency_list_edge_iterator_t
{
  public:
    using self_type                 = knn_adjacency_list_edge_iterator_t<Vertex>;
    using graph_type                = static_knn_adjacency_list_t<Vertex>;
    using vertices_type             = typename graph_type::vertices_type;
    using vertex_iterator_type      = typename graph_type::vertex_iterator_type;
    using vertex_neighborhoods_type = typename graph_type::vertex_neighborhoods_type;

    using value_type        = typename std::pair<vertex_iterator_type, vertex_iterator_type>;
    using reference         = value_type;
    using const_reference   = value_type const;
    using pointer           = value_type*;
    using const_pointer     = value_type const*;
    using iterator_category = std::random_access_iterator_tag;
    using difference_type   = typename vertices_type::difference_type;

    knn_adjacency_list_edge_iterator_t(self_type const&) = default;
    knn_adjacency_list_edge_iterator_t(graph_type& graph) : graph_(graph), n_() {}
    knn_adjacency_list_edge_iterator_t(graph_type& graph, difference_type n) : graph_(graph), n_(n)
    {
    }

    reference operator*() const
    {
        auto const source_vertex_index = get_source_vertex_index();
        auto u                         = std::begin(graph_.vertices_) + source_vertex_index;
        auto v                         = std::begin(graph_.vertex_neighborhoods_) + n_;
        return std::make_pair(u, v);
    }

    self_type& operator++()
    {
        ++n_;
        return *this;
    }

    self_type operator++(int)
    {
        self_type copy{*this};
        ++n_;
        return copy;
    }

    self_type const& operator--()
    {
        --n_;
        return *this;
    }

    self_type operator--(int)
    {
        self_type copy{*this};
        --n_;
        return copy;
    }

    self_type const& operator+=(difference_type n)
    {
        n_ += n;
        return *this;
    }

    self_type operator+(difference_type n) const { return self_type{graph_, n_ + n}; }
    friend self_type operator+(difference_type n, self_type const& it) { return it + n; }

    self_type const& operator-=(difference_type n)
    {
        n_ -= n;
        return *this;
    }

    self_type operator-(difference_type n) const { return self_type{graph_, n_ - n}; }
    difference_type operator-(self_type const& other) const { return n_ - other.n_; }

    reference operator[](difference_type n) const
    {
        self_type copy{graph_, n};
        return *copy;
    }

    bool operator==(self_type const& other) const
    {
        return &graph_ == &(other.graph_) && n_ == other.n_;
    }

    bool operator!=(self_type const& other) const { return !(*this == other); }

    bool operator<(self_type const& other) const { return n_ < other.n_; }
    bool operator>(self_type const& other) const { return n_ > other.n_; }
    bool operator<=(self_type const& other) const { return (*this < other) || (*this == other); }
    bool operator>=(self_type const& other) const { return (*this > other) || (*this == other); }

  private:
    difference_type get_source_vertex_index() const
    {
        auto const k                   = graph_.k();
        auto const source_vertex_index = n_ / k;
        return source_vertex_index;
    }

    graph_type& graph_;
    difference_type n_;
};

} // namespace graph
} // namespace pcp