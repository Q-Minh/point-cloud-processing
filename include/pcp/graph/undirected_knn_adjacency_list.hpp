#pragma once

#include "pcp/traits/knn_search_traits.hpp"

#include <utility>
#include <vector>

namespace pcp {
namespace graph {

template <class Vertex>
class undirected_knn_adjacency_list_edge_iterator_t;

template <class Vertex>
class undirected_knn_adjacency_list_t
{
    friend class undirected_knn_adjacency_list_edge_iterator_t<Vertex>;

  public:
    using vertex_type                = std::remove_cv_t<Vertex>;
    using self_type                  = undirected_knn_adjacency_list_t<Vertex>;
    using k_type                     = uint32_t;
    using vertices_type              = std::vector<Vertex>;
    using vertex_neighborhoods_type  = std::vector<Vertex>;
    using vertex_iterator_type       = typename vertices_type::iterator;
    using const_vertex_iterator_type = typename vertices_type::const_iterator;
    using vertex_iterator_range      = std::pair<vertex_iterator_type, vertex_iterator_type>;
    using const_vertex_iterator_range =
        std::pair<const_vertex_iterator_type, const_vertex_iterator_type>;
    using edge_iterator_type  = undirected_knn_adjacency_list_edge_iterator_t<Vertex>;
    using edge_iterator_range = std::pair<edge_iterator_type, edge_iterator_type>;
    using size_type           = typename vertices_type::size_type;

    undirected_knn_adjacency_list_t() noexcept                       = default;
    undirected_knn_adjacency_list_t(self_type const& other) noexcept = default;
    undirected_knn_adjacency_list_t(self_type&& other) noexcept      = default;

    /**
     * @brief Constructs the graph from a range of vertices and a user-provided k nearest neighbors
     * searcher.
     *
     * @tparam ForwardIter Type of the iterator where ForwardIter::value_type is convertible to
     * Vertex
     * @tparam KnnSearcher Type of callable satisfying KnnSearcher concept
     * @param begin
     * @param end
     * @param k Number of neighbors to search for
     * @param knn The knn searcher
     */
    template <class ForwardIter, class KnnSearcher>
    directed_knn_adjacency_list_t(ForwardIter begin, ForwardIter end, k_type k, KnnSearcher&& knn)
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

    /**
     * @brief Construct graph from range of vertices and knn searcher.
     * @tparam ForwardIter Type of the iterator where ForwardIter::value_type is convertible to
     * Vertex
     * @tparam KnnSearcher Type of callable satisfying KnnSearcher concept
     * @param begin
     * @param end
     * @param knn The knn searcher
     */
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
            vertex_type const& vertex = *it;
            auto const neighbors      = knn(vertex, k_);
            vertices_.push_back(vertex);
            std::copy(
                std::begin(neighbors),
                std::end(neighbors),
                std::back_inserter(vertex_neighborhoods_));
        }
    }

  private:
    k_type k_;
    vertices_type vertices_;
    
};

} // namespace graph
} // namespace pcp