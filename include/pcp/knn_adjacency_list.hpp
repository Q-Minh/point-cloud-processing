#pragma once

#include <vector>

namespace pcp {

template <class Point, class Index = size_t>
class static_knn_adjacency_list
{
  public:
    using index_type                = Index;
    using point_type                = Point;
    using self_type                 = static_knn_adjacency_list<Point, Index>;
    using k_type                    = std::uint32_t;
    using vertices_type             = std::vector<Point>;
    using vertex_neighborhoods_type = std::vector<Point>;

    static_knn_adjacency_list() = default;

    template <class ForwardIter>
    static_knn_adjacency_list(ForwardIter begin, ForwardIter end, k_type k)
        : k_(k), vertices_(begin, end), vertex_neighborhoods_(k * vertices_.size())
    {
    }

    k_type const& k() const { return k_; }

    template <class ForwardIter>
    void build_from(ForwardIter begin, ForwardIter end)
    {
    }

  private:
    std::uint32_t k_;
    vertices_type vertices_;
    vertex_neighborhoods_type vertex_neighborhoods_;
};

} // namespace pcp