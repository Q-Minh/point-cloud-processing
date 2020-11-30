#pragma once

#include <iterator>
#include <pcp/common/norm.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/normals/normal_estimation.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/graph/minimum_spanning_tree.hpp>
#include <pcp/graph/undirected_knn_adjacency_list.hpp>
#include <pcp/traits/graph_vertex_traits.hpp>
#include <pcp/traits/knn_search_traits.hpp>
#include <pcp/traits/normal_traits.hpp>
#include <pcp/traits/point_traits.hpp>
#include <utility>

namespace pcp {

/**
 * @brief
 * @tparam ForwardIter2
 * @tparam ForwardIter1
 * @tparam KnnSearcher
 * @tparam TransformOp
 * @tparam Normal
 * @param begin
 * @param end
 * @param out_begin
 * @param knn
 * @param op
 * @param k
 */
template <
    class ForwardIter1,
    class ForwardIter2,
    class KnnSearcher,
    class TransformOp,
    class Normal = pcp::normal_t>
void compute_normals(
    ForwardIter1 begin,
    ForwardIter1 end,
    ForwardIter2 out_begin,
    KnnSearcher&& knn,
    TransformOp&& op,
    std::uint64_t k = 5u)
{
    using point_view_type = typename std::iterator_traits<ForwardIter1>::value_type;
    using k_type          = decltype(k);
    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, point_view_type, k_type>,
        "knn must satisfy KnnSearcher concept");

    using knn_result_type          = std::invoke_result_t<KnnSearcher, point_view_type, k_type>;
    using knn_result_iterator_type = decltype(std::declval<knn_result_type&>().begin());
    using point_type               = decltype(*std::declval<knn_result_iterator_type&>());
    static_assert(traits::is_point_v<point_type>, "Return type of knn must satisfy Point concept");

    using normal_type = Normal;
    static_assert(traits::is_normal_v<normal_type>, "Normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, normal_type, point_view_type>,
        "op must be callable by result = op(Normal, *begin) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [knn = std::forward<KnnSearcher>(knn),
                               op  = std::forward<TransformOp>(op),
                               k](point_view_type const& p) {
        auto const neighbor_points = knn(p, k);
        auto const normal =
            pcp::estimate_normal(std::begin(neighbor_points), std::end(neighbor_points));
        return op(normal, p);
    };

    std::transform(begin, end, out_begin, transform_op);
}

template <
    class ForwardIter1,
    class ForwardIter2,
    class KnnSearcher,
    class GetPointOp,
    class GetNormalOp,
    class TransformOp>
void compute_normal_orientations(
    ForwardIter1 begin,
    ForwardIter1 end,
    ForwardIter2 out_begin,
    KnnSearcher&& knn,
    GetPointOp&& get_point,
    GetNormalOp&& get_normal,
    TransformOp&& op,
    std::uint64_t k = 5u)
{
    using input_element_type  = typename std::iterator_traits<ForwardIter1>::value_type;
    using output_element_type = typename std::iterator_traits<ForwardIter2>::value_type;
    using k_type              = decltype(k);

    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, input_element_type, k_type>,
        "knn must satisfy KnnSearcher concept");

    static_assert(
        traits::is_graph_vertex_v<input_element_type>,
        "*begin must satisfy GraphVertex concept");

    static_assert(
        std::is_invocable_v<GetNormalOp, input_element_type>,
        "GetNormalOp must be able to return normal from call to GetNormalOp(*begin)");

    using normal_type = std::invoke_result_t<GetNormalOp>;

    static_assert(
        std::is_invocable_r_v<output_element_type, TransformOp, normal_type>,
        "TransformOp must return type similar to decltype(*out_begin) and take as parameter the "
        "return type of GetNormalOp");

    auto graph = graph::undirected_knn_graph(begin, end, std::forward<KnnSearcher>(knn), k);
    auto [vbegin, vend] = graph.vertices();

    auto const is_higher =
        [get_point = std::forward<GetPointOp>(get_point)](auto const& v1, auto const& v2) {
            auto const& p1 = get_point(v1);
            auto const& p2 = get_point(v2);
            return p1.z() < p2.z();
        };

    auto const root = std::max_element(begin, end, is_higher);

    auto const cost = [get_normal =
                           std::forward<GetNormalOp>(get_normal)](auto const& v1, auto const& v2) {
        auto const& n1  = get_normal(v1);
        auto const& n2  = get_normal(v2);
        auto const prod = common::inner_product(n1, n2);
        using floating_point_type = decltype(prod);
        auto const one            = static_cast<floating_point_type>(1.0);
        return one - std::abs(prod);
    };

     auto [mst, get_root] = graph::prim_minimum_spanning_tree(g, cost, root);

}

} // namespace pcp