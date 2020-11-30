#pragma once

#include <iterator>
#include <pcp/common/norm.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/normals/normal_estimation.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/graph/minimum_spanning_tree.hpp>
#include <pcp/graph/search.hpp>
#include <pcp/graph/undirected_knn_adjacency_list.hpp>
#include <pcp/traits/graph_vertex_traits.hpp>
#include <pcp/traits/knn_search_traits.hpp>
#include <pcp/traits/normal_traits.hpp>
#include <pcp/traits/point_traits.hpp>
#include <utility>

namespace pcp {

/**
 * @brief
 * Performs normal estimation on each knn neighborhood of the given sequence
 * of elements using tangent plane estimation through PCA. Results are stored
 * in the out sequence through op.
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam KnnSearcher Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning a Normal with parameters (input element, normal)
 * @tparam Normal Type of normal
 * @param begin 
 * @param end
 * @param out_begin Start iterator of output sequence
 * @param knn The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed normal and
 * returning an output sequence element
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
    TransformOp&& op)
{
    using value_type = typename std::iterator_traits<ForwardIter1>::value_type;
    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, value_type>,
        "knn must satisfy KnnSearcher concept");

    using normal_type = Normal;
    static_assert(traits::is_normal_v<normal_type>, "Normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, normal_type>,
        "op must be callable by result = op(Normal, *begin) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [knn = std::forward<KnnSearcher>(knn),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points));
        return op(v, normal);
    };

    std::transform(begin, end, out_begin, transform_op);
}

/**
 * @brief
 * Adjusts the orientations of a point cloud's normals using a minimum spanning tree 
 * of the KNN graph of the point cloud, and propagating the MST's root's normal 
 * through the MST.
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam GetNormalOp Callable type returning a normal from an input element
 * @tparam TransformOp Callable type taking an input element and its oriented normal
 * @tparam KnnSearcher Callable type computing the k neighborhood of an input element
 * @tparam GetPointOp Callable type returning a point from an input element
 * @param begin
 * @param end
 * @param knn Callable query object for k neighborhoods
 * @param get_point Callable object to get a point from an input element
 * @param get_normal Callable object to get a normal from an input element
 * @param op Transformation callable object taking an input element and its computed normal
 */
template <
    class ForwardIter1,
    class KnnSearcher,
    class GetPointOp,
    class GetNormalOp,
    class TransformOp>
void compute_normal_orientations(
    ForwardIter1 begin,
    ForwardIter1 end,
    KnnSearcher&& knn,
    GetPointOp&& get_point,
    GetNormalOp& get_normal,
    TransformOp&& op)
{
    using input_element_type = typename std::iterator_traits<ForwardIter1>::value_type;

    static_assert(
        traits::is_knn_searcher_v<KnnSearcher, input_element_type>,
        "knn must satisfy KnnSearcher concept");

    static_assert(
        traits::is_graph_vertex_v<input_element_type>,
        "*begin must satisfy GraphVertex concept");

    static_assert(
        std::is_invocable_v<GetNormalOp, input_element_type>,
        "GetNormalOp must be able to return normal from call to GetNormalOp(*begin)");

    using normal_type = std::remove_reference_t<
        std::remove_cv_t<std::invoke_result_t<GetNormalOp, input_element_type>>>;

    static_assert(
        std::is_invocable_v<TransformOp, input_element_type, normal_type>,
        "TransformOp must be callable as op(*begin, get_normal(...))");

    auto graph          = graph::undirected_knn_graph(begin, end, std::forward<KnnSearcher>(knn));
    auto [vbegin, vend] = graph.vertices();

    auto const is_higher =
        [get_point = std::forward<GetPointOp>(get_point)](auto const& v1, auto const& v2) {
            auto const& p1 = get_point(v1);
            auto const& p2 = get_point(v2);
            return p1.z() < p2.z();
        };

    auto const root = std::max_element(vbegin, vend, is_higher);

    auto const cost = [get_normal](auto const& v1, auto const& v2) {
        auto const& n1            = get_normal(v1);
        auto const& n2            = get_normal(v2);
        auto const prod           = common::inner_product(n1, n2);
        using floating_point_type = decltype(prod);
        auto const one            = static_cast<floating_point_type>(1.0);
        return one - std::abs(prod);
    };

    auto [mst, get_root] = graph::prim_minimum_spanning_tree(graph, cost, root);
    auto mst_root        = get_root(mst);
    graph::depth_first_search(
        mst,
        mst_root,
        [op = std::forward<TransformOp>(op), get_normal](auto const& v1, auto const& v2) {
            auto const& n1            = get_normal(v1);
            auto const& n2            = get_normal(v2);
            auto const prod           = common::inner_product(n1, n2);
            using floating_point_type = decltype(prod);
            auto const zero           = static_cast<floating_point_type>(0.0);
            // flip normal orientation if
            // the angle between n1, n2 is
            // > 90 degrees
            if (prod < zero)
                op(v2, -n2);
        });
}

} // namespace pcp