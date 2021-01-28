#ifndef PCP_ALGORITHM_ESTIMATE_NORMALS_HPP
#define PCP_ALGORITHM_ESTIMATE_NORMALS_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/norm.hpp"
#include "pcp/common/normals/normal.hpp"
#include "pcp/common/normals/normal_estimation.hpp"
#include "pcp/common/points/point.hpp"
#include "pcp/graph/knn_adjacency_list.hpp"
#include "pcp/graph/search.hpp"
#include "pcp/traits/knn_map.hpp"
#include "pcp/traits/normal_traits.hpp"
#include "pcp/traits/point_traits.hpp"

#include <execution>
#include <iterator>
#include <utility>

namespace pcp {
namespace algorithm {

/**
 * @ingroup normals-estimation
 * @brief
 * Performs normal estimation on each knn_map neighborhood of the given sequence
 * of elements using tangent plane estimation through PCA. Results are stored
 * in the out sequence through op.
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam PointViewMap Type satisfying PointViewMap concept
 * @tparam KnnMap Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning an output element with parameters (input element,
 * normal)
 * @tparam Normal Type of normal
 * @tparam ExecutionPolicy Type of STL execution policy
 * @param policy The execution policy
 * @param begin Iterator to start of input sequence of elements
 * @param end Iterator to one past the end of input sequence of elements
 * @param out_begin Start iterator of output sequence
 * @param point_map The point view map property map
 * @param knn_map The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed normal and
 * returning an output sequence element
 */
template <
    class ExecutionPolicy,
    class ForwardIter1,
    class ForwardIter2,
    class PointViewMap,
    class KnnMap,
    class TransformOp,
    class Normal = pcp::normal_t>
void estimate_normals(
    ExecutionPolicy&& policy,
    ForwardIter1 begin,
    ForwardIter1 end,
    ForwardIter2 out_begin,
    PointViewMap const& point_map,
    KnnMap&& knn_map,
    TransformOp&& op)
{
    using value_type = typename std::iterator_traits<ForwardIter1>::value_type;
    static_assert(
        traits::is_knn_map_v<KnnMap, value_type>,
        "knn_map must satisfy KnnMap concept");

    using normal_type = Normal;
    static_assert(traits::is_normal_v<normal_type>, "Normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, normal_type>,
        "op must be callable by result = op(Normal, *begin) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [&,
                               knn = std::forward<KnnMap>(knn_map),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, PointViewMap, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points),
            point_map);
        return op(v, normal);
    };

    std::transform(std::forward<ExecutionPolicy>(policy), begin, end, out_begin, transform_op);
}

/**
 * @ingroup normals-estimation
 * @brief
 * Performs normal estimation on each knn_map neighborhood of the given sequence
 * of elements using tangent plane estimation through PCA. Results are stored
 * in the out sequence through op.
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam PointViewMap Type satisfying PointViewMap concept
 * @tparam KnnMap Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning an output element with parameters (input element,
 * normal)
 * @tparam Normal Type of normal
 * @param begin Iterator to start of input sequence of elements
 * @param end Iterator to one past the end of input sequence of elements
 * @param out_begin Start iterator of output sequence
 * @param point_map The point view map property map
 * @param knn_map The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed normal and
 * returning an output sequence element
 */
template <
    class ForwardIter1,
    class ForwardIter2,
    class PointViewMap,
    class KnnMap,
    class TransformOp,
    class Normal = pcp::normal_t>
void estimate_normals(
    ForwardIter1 begin,
    ForwardIter1 end,
    ForwardIter2 out_begin,
    PointViewMap const& point_map,
    KnnMap&& knn_map,
    TransformOp&& op)
{
    /**
     * Ideally, we would just want to delegate the call to the version of estimate_normals
     * that uses an execution policy, and simply give it the sequenced policy, but doing so
     * does not support std::back_inserter. STL execution policies only work with at least
     * forward iterators, which back_inserter is not. For this reason, we have to duplicate
     * the normals estimation implementation.
     */
    using value_type = typename std::iterator_traits<ForwardIter1>::value_type;
    static_assert(
        traits::is_knn_map_v<KnnMap, value_type>,
        "knn_map must satisfy KnnMap concept");

    using normal_type = Normal;
    static_assert(traits::is_normal_v<normal_type>, "Normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, normal_type>,
        "op must be callable by result = op(Normal, *begin) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [&,
                               knn = std::forward<KnnMap>(knn_map),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, PointViewMap, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points),
            point_map);
        return op(v, normal);
    };

    std::transform(begin, end, out_begin, transform_op);
}

/**
 * @ingroup normals-estimation
 * @brief
 * Adjusts the orientations of a point cloud's normals using a minimum spanning tree
 * of the KNN graph of the point cloud, and propagating the MST's root's normal
 * through the MST.
 *
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam IndexMap Type satisfying IndexMap concept
 * @tparam KnnMap Callable type computing the k neighborhood of an input element
 * @tparam PointViewMap Callable type returning a point from an input element
 * @tparam NormalMap Callable type returning a normal from an input element
 * @tparam TransformOp Callable type taking an input element and its oriented normal
 * @param begin
 * @param end
 * @param index_map The index map property map
 * @param knn_map Callable query object for k neighborhoods
 * @param point_map Callable object to get a point from an input element
 * @param normal_map Callable object to get a normal from an input element
 * @param op Transformation callable object taking an input element and its computed normal
 */
template <
    class ForwardIter1,
    class IndexMap,
    class KnnMap,
    class PointViewMap,
    class NormalMap,
    class TransformOp>
void propagate_normal_orientations(
    ForwardIter1 begin,
    ForwardIter1 end,
    IndexMap const& index_map,
    KnnMap&& knn_map,
    PointViewMap&& point_map,
    NormalMap& normal_map,
    TransformOp&& op)
{
    using input_element_type = typename std::iterator_traits<ForwardIter1>::value_type;

    static_assert(
        traits::is_knn_map_v<KnnMap, input_element_type>,
        "knn_map must satisfy KnnMap concept");

    static_assert(
        std::is_invocable_v<NormalMap, input_element_type>,
        "NormalMap must be able to return normal from call to NormalMap(*begin)");

    using normal_type = std::remove_reference_t<
        std::remove_cv_t<std::invoke_result_t<NormalMap, input_element_type>>>;

    static_assert(
        std::is_invocable_v<TransformOp, input_element_type, normal_type>,
        "TransformOp must be callable as op(*begin, normal_map(...))");

    auto graph = graph::directed_knn_graph(begin, end, std::forward<KnnMap>(knn_map), index_map);
    auto [vbegin, vend] = graph.vertices();

    auto const is_higher =
        [get_point = std::forward<PointViewMap>(point_map)](auto const& v1, auto const& v2) {
            auto const& p1 = get_point(v1);
            auto const& p2 = get_point(v2);
            return p1.z() < p2.z();
        };

    auto const root = std::max_element(vbegin, vend, is_higher);

    using floating_point_type = typename normal_type::component_type;
    op(*root,
       normal_type{
           static_cast<floating_point_type>(0.0),
           static_cast<floating_point_type>(0.0),
           static_cast<floating_point_type>(1.0)});

    /**
     * In Hoppe '92, normal orientation adjustments use a minimum spanning tree
     * using a cost function that assigns low weights to similarly oriented
     * normals, and high weights to normals approaching orthogonality. He then
     * traverses the MST and flips normal orientations when their inner-product
     * is negative. It seems that in our case, prim's minimum spanning tree algorithm
     * cannot work, since our knn_map graphs are directed rather than undirected. The
     * reason for the MST not working is not clear. We thought that having an
     * undirected graph in the form a directed graph where each undirected edge
     * is split into two parallel and opposite directed edges would be enough
     * for prim's mst algorithm to work, but I think it doesn't and I believe
     * that that is the culprit for the normal orientation adjustments not
     * working when using the MST version.
     */
    // auto const cost = [normal_map](auto const& v1, auto const& v2) {
    //    auto const& n1            = normal_map(v1);
    //    auto const& n2            = normal_map(v2);
    //    auto const prod           = common::inner_product(n1, n2);
    //    using floating_point_type = decltype(prod);
    //    auto const one            = static_cast<floating_point_type>(1.0);
    //    return one - std::abs(prod);
    //};
    //
    // auto [mst, get_root]      = graph::prim_minimum_spanning_tree(graph, cost, root);
    // auto mst_root             = get_root(mst);
    // graph::depth_first_search(
    //    mst,
    //    mst_root,
    //    [op = std::forward<TransformOp>(op), normal_map](auto const& v1, auto const& v2) {
    //        auto const& n1            = normal_map(v1);
    //        auto const& n2            = normal_map(v2);
    //        auto const prod           = common::inner_product(n1, n2);
    //        using floating_point_type = decltype(prod);
    //        auto const zero           = static_cast<floating_point_type>(0.0);
    //        // flip normal orientation if
    //        // the angle between n1, n2 is
    //        // > 90 degrees
    //        if (prod < zero)
    //        {
    //            op(v2, -n2);
    //        }
    //    });

    /**
     * Use BFS (Breadth First Search) instead of Hoppe '92 MST propagation
     */
    graph::breadth_first_search(
        graph,
        root,
        index_map,
        [op = std::forward<TransformOp>(op), normal_map](auto const& v1, auto const& v2) {
            auto const& n1  = normal_map(v1);
            auto const& n2  = normal_map(v2);
            auto const prod = common::inner_product(n1, n2);
            auto const zero = static_cast<floating_point_type>(0.0);
            // flip normal orientation if
            // the angle between n1, n2 is
            // > 90 degrees
            if (prod < zero)
            {
                op(v2, -n2);
            }
        });
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_ESTIMATE_NORMALS_HPP