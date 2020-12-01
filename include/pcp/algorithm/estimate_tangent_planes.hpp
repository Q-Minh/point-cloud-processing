#pragma once

#include <iterator>
#include <pcp/common/normals/normal_estimation.hpp>
#include <pcp/common/plane3d.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/traits/knn_search_traits.hpp>
#include <pcp/traits/normal_traits.hpp>
#include <pcp/traits/plane_traits.hpp>
#include <pcp/traits/point_traits.hpp>

namespace pcp {
namespace algorithm {

/**
 * @brief
 * Performs tangent plane estimation on each knn neighborhood of the given sequence
 * of elements using PCA. Results are stored in the out sequence through op.
 * @tparam ExecutionPolicy STL execution policy
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam KnnSearcher Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning an output element with parameters (input element,
 * tangent plane)
 * @tparam Plane Type of tangent plane
 * @param policy
 * @param begin
 * @param end
 * @param out_begin Start iterator of output sequence
 * @param knn The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed tangent plane
 * and returning an output sequence element
 */
template <
    class ExecutionPolicy,
    class ForwardIter1,
    class ForwardIter2,
    class KnnSearcher,
    class TransformOp,
    class Plane = pcp::common::plane3d_t>
void estimate_tangent_planes(
    ExecutionPolicy&& policy,
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

    using neighbor_points_type = std::invoke_result_t<KnnSearcher, value_type>;
    using neighbor_point_type  = traits::value_type_of_iterable_t<neighbor_points_type>;
    static_assert(
        traits::is_point_v<neighbor_point_type>,
        "KnnSearcher must return points satisfying Point concept");

    using plane_type  = Plane;
    using point_type  = typename plane_type::point_type;
    using normal_type = typename plane_type::normal_type;
    static_assert(traits::is_plane_v<plane_type>, "Plane must satisfy Plane concept");
    static_assert(traits::is_point_v<point_type>, "Plane's point must satisfy Point concept");
    static_assert(traits::is_normal_v<normal_type>, "Plane's normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, plane_type>,
        "op must be callable by result = op(*begin, plane) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [knn = std::forward<KnnSearcher>(knn),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points));

        auto point = pcp::common::center_of_geometry<iterator_type, neighbor_point_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points));

        return op(v, plane_type{point_type{point}, normal});
    };

    std::transform(std::forward<ExecutionPolicy>(policy), begin, end, out_begin, transform_op);
}

/**
 * @brief
 * Performs tangent plane estimation on each knn neighborhood of the given sequence
 * of elements using PCA. Results are stored in the out sequence through op.
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam KnnSearcher Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning an output element with parameters (input element,
 * tangent plane)
 * @tparam Plane Type of tangent plane
 * @param begin
 * @param end
 * @param out_begin Start iterator of output sequence
 * @param knn The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed tangent plane
 * and returning an output sequence element
 */
template <
    class ForwardIter1,
    class ForwardIter2,
    class KnnSearcher,
    class TransformOp,
    class Plane = pcp::common::plane3d_t>
void estimate_tangent_planes(
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

    using neighbor_points_type = std::invoke_result_t<KnnSearcher, value_type>;
    using neighbor_point_type  = traits::value_type_of_iterable_t<neighbor_points_type>;
    static_assert(
        traits::is_point_v<neighbor_point_type>,
        "KnnSearcher must return points satisfying Point concept");

    using plane_type  = Plane;
    using point_type  = typename plane_type::point_type;
    using normal_type = typename plane_type::normal_type;
    static_assert(traits::is_plane_v<plane_type>, "Plane must satisfy Plane concept");
    static_assert(traits::is_point_v<point_type>, "Plane's point must satisfy Point concept");
    static_assert(traits::is_normal_v<normal_type>, "Plane's normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, plane_type>,
        "op must be callable by result = op(*begin, plane) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [knn = std::forward<KnnSearcher>(knn),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points));

        auto point = pcp::common::center_of_geometry<iterator_type, neighbor_point_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points));

        return op(v, plane_type{point_type{point}, normal});
    };

    std::transform(begin, end, out_begin, transform_op);
}

} // namespace algorithm
} // namespace pcp