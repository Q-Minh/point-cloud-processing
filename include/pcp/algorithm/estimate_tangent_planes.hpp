#ifndef PCP_ALGORITHM_ESTIMATE_TANGENT_PLANES_HPP
#define PCP_ALGORITHM_ESTIMATE_TANGENT_PLANES_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/normals/normal_estimation.hpp"
#include "pcp/common/plane3d.hpp"
#include "pcp/common/vector3d_queries.hpp"
#include "pcp/traits/knn_map.hpp"
#include "pcp/traits/normal_traits.hpp"
#include "pcp/traits/plane_traits.hpp"
#include "pcp/traits/point_traits.hpp"

#include <iterator>

namespace pcp {
namespace algorithm {

/**
 * @ingroup tangent-planes-estimation
 * @brief
 * Performs tangent plane estimation on each knn_map neighborhood of the given sequence
 * of elements using PCA. Results are stored in the out sequence through op.
 * @tparam ExecutionPolicy STL execution policy
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam KnnMap Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning an output element with parameters (input element,
 * tangent plane)
 * @tparam Plane Type of tangent plane
 * @param policy
 * @param begin
 * @param end
 * @param out_begin Start iterator of output sequence
 * @param knn_map The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed tangent plane
 * and returning an output sequence element
 */
template <
    class ExecutionPolicy,
    class ForwardIter1,
    class ForwardIter2,
    class PointMap,
    class KnnMap,
    class TransformOp,
    class Plane = pcp::common::plane3d_t>
void estimate_tangent_planes(
    ExecutionPolicy&& policy,
    ForwardIter1 begin,
    ForwardIter1 end,
    ForwardIter2 out_begin,
    PointMap const& point_map,
    KnnMap&& knn_map,
    TransformOp&& op)
{
    using value_type  = typename std::iterator_traits<ForwardIter1>::value_type;
    using plane_type  = Plane;
    using point_type  = typename plane_type::point_type;
    using normal_type = typename plane_type::normal_type;

    static_assert(
        traits::is_point_map_v<PointMap, value_type>,
        "point_map must satisfy PointMap concept");
    static_assert(traits::is_knn_map_v<KnnMap, value_type>, "knn_map must satisfy KnnMap concept");
    static_assert(traits::is_plane_v<plane_type>, "Plane must satisfy Plane concept");
    static_assert(traits::is_point_v<point_type>, "Plane's point must satisfy Point concept");
    static_assert(traits::is_normal_v<normal_type>, "Plane's normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, plane_type>,
        "op must be callable by result = op(*begin, plane) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [&,
                               knn = std::forward<KnnMap>(knn_map),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, PointMap, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points),
            point_map);

        auto point = pcp::common::center_of_geometry<iterator_type, PointMap, point_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points),
            point_map);

        return op(v, plane_type{point_type{point}, normal});
    };

    std::transform(std::forward<ExecutionPolicy>(policy), begin, end, out_begin, transform_op);
}

/**
 * @ingroup tangent-planes-estimation
 * @brief
 * Performs tangent plane estimation on each knn_map neighborhood of the given sequence
 * of elements using PCA. Results are stored in the out sequence through op.
 * @tparam ForwardIter1 Type of input sequence iterator
 * @tparam ForwardIter2 Type of output sequence iterator
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam KnnMap Callable type returning k neighborhood of input element
 * @tparam TransformOp Callable type returning an output element with parameters (input element,
 * tangent plane)
 * @tparam Plane Type of tangent plane
 * @param begin Iterator to start of sequence of elements
 * @param end Iterator to one past the end of sequence of elements
 * @param out_begin Start iterator of output sequence
 * @param point_map The point map property map
 * @param knn_map The callable object to query k nearest neighbors
 * @param op Transformation callable object taking an input element and its computed tangent plane
 * and returning an output sequence element
 */
template <
    class ForwardIter1,
    class ForwardIter2,
    class PointMap,
    class KnnMap,
    class TransformOp,
    class Plane = pcp::common::plane3d_t>
void estimate_tangent_planes(
    ForwardIter1 begin,
    ForwardIter1 end,
    ForwardIter2 out_begin,
    PointMap const& point_map,
    KnnMap&& knn_map,
    TransformOp&& op)
{
    using value_type  = typename std::iterator_traits<ForwardIter1>::value_type;
    using plane_type  = Plane;
    using point_type  = typename plane_type::point_type;
    using normal_type = typename plane_type::normal_type;

    static_assert(
        traits::is_point_map_v<PointMap, value_type>,
        "point_map must satisfy PointMap concept");
    static_assert(traits::is_knn_map_v<KnnMap, value_type>, "knn_map must satisfy KnnMap concept");
    static_assert(traits::is_plane_v<plane_type>, "Plane must satisfy Plane concept");
    static_assert(traits::is_point_v<point_type>, "Plane's point must satisfy Point concept");
    static_assert(traits::is_normal_v<normal_type>, "Plane's normal must satisfy Normal concept");

    using result_type = typename std::iterator_traits<ForwardIter2>::value_type;

    static_assert(
        std::is_invocable_r_v<result_type, TransformOp, value_type, plane_type>,
        "op must be callable by result = op(*begin, plane) where type of result is same as "
        "dereferencing out_begin decltype(*out_begin)");

    auto const transform_op = [&,
                               knn = std::forward<KnnMap>(knn_map),
                               op  = std::forward<TransformOp>(op)](value_type const& v) {
        auto const neighbor_points = knn(v);
        using iterator_type        = decltype(neighbor_points.begin());
        auto normal                = pcp::estimate_normal<iterator_type, PointMap, normal_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points),
            point_map);

        auto point = pcp::common::center_of_geometry<iterator_type, PointMap, point_type>(
            std::begin(neighbor_points),
            std::end(neighbor_points),
            point_map);

        return op(v, plane_type{point_type{point}, normal});
    };

    std::transform(begin, end, out_begin, transform_op);
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_ESTIMATE_TANGENT_PLANES_HPP