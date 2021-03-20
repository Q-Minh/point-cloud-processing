//#ifndef PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
//#define PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
//
///**
// * @file
// * @ingroup algorithm
// */
//
//
//#include <pcp/algorithm/average_distance_to_neighbors.hpp>
//#include <pcp/algorithm/hierarchy_simplification.hpp>
//#include <pcp/algorithm/random_simplification.hpp>
//#include <pcp/algorithm/wlop.hpp>
//#include <pcp/common/normals/normal.hpp>
//#include <pcp/common/timer.hpp>
//#include <pcp/io/ply.hpp>
//
//
//namespace pcp {
//namespace algorithm {
//
///**
// * @ingroup algorithm
// * @brief
// * Computes every point's mean distance to their k neighbors in [begin, end).
// * @tparam RandomAccessIter
// * @tparam PointMap Type satisfying PointMap concept
// * @tparam KnnMap Type satisfying KnnMap concept
// * @tparam ScalarType Numeric type, usually float/double
// * @param begin Start iterator to the range of points
// * @param end End iterator to the range of points
// * @param point_map The point map property map
// * @param knn_map The knn map property map
// * @return Mean distances to neighbors for every point in [begin, end)
// */
//template <
//    class RandomAccessIter,
//    class PointMap,
//    class KnnMap,
//    class RangeSearchMap,
//    class ScalarType = typename std::invoke_result_t<
//        PointMap,
//        typename std::iterator_traits<RandomAccessIter>::value_type>::coordinate_type>
//std::vector<ScalarType> point_density(
//    RandomAccessIter begin,
//    RandomAccessIter end,
//    PointMap const& point_map,
//    KnnMap const& knn_map,
//    RangeSearchMap const& range_search_map)
//{
//    using scalar_type  = ScalarType;
//    using element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
//    using point_type   = std::invoke_result_t<PointMap, element_type>;
//
//    std::vector<scalar_type> mean_distances =
//        pcp::algorithm::average_distances_to_neighbors(begin, end, point_map, knn_map);
//
//    scalar_type radius =
//        std::reduce(std::execution::par, mean_distances.cbegin(), mean_distances.cend(), 0.f) /
//        static_cast<float>(mean_distances.size());
//
//    std::vector<scalar_type> density;
//    std::transform(
//        std::execution::par,
//        mean_distances.cbegin(),
//        mean_distances.cend(),
//        density.begin(),
//        [&](element_type const& p) {
//            auto points_in_ball = range_search_map(p, radius);
//            auto density        = points_in_ball.size();
//            return density;
//        });
//}
//
//} // namespace algorithm
//} // namespace pcp
//
//#endif // PCP_ALGORITHM_POINT_DENSITY_HPP