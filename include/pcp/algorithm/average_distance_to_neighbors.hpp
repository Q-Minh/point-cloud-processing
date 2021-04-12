#ifndef PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
#define PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/norm.hpp"
#include "pcp/traits/point_map.hpp"

#include <algorithm>
#include <execution>

namespace pcp {
namespace algorithm {

/**
 * @ingroup algorithm
 * @brief
 * Computes every point's mean distance to their k neighbors in [begin, end).
 * @tparam ExecutionPolicy The execution policy type
 * @tparam RandomAccessIter
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam KnnMap Type satisfying KnnMap concept
 * @tparam ScalarType Numeric type, usually float/double
 * @param begin Start iterator to the range of points
 * @param end End iterator to the range of points
 * @param point_map The point map property map
 * @param knn_map The knn map property map
 * @return Mean distances to neighbors for every point in [begin, end)
 */
template <
    class ExecutionPolicy,
    class RandomAccessIter,
    class PointMap,
    class KnnMap,
    class ScalarType = typename std::invoke_result_t<
        PointMap,
        typename std::iterator_traits<RandomAccessIter>::value_type>::coordinate_type>
std::vector<ScalarType> average_distances_to_neighbors(
    ExecutionPolicy&& policy,
    RandomAccessIter begin,
    RandomAccessIter end,
    PointMap const& point_map,
    KnnMap const& knn_map)
{
    using scalar_type  = ScalarType;
    using element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using point_type   = std::invoke_result_t<PointMap, element_type>;

    std::size_t const n = static_cast<std::size_t>(std::distance(begin, end));
    std::vector<scalar_type> mean_distances(n);
    std::transform(
        policy,
        begin,
        end,
        mean_distances.begin(),
        [&](element_type const& e) {
            auto const neighbors   = knn_map(e);
            point_type const pi    = point_map(e);
            scalar_type const mean = std::accumulate(
                neighbors.begin(),
                neighbors.end(),
                static_cast<scalar_type>(0.),
                [&](scalar_type const sum, element_type const& j) {
                    point_type const pj = point_map(j);
                    scalar_type const d = pcp::common::norm(pi - pj);
                    return sum + d;
                });

            return mean / static_cast<scalar_type>(neighbors.size());
        });

    return mean_distances;
}

/**
 * @ingroup algorithm
 * @brief
 * Computes the average of all mean distances to neighbors of all points.
 * @tparam ExecutionPolicy The execution policy type
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam KnnMap Type satisfying KnnMap concept
 * @tparam ScalarType Numeric type, usually float/double
 * @tparam RandomAccessIter
 * @param begin Start iterator to the range of points
 * @param end End iterator to the range of points
 * @param point_map The point map property map
 * @param knn_map The knn map property map
 * @return Average of all mean distances to neighbors of every point in [begin, end)
 */
template <
    class ExecutionPolicy,
    class RandomAccessIter,
    class PointMap,
    class KnnMap,
    class ScalarType = typename std::invoke_result_t<
        PointMap,
        typename std::iterator_traits<RandomAccessIter>::value_type>::coordinate_type>
ScalarType average_distance_to_neighbors(
    ExecutionPolicy&& policy,
    RandomAccessIter begin,
    RandomAccessIter end,
    PointMap const& point_map,
    KnnMap const& knn_map)
{
    using scalar_type  = ScalarType;
    using element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using point_type   = std::invoke_result_t<PointMap, element_type>;

    std::vector<scalar_type> mean_distances =
        average_distances_to_neighbors(policy, begin, end, point_map, knn_map);

    float const sum =
        std::reduce(mean_distances.begin(), mean_distances.end(), static_cast<scalar_type>(0.));
    float const mu = sum / static_cast<scalar_type>(mean_distances.size());
    return mu;
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
