
#ifndef PCP_ALGORITHM_POINT_DENSITY_HPP
#define PCP_ALGORITHM_POINT_DENSITY_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include <pcp/algorithm/average_distance_to_neighbors.hpp>

namespace pcp {
namespace algorithm {

/**
 * @ingroup algorithm
 * @brief Computes every point's density in [begin,end]
 * 
 * @tparam Element element type (point)
 * @tparam ScalarType Numeric type, usually float/double
 * @tparam ForwardIterator
 * @tparam RangeSearchMap Type satisfying RangeSearchMap concept
 * @tparam PointMap type satisfying PointMap concept
 * @tparam KnnMap Type
 * @param begin Start iterator to the range of points
 * @param end End iterator to the range of points
 * @param point_map The point map property map
 * @param knn_map The knn map property map
 * @param range_search_map The range search map property map
 * @return
 */
template <
    class Element,
    class ScalarType,
    class ForwardIterator,
    class PointMap,
    class KnnMap,
    class RangeSearchMap>
std::vector<ScalarType> point_density(
    ForwardIterator begin,
    ForwardIterator end,
    PointMap point_map,
    KnnMap knn_map,
    RangeSearchMap range_search_map)
{
    using scalar_type  = ScalarType;
    using element_type = Element;

    std::vector<float> mean_distances =
        pcp::algorithm::average_distances_to_neighbors(begin, end, point_map, knn_map);
    scalar_type radius =
        std::reduce(std::execution::par, mean_distances.cbegin(), mean_distances.cend(), 0.f) /
        static_cast<float>(mean_distances.size());

    std::vector<scalar_type> density(std::distance(begin, end), 0.f);

    std::transform(std::execution::par, begin, end, density.begin(), [&](element_type const& p) {
        auto points_in_ball = range_search_map(p, radius);
        auto density        = points_in_ball.size();
        return density;
    });
    return density;
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_POINT_DENSITY_HPP
