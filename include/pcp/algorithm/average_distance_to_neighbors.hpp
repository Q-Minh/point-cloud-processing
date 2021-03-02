#ifndef PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
#define PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP

#include "pcp/common/norm.hpp"
#include "pcp/traits/point_map.hpp"

#include <algorithm>
#include <execution>

namespace pcp {
namespace algorithm {

template <
    class RandomAccessIter,
    class PointMap,
    class KnnMap,
    class ScalarType = typename std::invoke_result_t<
        PointMap,
        typename std::iterator_traits<RandomAccessIter>::value_type>::coordinate_type>
ScalarType average_distance_to_neighbors(
    RandomAccessIter begin,
    RandomAccessIter end,
    PointMap const& point_map,
    KnnMap const& knn_map)
{
    using scalar_type  = ScalarType;
    using element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using point_type   = std::invoke_result_t<PointMap, element_type>;

    auto const n = std::distance(begin, end);
    std::vector<scalar_type> mean_distances(n);
    std::transform(
        std::execution::par,
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

    float const sum =
        std::reduce(mean_distances.begin(), mean_distances.end(), static_cast<scalar_type>(0.));
    float const mu = sum / static_cast<scalar_type>(mean_distances.size());
    return mu;
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP