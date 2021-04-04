#ifndef PCP_ALGORITHM_EDGE_AWARE_UPSAMPLING_HPP
#define PCP_ALGORITHM_EDGE_AWARE_UPSAMPLING_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "average_distance_to_neighbors.hpp"
#include "pcp/common/sphere.hpp"
#include "pcp/common/vector3d.hpp"
#include "pcp/kdtree/linked_kdtree.hpp"
#include "pcp/traits/normal_map.hpp"
#include "pcp/traits/output_iterator_traits.hpp"
#include "pcp/traits/point_map.hpp"
#include "resample_away_from_edges.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cstddef>
#include <execution>
#include <random>
#include <vector>

namespace pcp {
namespace algorithm {
namespace ear {

struct params_t
{
    std::size_t resampling_away_from_edges_iteration_count =
        3u; ///< Number of iterations of the preprocessing step for pushing points away from edges
};


} // namespace ear

template <class RandomAccessIter, class OutputIter, class PointMap, class NormalMap>
OutputIter edge_aware_upsampling(
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    ear::params_t const& params)
{
    
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_EDGE_AWARE_UPSAMPLING_HPP