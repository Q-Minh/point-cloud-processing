#ifndef PCP_ALGORITHM_HPP
#define PCP_ALGORITHM_HPP

/**
 * @file 
 * @defgroup algorithm algorithm
 * The algorithm module
 */

/**
 * @defgroup normals-estimation "Normals Estimation"
 * Normals estimation algorithms.
 * @ingroup algorithm
 */

/**
 * @defgroup tangent-planes-estimation "Tangent Plane Estimation"
 * Tangent plane estimation algorithms.
 * @ingroup algorithm
 */

/**
 * @defgroup isosurface-extraction "Isosurface Extraction"
 * Isosurface extraction algorithms.
 * @ingroup algorithm
 */

/**
 * @defgroup point-cloud-simplification "Point cloud simplification"
 * Point cloud simplification algorithms.
 * @ingroup algorithm
 */

#include "average_distance_to_neighbors.hpp"
#include "common.hpp"
#include "covariance.hpp"
#include "edge_aware_upsampling.hpp"
#include "estimate_normals.hpp"
#include "estimate_tangent_planes.hpp"
#include "hierarchy_simplification.hpp"
#include "random_simplification.hpp"
#include "resample_away_from_edges.hpp"
#include "surface_nets.hpp"
#include "wlop.hpp"

#endif // PCP_ALGORITHM_HPP