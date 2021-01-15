#ifndef PCP_COMMON_HPP
#define PCP_COMMON_HPP

/**
 * @file
 * @defgroup common common
 * The common module
 */

/**
 * @defgroup geometric-primitives "Geometric Primitives"
 * Geometric primitive types
 * @ingroup common
 */

/**
 * @defgroup intersection-tests "Intersection Tests"
 * Geometric intersection tests.
 * @ingroup common
 */

/**
 * @defgroup common-vector3 "3D Vectors"
 * Common 3D Vector Operations and Types
 * @ingroup common
 */

#include "axis_aligned_bounding_box.hpp"
#include "intersections.hpp"
#include "mesh_triangle.hpp"
#include "norm.hpp"
#include "normals/normal.hpp"
#include "normals/normal_estimation.hpp"
#include "plane3d.hpp"
#include "points/point.hpp"
#include "points/point_view.hpp"
#include "points/vertex.hpp"
#include "regular_grid3d.hpp"
#include "sphere.hpp"
#include "timer.hpp"
#include "vector3d.hpp"
#include "vector3d_queries.hpp"

#endif // PCP_COMMON_HPP