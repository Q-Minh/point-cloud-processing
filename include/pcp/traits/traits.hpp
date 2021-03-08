#ifndef PCP_TRAITS_HPP
#define PCP_TRAITS_HPP

/**
 * @file
 * @defgroup traits traits
 * The traits module
 */

/**
 * @defgroup traits-geometry "Geometry Type Traits"
 * Type traits for geometry-related types. Defines geometric concepts.
 * @ingroup traits
 */

/**
 * @defgroup traits-spatial-query "Spatial Query Type Traits"
 * Type traits for common spatial query callable types. Defines spatial query concepts.
 * @ingroup traits
 */

/**
 * @defgroup traits-graph "Graph Type Traits"
 * Type traits for graph-related types. Defines graph concepts.
 * @ingroup traits
 */

/**
 * @defgroup traits-property-maps "Property Map Type Traits"
 * Type traits for common property maps used in the API.
 * @ingroup traits
 */

#include "coordinate_map.hpp"
#include "function_traits.hpp"
#include "graph_traits.hpp"
#include "graph_vertex_traits.hpp"
#include "identity_map.hpp"
#include "index_map.hpp"
#include "knn_map.hpp"
#include "normal_map.hpp"
#include "normal_traits.hpp"
#include "output_iterator_traits.hpp"
#include "plane_traits.hpp"
#include "point_map.hpp"
#include "point_traits.hpp"
#include "property_map_traits.hpp"
#include "range_neighbor_map.hpp"
#include "range_traits.hpp"
#include "signed_distance_map.hpp"
#include "triangle_traits.hpp"
#include "vector3d_traits.hpp"

#endif // PCP_TRAITS_HPP