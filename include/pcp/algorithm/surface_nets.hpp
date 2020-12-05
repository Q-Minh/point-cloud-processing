#pragma once

#include <algorithm>
#include <execution>
#include <mutex>
#include <pcp/common/axis_aligned_bounding_box.hpp>
#include <pcp/common/mesh_triangle.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/regular_grid3d.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/traits/function_traits.hpp>
#include <pcp/traits/point_traits.hpp>
#include <pcp/traits/triangle_traits.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pcp {
namespace algorithm {
namespace isosurface {

template <
    class Point,
    class Scalar = typename Point::component_type,
    class Grid   = common::regular_grid3d_t<Scalar>>
Point get_world_point_of(std::size_t i, std::size_t j, std::size_t k, Grid const& grid)
{
    return Point{
        grid.x + static_cast<Scalar>(i) * grid.dx,
        grid.y + static_cast<Scalar>(j) * grid.dy,
        grid.z + static_cast<Scalar>(k) * grid.dz};
}

template <
    class Point,
    class Scalar = typename Point::component_type,
    class Grid   = common::regular_grid3d_t<Scalar>>
auto get_grid_point_of(Point const& p, Grid const& grid)
    -> std::tuple<std::size_t, std::size_t, std::size_t>
{
    // if x = grid.x + i*grid.dx, then i = (x - grid.x) / grid.dx
    return std::make_tuple(
        static_cast<std::size_t>((p.x() - grid.x) / grid.dx),
        static_cast<std::size_t>((p.y() - grid.y) / grid.dy),
        static_cast<std::size_t>((p.z() - grid.z) / grid.dz));
}

/**
 * @brief Mapping from 3d grid coordinates to 1d
 * @tparam Grid
 * @param i
 * @param j
 * @param k
 * @param grid
 * @return 1d index of grid coordinates i,j,k
 */
template <class Grid>
std::size_t get_active_cube_index(std::size_t i, std::size_t j, std::size_t k, Grid const& grid)
{
    return i + (j * grid.sx) + (k * grid.sx * grid.sy);
}

/**
 * @brief mapping from 1d index to 3d grid coordinates
 * @tparam Grid
 * @param active_cube_index
 * @param grid
 * @return i,j,k coordinates in grid frame
 */
template <class Grid>
auto get_ijk_from_idx(std::size_t active_cube_index, Grid const& grid)
    -> std::tuple<std::size_t, std::size_t, std::size_t>
{
    std::size_t i = (active_cube_index) % grid.sx;
    std::size_t j = (active_cube_index / grid.sx) % grid.sy;
    std::size_t k = (active_cube_index) / (grid.sx * grid.sy);
    return std::make_tuple(i, j, k);
}

/**
 * @brief Gets the positions of the voxel's corners in a regular grid frame
 * @tparam Point
 * @tparam Scalar
 * @param i
 * @param j
 * @param k
 * @return
 */
template <class Point, class Scalar = typename Point::component_type>
std::array<Point, 8> get_voxel_corner_grid_positions(std::size_t i, std::size_t j, std::size_t k)
{
    using point_type  = Point;
    using scalar_type = Scalar;
    auto const ifloat = static_cast<scalar_type>(i);
    auto const jfloat = static_cast<scalar_type>(j);
    auto const kfloat = static_cast<scalar_type>(k);
    return std::array<point_type, 8>{
        point_type{ifloat, jfloat, kfloat},
        point_type{ifloat + 1.f, jfloat, kfloat},
        point_type{ifloat + 1.f, jfloat + 1.f, kfloat},
        point_type{ifloat, jfloat + 1.f, kfloat},
        point_type{ifloat, jfloat, kfloat + 1.f},
        point_type{ifloat + 1.f, jfloat, kfloat + 1.f},
        point_type{ifloat + 1.f, jfloat + 1.f, kfloat + 1.f},
        point_type{ifloat, jfloat + 1.f, kfloat + 1.f}};
}

/**
 * @brief Get the voxel's corners' positions in world frame
 * @tparam Point
 * @tparam Scalar
 * @tparam Grid
 * @param i
 * @param j
 * @param k
 * @param grid
 * @return
 */
template <
    class Point,
    class Scalar = typename Point::component_type,
    class Grid   = common::regular_grid3d_t<Scalar>>
std::array<Point, 8>
get_voxel_corner_world_positions(std::size_t i, std::size_t j, std::size_t k, Grid const& grid)
{
    using point_type  = Point;
    using scalar_type = Scalar;
    return std::array<point_type, 8>{
        point_type{
            grid.x + static_cast<scalar_type>(i) * grid.dx,
            grid.y + static_cast<scalar_type>(j) * grid.dy,
            grid.z + static_cast<scalar_type>(k) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i + 1) * grid.dx,
            grid.y + static_cast<scalar_type>(j) * grid.dy,
            grid.z + static_cast<scalar_type>(k) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i + 1) * grid.dx,
            grid.y + static_cast<scalar_type>(j + 1) * grid.dy,
            grid.z + static_cast<scalar_type>(k) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i) * grid.dx,
            grid.y + static_cast<scalar_type>(j + 1) * grid.dy,
            grid.z + static_cast<scalar_type>(k) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i) * grid.dx,
            grid.y + static_cast<scalar_type>(j) * grid.dy,
            grid.z + static_cast<scalar_type>(k + 1) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i + 1) * grid.dx,
            grid.y + static_cast<scalar_type>(j) * grid.dy,
            grid.z + static_cast<scalar_type>(k + 1) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i + 1) * grid.dx,
            grid.y + static_cast<scalar_type>(j + 1) * grid.dy,
            grid.z + static_cast<scalar_type>(k + 1) * grid.dz},
        point_type{
            grid.x + static_cast<scalar_type>(i) * grid.dx,
            grid.y + static_cast<scalar_type>(j + 1) * grid.dy,
            grid.z + static_cast<scalar_type>(k + 1) * grid.dz}};
}

/**
 * @brief Get the value of the scalar function f(x,y,z) at the voxel's corners
 * @tparam Func
 * @tparam Point
 * @tparam Scalar
 * @param voxel_corner_world_positions
 * @param f
 * @return
 */
template <class Func, class Point, class Scalar = typename Point::component_type>
std::array<Scalar, 8>
get_voxel_corner_values(std::array<Point, 8> const& voxel_corner_world_positions, Func& f)
{
    using scalar_type = Scalar;
    return std::array<scalar_type, 8>{
        f(voxel_corner_world_positions[0].x(),
          voxel_corner_world_positions[0].y(),
          voxel_corner_world_positions[0].z()),
        f(voxel_corner_world_positions[1].x(),
          voxel_corner_world_positions[1].y(),
          voxel_corner_world_positions[1].z()),
        f(voxel_corner_world_positions[2].x(),
          voxel_corner_world_positions[2].y(),
          voxel_corner_world_positions[2].z()),
        f(voxel_corner_world_positions[3].x(),
          voxel_corner_world_positions[3].y(),
          voxel_corner_world_positions[3].z()),
        f(voxel_corner_world_positions[4].x(),
          voxel_corner_world_positions[4].y(),
          voxel_corner_world_positions[4].z()),
        f(voxel_corner_world_positions[5].x(),
          voxel_corner_world_positions[5].y(),
          voxel_corner_world_positions[5].z()),
        f(voxel_corner_world_positions[6].x(),
          voxel_corner_world_positions[6].y(),
          voxel_corner_world_positions[6].z()),
        f(voxel_corner_world_positions[7].x(),
          voxel_corner_world_positions[7].y(),
          voxel_corner_world_positions[7].z())};
}

/**
 * @brief Returns a boolean array indicating which edges are bipolar (true) and which are not
 * (false)
 * @tparam Scalar
 * @param voxel_corner_values
 * @param isovalue
 * @param edges
 * @return
 */
template <class Scalar>
std::array<bool, 12> get_edge_bipolarity_array(
    std::array<Scalar, 8> const& voxel_corner_values,
    Scalar isovalue,
    std::uint8_t const edges[12][2])
{
    using scalar_type             = Scalar;
    auto const is_scalar_positive = [&isovalue](scalar_type scalar) -> bool {
        return scalar >= isovalue;
    };

    auto const are_edge_scalars_bipolar =
        [&is_scalar_positive](scalar_type scalar1, scalar_type scalar2) -> bool {
        return is_scalar_positive(scalar1) != is_scalar_positive(scalar2);
    };

    std::array<bool, 12> const edge_bipolarity_array = {
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[0][0]],
            voxel_corner_values[edges[0][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[1][0]],
            voxel_corner_values[edges[1][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[2][0]],
            voxel_corner_values[edges[2][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[3][0]],
            voxel_corner_values[edges[3][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[4][0]],
            voxel_corner_values[edges[4][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[5][0]],
            voxel_corner_values[edges[5][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[6][0]],
            voxel_corner_values[edges[6][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[7][0]],
            voxel_corner_values[edges[7][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[8][0]],
            voxel_corner_values[edges[8][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[9][0]],
            voxel_corner_values[edges[9][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[10][0]],
            voxel_corner_values[edges[10][1]]),
        are_edge_scalars_bipolar(
            voxel_corner_values[edges[11][0]],
            voxel_corner_values[edges[11][1]])};

    return edge_bipolarity_array;
}

/**
 * @brief Check if a cube intersects the isosurface or not
 * @param edge_bipolarity_array
 * @return
 */
bool get_is_cube_active(std::array<bool, 12> const& edge_bipolarity_array)
{
    // clang-format off
	// an active voxel must have at least one bipolar edge
	bool const is_voxel_active = 
        edge_bipolarity_array[0] ||
		edge_bipolarity_array[1] ||
		edge_bipolarity_array[2] ||
		edge_bipolarity_array[3] ||
		edge_bipolarity_array[4] ||
		edge_bipolarity_array[5] ||
		edge_bipolarity_array[6] ||
		edge_bipolarity_array[7] ||
		edge_bipolarity_array[8] ||
		edge_bipolarity_array[9] ||
		edge_bipolarity_array[10] ||
		edge_bipolarity_array[11];
    // clang-format on

    return is_voxel_active;
}

/**
 * @brief Returns 3 adjacent cubes of the specified edge as 3 triples of (i,j,k) coords of adjacent
 * cubes
 * @param i The i coordinate of the current cube in grid frame
 * @param j The j coordinate of the current cube in grid frame
 * @param k The k coordinate of the current cube in grid frame
 * @param edge The edge shared by the current cube and its returned 3 adjacent cubes
 * @param adjacent_cubes_of_edges Mapping of edge to adjacent cube grid frame
 * @return
 */
auto get_adjacent_cubes_of_edge(
    std::size_t i,
    std::size_t j,
    std::size_t k,
    std::size_t edge,
    std::int8_t const adjacent_cubes_of_edges[12][3][3])
    -> std::array<std::array<std::size_t, 3>, 3>
{
    std::array<std::array<std::size_t, 3>, 3> adjacent_cubes;
    adjacent_cubes[0] = {
        i + adjacent_cubes_of_edges[edge][0][0],
        j + adjacent_cubes_of_edges[edge][0][1],
        k + adjacent_cubes_of_edges[edge][0][2]};
    adjacent_cubes[1] = {
        i + adjacent_cubes_of_edges[edge][1][0],
        j + adjacent_cubes_of_edges[edge][1][1],
        k + adjacent_cubes_of_edges[edge][1][2]};
    adjacent_cubes[2] = {
        i + adjacent_cubes_of_edges[edge][2][0],
        j + adjacent_cubes_of_edges[edge][2][1],
        k + adjacent_cubes_of_edges[edge][2][2]};
    return adjacent_cubes;
}

/**
 * @brief
 * Implements the naive surface nets algorithm which approximates the isosurface
 * of the given implicit function at the given isovalue in the given regular grid
 * by a triangle mesh. This overload marches over the whole regular grid.
 * @tparam Func
 * @tparam Scalar
 * @tparam Point
 * @tparam SharedVertexMeshTriangle
 * @tparam ExecutionPolicy
 * @param policy
 * @param f
 * @param grid
 * @param isovalue
 * @return
 */
template <
    class ExecutionPolicy,
    class Func,
    class Scalar,
    class Point                    = pcp::point_t,
    class SharedVertexMeshTriangle = pcp::common::shared_vertex_mesh_triangle<std::uint32_t>>
auto surface_nets(
    ExecutionPolicy&& policy,
    Func&& f,
    common::regular_grid3d_t<Scalar> const& grid,
    Scalar const isovalue = static_cast<Scalar>(0))
    -> std::pair<std::vector<Point>, std::vector<SharedVertexMeshTriangle>>
{
    static_assert(
        traits::is_3d_scalar_function_v<Func, Scalar>,
        "Func must be 3d scalar function Scalar Func(Scalar, Scalar, Scalar)");
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(
        traits::is_shared_vertex_mesh_triangle_v<SharedVertexMeshTriangle>,
        "Triangle must satisfy SharedVertexMeshTriangle concept");

    using point_type    = Point;
    using triangle_type = SharedVertexMeshTriangle;
    using index_type    = typename triangle_type::index_type;
    using scalar_type   = Scalar;
    using grid_type     = common::regular_grid3d_t<scalar_type>;
    using function_type = Func;

    // the shared vertex mesh
    std::vector<point_type> vertices;
    std::vector<triangle_type> triangles;

    // bounding box of the mesh in coordinate frame of the mesh
    pcp::axis_aligned_bounding_box_t<point_type> mesh_aabb = {
        get_world_point_of<point_type, scalar_type, grid_type>(0u, 0u, 0u, grid),
        get_world_point_of<point_type, scalar_type, grid_type>(grid.sx, grid.sy, grid.sz, grid)};

    // mapping from active cube indices to vertex indices of the generated mesh
    std::unordered_map<std::size_t, std::uint64_t> active_cube_to_vertex_index_map{};

    bool const is_x_longest_dimension = grid.sx > grid.sy && grid.sx > grid.sz;
    bool const is_y_longest_dimension = grid.sy > grid.sx && grid.sy > grid.sz;

    std::size_t const longest_dimension_size =
        is_x_longest_dimension ? grid.sx : is_y_longest_dimension ? grid.sy : grid.sz;

    std::vector<std::size_t> steps_along_longest_dimension(longest_dimension_size, 0u);
    std::iota(steps_along_longest_dimension.begin(), steps_along_longest_dimension.end(), 0u);

    // synchronization object to prevent concurrently modifying the building mesh
    std::mutex sync;

    std::for_each(
        policy,
        steps_along_longest_dimension.cbegin(),
        steps_along_longest_dimension.cend(),
        [&](std::size_t k) {
            for (std::size_t j = 0; j < grid.sy; ++j)
            {
                for (std::size_t i = 0; i < grid.sx; ++i)
                {
                    if (is_x_longest_dimension)
                    {
                        std::swap(i, k);
                    }

                    if (is_y_longest_dimension)
                    {
                        std::swap(j, k);
                    }

                    // coordinates of voxel corners in voxel grid coordinate frame
                    std::array<point_type, 8> const voxel_corner_grid_positions =
                        get_voxel_corner_grid_positions<point_type, scalar_type>(i, j, k);

                    // coordinates of voxel corners in the mesh's coordinate frame
                    std::array<point_type, 8> const voxel_corner_world_positions =
                        get_voxel_corner_world_positions<point_type, scalar_type, grid_type>(
                            i,
                            j,
                            k,
                            grid);

                    // scalar values of the implicit function evaluated at cube vertices (voxel
                    // corners)
                    std::array<scalar_type, 8> const voxel_corner_values =
                        get_voxel_corner_values<function_type, point_type, scalar_type>(
                            voxel_corner_world_positions,
                            std::forward<function_type>(f));

                    // the edges provide indices to the corresponding current cube's vertices (voxel
                    // corners)
                    std::uint8_t constexpr edges[12][2] = {
                        {0u, 1u},
                        {1u, 2u},
                        {2u, 3u},
                        {3u, 0u},
                        {4u, 5u},
                        {5u, 6u},
                        {6u, 7u},
                        {7u, 4u},
                        {0u, 4u},
                        {1u, 5u},
                        {2u, 6u},
                        {3u, 7u}};

                    std::array<bool, 12> const edge_bipolarity_array =
                        get_edge_bipolarity_array<scalar_type>(
                            voxel_corner_values,
                            isovalue,
                            edges);

                    // an active voxel must have at least one bipolar edge
                    bool const is_voxel_active = get_is_cube_active(edge_bipolarity_array);

                    // cubes that are not active do not generate mesh vertices
                    if (!is_voxel_active)
                        continue;

                    // store all edge intersection points with the implicit surface in voxel grid
                    // coordinates
                    std::vector<point_type> edge_intersection_points;

                    // visit every bipolar edge
                    for (std::size_t e = 0; e < 12; ++e)
                    {
                        if (!edge_bipolarity_array[e])
                            continue;

                        // get points p1, p2 of the edge e in grid coordinates
                        auto const p1 = voxel_corner_grid_positions[edges[e][0]];
                        auto const p2 = voxel_corner_grid_positions[edges[e][1]];

                        // get value of the implicit function at edge vertices
                        auto const s1 = voxel_corner_values[edges[e][0]];
                        auto const s2 = voxel_corner_values[edges[e][1]];

                        // perform linear interpolation using implicit function
                        // values at vertices
                        auto const t = (isovalue - s1) / (s2 - s1);
                        edge_intersection_points.push_back(point_type{p1 + t * (p2 - p1)});
                    }

                    point_type const geometric_center_of_edge_intersection_points =
                        pcp::common::center_of_geometry(
                            edge_intersection_points.cbegin(),
                            edge_intersection_points.cend());

                    point_type const mesh_vertex = {
                        mesh_aabb.min.x() +
                            (mesh_aabb.max.x() - mesh_aabb.min.x()) *
                                (geometric_center_of_edge_intersection_points.x() - 0.f) /
                                (static_cast<scalar_type>(grid.sx) - 0.f),
                        mesh_aabb.min.y() +
                            (mesh_aabb.max.y() - mesh_aabb.min.y()) *
                                (geometric_center_of_edge_intersection_points.y() - 0.f) /
                                (static_cast<scalar_type>(grid.sy) - 0.f),
                        mesh_aabb.min.z() +
                            (mesh_aabb.max.z() - mesh_aabb.min.z()) *
                                (geometric_center_of_edge_intersection_points.z() - 0.f) /
                                (static_cast<scalar_type>(grid.sz) - 0.f),
                    };

                    std::size_t const active_cube_index =
                        get_active_cube_index<grid_type>(i, j, k, grid);

                    std::lock_guard<std::mutex> lock(sync);
                    auto const vertex_index                            = vertices.size();
                    active_cube_to_vertex_index_map[active_cube_index] = vertex_index;

                    vertices.push_back(mesh_vertex);
                }
            }
        });

    using key_value_type = typename decltype(active_cube_to_vertex_index_map)::value_type;

    std::for_each(
        policy,
        active_cube_to_vertex_index_map.cbegin(),
        active_cube_to_vertex_index_map.cend(),
        [&](key_value_type const& key_value) {
            auto const active_cube_index = key_value.first;
            auto const vertex_index      = key_value.second;

            auto const is_lower_boundary_cube = [](auto i, auto j, auto k) {
                return (i == 0 || j == 0 || k == 0);
            };

            auto const [i, j, k] = get_ijk_from_idx<grid_type>(active_cube_index, grid);

            if (is_lower_boundary_cube(i, j, k))
                return;

            // clang-format off
            std::size_t const neighbor_grid_positions[6][3] = {
                {i - 1, j,     k},
                {i - 1, j - 1, k},
                {i,     j - 1, k},
                {i,     j - 1, k - 1},
                {i,     j,     k - 1},
                {i - 1, j,     k - 1}};
            // clang-format on

            point_type const voxel_corners_of_interest[4] = {
                // vertex 0,
                get_world_point_of<point_type, scalar_type, grid_type>(i, j, k, grid),
                // vertex 4
                get_world_point_of<point_type, scalar_type, grid_type>(i, j, k + 1, grid),
                // vertex 3
                get_world_point_of<point_type, scalar_type, grid_type>(i, j + 1, k, grid),
                // vertex 1
                get_world_point_of<point_type, scalar_type, grid_type>(i + 1, j, k, grid)};

            scalar_type const edge_scalar_values[3][2] = {// directed edge (0,4)
                                                          {f(voxel_corners_of_interest[0].x(),
                                                             voxel_corners_of_interest[0].y(),
                                                             voxel_corners_of_interest[0].z()),
                                                           f(voxel_corners_of_interest[1].x(),
                                                             voxel_corners_of_interest[1].y(),
                                                             voxel_corners_of_interest[1].z())},
                                                          // directed edge (3,0)
                                                          {f(voxel_corners_of_interest[2].x(),
                                                             voxel_corners_of_interest[2].y(),
                                                             voxel_corners_of_interest[2].z()),
                                                           f(voxel_corners_of_interest[0].x(),
                                                             voxel_corners_of_interest[0].y(),
                                                             voxel_corners_of_interest[0].z())},
                                                          // directed edge (0,1)
                                                          {f(voxel_corners_of_interest[0].x(),
                                                             voxel_corners_of_interest[0].y(),
                                                             voxel_corners_of_interest[0].z()),
                                                           f(voxel_corners_of_interest[3].x(),
                                                             voxel_corners_of_interest[3].y(),
                                                             voxel_corners_of_interest[3].z())}};

            std::size_t const quad_neighbors[3][3] = {{0, 1, 2}, {0, 5, 4}, {2, 3, 4}};

            std::array<std::size_t, 3> const quad_neighbor_orders[2] = {{0, 1, 2}, {2, 1, 0}};

            for (std::size_t idx = 0; idx < 3; ++idx)
            {
                auto const neighbor1 = get_active_cube_index<grid_type>(
                    neighbor_grid_positions[quad_neighbors[idx][0]][0],
                    neighbor_grid_positions[quad_neighbors[idx][0]][1],
                    neighbor_grid_positions[quad_neighbors[idx][0]][2],
                    grid);

                auto const neighbor2 = get_active_cube_index(
                    neighbor_grid_positions[quad_neighbors[idx][1]][0],
                    neighbor_grid_positions[quad_neighbors[idx][1]][1],
                    neighbor_grid_positions[quad_neighbors[idx][1]][2],
                    grid);

                auto const neighbor3 = get_active_cube_index(
                    neighbor_grid_positions[quad_neighbors[idx][2]][0],
                    neighbor_grid_positions[quad_neighbors[idx][2]][1],
                    neighbor_grid_positions[quad_neighbors[idx][2]][2],
                    grid);

                if (active_cube_to_vertex_index_map.count(neighbor1) == 0 ||
                    active_cube_to_vertex_index_map.count(neighbor2) == 0 ||
                    active_cube_to_vertex_index_map.count(neighbor3) == 0)
                    continue;

                std::size_t const neighbor_vertices[3] = {
                    active_cube_to_vertex_index_map[neighbor1],
                    active_cube_to_vertex_index_map[neighbor2],
                    active_cube_to_vertex_index_map[neighbor3]};

                auto const& neighbor_vertices_order =
                    edge_scalar_values[idx][1] > edge_scalar_values[idx][0] ?
                        quad_neighbor_orders[0] :
                        quad_neighbor_orders[1];

                auto const v0 = vertex_index;
                auto const v1 = neighbor_vertices[neighbor_vertices_order[0]];
                auto const v2 = neighbor_vertices[neighbor_vertices_order[1]];
                auto const v3 = neighbor_vertices[neighbor_vertices_order[2]];

                std::lock_guard<std::mutex> lock(sync);
                triangles.push_back(triangle_type{
                    static_cast<index_type>(v0),
                    static_cast<index_type>(v1),
                    static_cast<index_type>(v2)});
                triangles.push_back(triangle_type{
                    static_cast<index_type>(v0),
                    static_cast<index_type>(v2),
                    static_cast<index_type>(v3)});
            }
        });

    return {vertices, triangles};
}

/**
 * @brief
 * Implements naive surface nets optimized for cases where
 * you know approximately in which neighborhood there is
 * a voxel that intersects the surface. This surface_nets
 * overload takes a "hint", which is a point is 3d space.
 *
 * This modified algorithm starts by performing a breadth
 * first search of the voxel grid starting from that 3d
 * point in space, looking at neighboring cubes as neighbor
 * vertices in a graph.
 *
 * The breadth first search stops when
 * the first active cube is found (the first cube intersecting
 * the surface). This means that if the given hint was not
 * too far off the surface, then the breadth first search
 * will terminate very quickly. If the hint was very far
 * from the surface, the breadth first search will be very
 * slow.
 *
 * Once the breadth first search has found an active cube,
 * we can now start a new breadth first search, but now,
 * we only consider neighboring cubes that share a bipolar
 * edge with the current cube. As such, we will only ever
 * iterate on active cubes. This means that iteration
 * in this second breadth first search will only ever
 * iterate on vertices of the resulting mesh. The time
 * complexity for the search is then linear in the number of
 * resulting vertices of the mesh. This is a lot better
 * than iterating over every single voxel in the regular
 * grid, which can get very long very fast.
 *
 * Finally, we triangulate the surface in the same way
 * as before.
 *
 * Another issue with this version of surface nets is that
 * if the surface has different separated regions in the
 * grid, then we will only mesh the region in which the
 * active cube found by breadth first search is part of.
 * For example, let's say the isosurface described by the
 * implicit function is actually two separate sphere, one
 * to the left of the grid, and the other to the right,
 * both not touching each other. If our breadth first search
 * finds an active cube in the left sphere, then only the
 * left sphere will be meshed. If the found active cube is
 * on the right sphere, then only the right sphere will be
 * meshed.
 * @tparam ExecutionPolicy
 * @tparam Func
 * @tparam Scalar
 * @tparam Point
 * @tparam SharedVertexMeshTriangle
 * @param policy
 * @param f
 * @param grid
 * @param hint
 * @param isovalue
 * @param breadth_first_search_queue_max_size Maximum size of the bfs queue before we decide to just
 * revert to marching over the whole grid
 * @return
 */
template <
    class ExecutionPolicy,
    class Func,
    class Scalar,
    class Point                    = pcp::point_t,
    class SharedVertexMeshTriangle = pcp::common::shared_vertex_mesh_triangle<std::uint32_t>>
auto surface_nets(
    ExecutionPolicy&& policy,
    Func&& f,
    common::regular_grid3d_t<Scalar> const& grid,
    Point const& hint,
    Scalar const isovalue                           = static_cast<Scalar>(0),
    std::size_t breadth_first_search_queue_max_size = 32'768u)
    -> std::pair<std::vector<Point>, std::vector<SharedVertexMeshTriangle>>
{
    static_assert(
        traits::is_3d_scalar_function_v<Func, Scalar>,
        "Func must be 3d scalar function Scalar Func(Scalar, Scalar, Scalar)");
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(
        traits::is_shared_vertex_mesh_triangle_v<SharedVertexMeshTriangle>,
        "Triangle must satisfy SharedVertexMeshTriangle concept");

    using point_type    = Point;
    using triangle_type = SharedVertexMeshTriangle;
    using index_type    = typename triangle_type::index_type;
    using scalar_type   = Scalar;
    using grid_type     = common::regular_grid3d_t<scalar_type>;
    using function_type = Func;

    // the shared vertex mesh
    std::vector<point_type> vertices;
    std::vector<triangle_type> triangles;

    // bounding box of the mesh in coordinate frame of the mesh
    pcp::axis_aligned_bounding_box_t<point_type> mesh_aabb = {
        {grid.x, grid.y, grid.z},
        {grid.x + static_cast<scalar_type>(grid.sx) * grid.dx,
         grid.y + static_cast<scalar_type>(grid.sy) * grid.dy,
         grid.z + static_cast<scalar_type>(grid.sz) * grid.dz}};

    std::uint8_t constexpr edges[12][2] = {
        {0u, 1u},
        {1u, 2u},
        {2u, 3u},
        {3u, 0u},
        {4u, 5u},
        {5u, 6u},
        {6u, 7u},
        {7u, 4u},
        {0u, 4u},
        {1u, 5u},
        {2u, 6u},
        {3u, 7u}};

    // use hint as starting point of a breadth first search for all active cubes
    struct active_cube_t
    {
        std::size_t idx;
        std::array<scalar_type, 8> voxel_corner_values;
        std::uint64_t vertex_idx;
    };

    auto const make_cube = [&grid, &f](std::size_t i, std::size_t j, std::size_t k) {
        active_cube_t cube{};
        cube.idx = get_active_cube_index<grid_type>(i, j, k, grid);
        auto const voxel_corner_world_positions =
            get_voxel_corner_world_positions<point_type, scalar_type, grid_type>(i, j, k, grid);
        cube.voxel_corner_values = get_voxel_corner_values<function_type, point_type, scalar_type>(
            voxel_corner_world_positions,
            std::forward<function_type>(f));
        return cube;
    };

    // perform breadth first search starting from the hint in grid coordinates
    auto const [ihint, jhint, khint] =
        get_grid_point_of<point_type, scalar_type, grid_type>(hint, grid);

    using bfs_queue_type = std::queue<active_cube_t>;
    using visited_type   = std::unordered_set<std::size_t>;

    auto const clear_memory = [](bfs_queue_type& queue, visited_type& visited) {
        bfs_queue_type empty_queue{};
        queue.swap(empty_queue);
        visited_type empty_set{};
        visited.swap(empty_set);
    };

    std::unordered_set<std::size_t> visited{};

    bfs_queue_type bfs_queue;
    bfs_queue.push(make_cube(ihint, jhint, khint));

    active_cube_t root;
    while (!bfs_queue.empty())
    {
        if (bfs_queue.size() == breadth_first_search_queue_max_size)
        {
            // bfs still hasn't found an active cube, just use the regular surface nets
            // implementation
            return surface_nets<ExecutionPolicy, Func, Scalar, Point, SharedVertexMeshTriangle>(
                std::forward<ExecutionPolicy>(policy),
                std::forward<Func>(f),
                grid,
                isovalue);
        }

        active_cube_t cube = bfs_queue.front();
        bfs_queue.pop();

        visited.insert(cube.idx);
        auto const edge_bipolarity_array =
            get_edge_bipolarity_array<scalar_type>(cube.voxel_corner_values, isovalue, edges);
        bool const is_cube_active = get_is_cube_active(edge_bipolarity_array);
        if (is_cube_active)
        {
            root = cube;
            break;
        }

        // if cube is inactive, add cube neighbors to the search queue if they haven't been visited
        // yet
        auto const [i, j, k]                  = get_ijk_from_idx<grid_type>(cube.idx, grid);
        std::size_t const neighbor_ijks[6][3] = {
            {i + 1, j, k},
            {i - 1, j, k},
            {i, j + 1, k},
            {i, j - 1, k},
            {i, j, k + 1},
            {i, j, k - 1}};

        std::uint64_t const neighbor_indexes[6] = {
            get_active_cube_index<grid_type>(
                neighbor_ijks[0][0],
                neighbor_ijks[0][1],
                neighbor_ijks[0][2],
                grid),
            get_active_cube_index<grid_type>(
                neighbor_ijks[1][0],
                neighbor_ijks[1][1],
                neighbor_ijks[1][2],
                grid),
            get_active_cube_index<grid_type>(
                neighbor_ijks[2][0],
                neighbor_ijks[2][1],
                neighbor_ijks[2][2],
                grid),
            get_active_cube_index<grid_type>(
                neighbor_ijks[3][0],
                neighbor_ijks[3][1],
                neighbor_ijks[3][2],
                grid),
            get_active_cube_index<grid_type>(
                neighbor_ijks[4][0],
                neighbor_ijks[4][1],
                neighbor_ijks[4][2],
                grid),
            get_active_cube_index<grid_type>(
                neighbor_ijks[5][0],
                neighbor_ijks[5][1],
                neighbor_ijks[5][2],
                grid)};

        for (auto l = 0u; l < 6u; ++l)
        {
            if (visited.count(neighbor_indexes[l]) == 0u)
            {
                bfs_queue.push(
                    make_cube(neighbor_ijks[l][0], neighbor_ijks[l][1], neighbor_ijks[l][2]));
            }
        }
    }

    // if an edge is bipolar, it means that
    // the other three cubes sharing this edges are
    // also active
    std::int8_t constexpr adjacent_cubes_of_edges[12][3][3] = {
        {{0, -1, 0}, {0, -1, -1}, {0, 0, -1}},
        {{1, 0, 0}, {1, 0, -1}, {0, 0, -1}},
        {{0, 1, 0}, {0, 1, -1}, {0, 0, -1}},
        {{-1, 0, 0}, {-1, 0, -1}, {0, 0, -1}},
        {{0, -1, 0}, {0, -1, 1}, {0, 0, 1}},
        {{1, 0, 0}, {1, 0, 1}, {0, 0, 1}},
        {{0, 1, 0}, {0, 1, 1}, {0, 0, 1}},
        {{-1, 0, 0}, {-1, 0, 1}, {0, 0, 1}},
        {{-1, 0, 0}, {-1, -1, 0}, {0, -1, 0}},
        {{1, 0, 0}, {1, -1, 0}, {0, -1, 0}},
        {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}},
        {{-1, 0, 0}, {-1, 1, 0}, {0, 1, 0}},
    };

    // now look at active cubes only which are connected to the root
    // and perform vertex placement
    std::unordered_map<std::size_t, active_cube_t> active_cubes_map{};
    clear_memory(bfs_queue, visited);
    bfs_queue.push(root);
    while (!bfs_queue.empty())
    {
        active_cube_t active_cube = bfs_queue.front();
        bfs_queue.pop();
        if (active_cubes_map.count(active_cube.idx) == 1)
            continue;

        auto const [i, j, k] = get_ijk_from_idx<grid_type>(active_cube.idx, grid);

        auto const voxel_corner_grid_positions =
            get_voxel_corner_grid_positions<point_type, scalar_type>(i, j, k);

        auto const edge_bipolarity_array = get_edge_bipolarity_array<scalar_type>(
            active_cube.voxel_corner_values,
            isovalue,
            edges);

        std::vector<point_type> edge_intersection_points;

        // visit every bipolar edge
        for (std::size_t e = 0; e < 12; ++e)
        {
            if (!edge_bipolarity_array[e])
                continue;

            // since this edge is bipolar, all cubes adjacent to it
            // are also active, so we add them to the queue
            auto const adjacent_cubes_of_edge =
                get_adjacent_cubes_of_edge(i, j, k, e, adjacent_cubes_of_edges);

            auto const c1 = get_active_cube_index<grid_type>(
                adjacent_cubes_of_edge[0][0],
                adjacent_cubes_of_edge[0][1],
                adjacent_cubes_of_edge[0][2],
                grid);
            auto const c2 = get_active_cube_index<grid_type>(
                adjacent_cubes_of_edge[1][0],
                adjacent_cubes_of_edge[1][1],
                adjacent_cubes_of_edge[1][2],
                grid);
            auto const c3 = get_active_cube_index<grid_type>(
                adjacent_cubes_of_edge[2][0],
                adjacent_cubes_of_edge[2][1],
                adjacent_cubes_of_edge[2][2],
                grid);

            if (active_cubes_map.count(c1) == 0)
            {
                bfs_queue.push(make_cube(
                    adjacent_cubes_of_edge[0][0],
                    adjacent_cubes_of_edge[0][1],
                    adjacent_cubes_of_edge[0][2]));
            }
            if (active_cubes_map.count(c2) == 0)
            {
                bfs_queue.push(make_cube(
                    adjacent_cubes_of_edge[1][0],
                    adjacent_cubes_of_edge[1][1],
                    adjacent_cubes_of_edge[1][2]));
            }
            if (active_cubes_map.count(c3) == 0)
            {
                bfs_queue.push(make_cube(
                    adjacent_cubes_of_edge[2][0],
                    adjacent_cubes_of_edge[2][1],
                    adjacent_cubes_of_edge[2][2]));
            }

            // get points p1, p2 of the edge e in grid coordinates
            auto const p1 = voxel_corner_grid_positions[edges[e][0]];
            auto const p2 = voxel_corner_grid_positions[edges[e][1]];

            // get value of the implicit function at edge vertices
            auto const s1 = active_cube.voxel_corner_values[edges[e][0]];
            auto const s2 = active_cube.voxel_corner_values[edges[e][1]];

            // perform linear interpolation using implicit function
            // values at vertices
            auto const t = (isovalue - s1) / (s2 - s1);
            edge_intersection_points.push_back(point_type{p1 + t * (p2 - p1)});
        }

        point_type const geometric_center_of_edge_intersection_points =
            pcp::common::center_of_geometry(
                edge_intersection_points.cbegin(),
                edge_intersection_points.cend());

        point_type const mesh_vertex = {
            mesh_aabb.min.x() + (mesh_aabb.max.x() - mesh_aabb.min.x()) *
                                    (geometric_center_of_edge_intersection_points.x() - 0.f) /
                                    (static_cast<scalar_type>(grid.sx) - 0.f),
            mesh_aabb.min.y() + (mesh_aabb.max.y() - mesh_aabb.min.y()) *
                                    (geometric_center_of_edge_intersection_points.y() - 0.f) /
                                    (static_cast<scalar_type>(grid.sy) - 0.f),
            mesh_aabb.min.z() + (mesh_aabb.max.z() - mesh_aabb.min.z()) *
                                    (geometric_center_of_edge_intersection_points.z() - 0.f) /
                                    (static_cast<scalar_type>(grid.sz) - 0.f),
        };

        auto const vertex_index           = vertices.size();
        active_cube.vertex_idx            = vertex_index;
        active_cubes_map[active_cube.idx] = active_cube;

        vertices.push_back(mesh_vertex);
    }

    using key_value_type = typename decltype(active_cubes_map)::value_type;

    std::mutex sync;

    std::for_each(
        policy,
        active_cubes_map.cbegin(),
        active_cubes_map.cend(),
        [&](key_value_type const& key_value) {
            auto const active_cube_index = key_value.first;
            auto const active_cube       = key_value.second;

            auto const is_lower_boundary_cube = [](auto i, auto j, auto k) {
                return (i == 0 || j == 0 || k == 0);
            };

            auto const [i, j, k] = get_ijk_from_idx<grid_type>(active_cube_index, grid);

            if (is_lower_boundary_cube(i, j, k))
                return;

            std::size_t const neighbor_grid_positions[6][3] = {
                {i - 1, j, k},
                {i - 1, j - 1, k},
                {i, j - 1, k},
                {i, j - 1, k - 1},
                {i, j, k - 1},
                {i - 1, j, k - 1}};

            scalar_type const edge_scalar_values[3][2] = {
                // directed edge (0,4)
                {active_cube.voxel_corner_values[0], active_cube.voxel_corner_values[4]},
                // directed edge (3,0)
                {active_cube.voxel_corner_values[3], active_cube.voxel_corner_values[0]},
                // directed edge (0,1)
                {active_cube.voxel_corner_values[0], active_cube.voxel_corner_values[1]}};

            std::size_t const quad_neighbors[3][3] = {{0, 1, 2}, {0, 5, 4}, {2, 3, 4}};

            std::array<std::size_t, 3> const quad_neighbor_orders[2] = {{0, 1, 2}, {2, 1, 0}};

            for (std::size_t idx = 0; idx < 3; ++idx)
            {
                auto const neighbor1 = get_active_cube_index<grid_type>(
                    neighbor_grid_positions[quad_neighbors[idx][0]][0],
                    neighbor_grid_positions[quad_neighbors[idx][0]][1],
                    neighbor_grid_positions[quad_neighbors[idx][0]][2],
                    grid);

                auto const neighbor2 = get_active_cube_index<grid_type>(
                    neighbor_grid_positions[quad_neighbors[idx][1]][0],
                    neighbor_grid_positions[quad_neighbors[idx][1]][1],
                    neighbor_grid_positions[quad_neighbors[idx][1]][2],
                    grid);

                auto const neighbor3 = get_active_cube_index<grid_type>(
                    neighbor_grid_positions[quad_neighbors[idx][2]][0],
                    neighbor_grid_positions[quad_neighbors[idx][2]][1],
                    neighbor_grid_positions[quad_neighbors[idx][2]][2],
                    grid);

                if (active_cubes_map.count(neighbor1) == 0 ||
                    active_cubes_map.count(neighbor2) == 0 ||
                    active_cubes_map.count(neighbor3) == 0)
                    continue;

                std::size_t const neighbor_vertices[3] = {
                    active_cubes_map[neighbor1].vertex_idx,
                    active_cubes_map[neighbor2].vertex_idx,
                    active_cubes_map[neighbor3].vertex_idx};

                auto const& neighbor_vertices_order =
                    edge_scalar_values[idx][1] > edge_scalar_values[idx][0] ?
                        quad_neighbor_orders[0] :
                        quad_neighbor_orders[1];

                auto const v0 = active_cube.vertex_idx;
                auto const v1 = neighbor_vertices[neighbor_vertices_order[0]];
                auto const v2 = neighbor_vertices[neighbor_vertices_order[1]];
                auto const v3 = neighbor_vertices[neighbor_vertices_order[2]];

                std::lock_guard<std::mutex> lock(sync);

                triangles.push_back(triangle_type{
                    static_cast<index_type>(v0),
                    static_cast<index_type>(v1),
                    static_cast<index_type>(v2)});
                triangles.push_back(triangle_type{
                    static_cast<index_type>(v0),
                    static_cast<index_type>(v2),
                    static_cast<index_type>(v3)});
            }
        });

    return {vertices, triangles};
}

} // namespace isosurface
} // namespace algorithm
} // namespace pcp
