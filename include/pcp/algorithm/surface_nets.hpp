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
#include <unordered_map>
#include <utility>
#include <vector>

namespace pcp {
namespace algorithm {
namespace isosurface {

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
    using grid_type     = common::regular_grid3d_t<Scalar>;
    using scalar_type   = Scalar;

    // the shared vertex mesh
    std::vector<point_type> vertices;
    std::vector<triangle_type> triangles;

    // bounding box of the mesh in coordinate frame of the mesh
    pcp::axis_aligned_bounding_box_t<point_type> mesh_aabb = {
        {grid.x, grid.y, grid.z},
        {grid.x + grid.sx * grid.dx, grid.y + grid.sy * grid.dy, grid.z + grid.sz * grid.dz}};

    // mapping from 3d grid coordinates to 1d
    auto const get_active_cube_index =
        [](std::size_t x, std::size_t y, std::size_t z, grid_type const& grid) -> std::size_t {
        return x + (y * grid.sx) + (z * grid.sx * grid.sy);
    };

    // mapping from 1d index to 3d grid coordinates
    auto const get_ijk_from_idx =
        [](std::size_t active_cube_index,
           grid_type const& grid) -> std::tuple<std::size_t, std::size_t, std::size_t> {
        std::size_t i = (active_cube_index) % grid.sx;
        std::size_t j = (active_cube_index / grid.sx) % grid.sy;
        std::size_t k = (active_cube_index) / (grid.sx * grid.sy);
        return std::make_tuple(i, j, k);
    };

    // mapping from active cube indices to vertex indices of the generated mesh
    std::unordered_map<std::size_t, std::uint64_t> active_cube_to_vertex_index_map{};

    bool const is_x_longest_dimension = grid.sx > grid.sy && grid.sx > grid.sz;
    bool const is_y_longest_dimension = grid.sy > grid.sx && grid.sy > grid.sz;
    bool const is_z_longest_dimension = grid.sz > grid.sx && grid.sz > grid.sy;

    std::size_t longest_dimension_size =
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
                    // clang-format off
			        point_type const voxel_corner_grid_positions[8] =
			        {
				        { static_cast<scalar_type>(i),     static_cast<scalar_type>(j),     static_cast<scalar_type>(k) },
				        { static_cast<scalar_type>(i + 1), static_cast<scalar_type>(j),     static_cast<scalar_type>(k) },
				        { static_cast<scalar_type>(i + 1), static_cast<scalar_type>(j + 1), static_cast<scalar_type>(k) },
				        { static_cast<scalar_type>(i),     static_cast<scalar_type>(j + 1), static_cast<scalar_type>(k) },
				        { static_cast<scalar_type>(i),     static_cast<scalar_type>(j),     static_cast<scalar_type>(k + 1) },
				        { static_cast<scalar_type>(i + 1), static_cast<scalar_type>(j),     static_cast<scalar_type>(k + 1) },
				        { static_cast<scalar_type>(i + 1), static_cast<scalar_type>(j + 1), static_cast<scalar_type>(k + 1) },
				        { static_cast<scalar_type>(i),     static_cast<scalar_type>(j + 1), static_cast<scalar_type>(k + 1) },
			        };

			        // coordinates of voxel corners in the mesh's coordinate frame
			        point_type const voxel_corner_positions[8] =
			        {
				        { grid.x + i * grid.dx,       grid.y + j * grid.dy,       grid.z + k * grid.dz },
				        { grid.x + (i + 1) * grid.dx, grid.y + j * grid.dy,       grid.z + k * grid.dz },
				        { grid.x + (i + 1) * grid.dx, grid.y + (j + 1) * grid.dy, grid.z + k * grid.dz },
				        { grid.x + i * grid.dx,       grid.y + (j + 1) * grid.dy, grid.z + k * grid.dz },
				        { grid.x + i * grid.dx,       grid.y + j * grid.dy,       grid.z + (k + 1) * grid.dz },
				        { grid.x + (i + 1) * grid.dx, grid.y + j * grid.dy,       grid.z + (k + 1) * grid.dz },
				        { grid.x + (i + 1) * grid.dx, grid.y + (j + 1) * grid.dy, grid.z + (k + 1) * grid.dz },
				        { grid.x + i * grid.dx,       grid.y + (j + 1) * grid.dy, grid.z + (k + 1) * grid.dz }
			        };

			        // scalar values of the implicit function evaluated at cube vertices (voxel corners)
			        scalar_type const voxel_corner_values[8] =
			        {
				        f(voxel_corner_positions[0].x(), voxel_corner_positions[0].y(), voxel_corner_positions[0].z()),
				        f(voxel_corner_positions[1].x(), voxel_corner_positions[1].y(), voxel_corner_positions[1].z()),
				        f(voxel_corner_positions[2].x(), voxel_corner_positions[2].y(), voxel_corner_positions[2].z()),
				        f(voxel_corner_positions[3].x(), voxel_corner_positions[3].y(), voxel_corner_positions[3].z()),
				        f(voxel_corner_positions[4].x(), voxel_corner_positions[4].y(), voxel_corner_positions[4].z()),
				        f(voxel_corner_positions[5].x(), voxel_corner_positions[5].y(), voxel_corner_positions[5].z()),
				        f(voxel_corner_positions[6].x(), voxel_corner_positions[6].y(), voxel_corner_positions[6].z()),
				        f(voxel_corner_positions[7].x(), voxel_corner_positions[7].y(), voxel_corner_positions[7].z())
			        };
                    // clang-format on

                    // the edges provide indices to the corresponding current cube's vertices (voxel
                    // corners)
                    std::size_t const edges[12][2] = {
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

                    auto const is_scalar_positive = [](scalar_type scalar,
                                                       scalar_type isovalue) -> bool {
                        return scalar >= isovalue;
                    };

                    auto const are_edge_scalars_bipolar = [&is_scalar_positive](
                                                              scalar_type scalar1,
                                                              scalar_type scalar2,
                                                              scalar_type isovalue) -> bool {
                        return is_scalar_positive(scalar1, isovalue) !=
                               is_scalar_positive(scalar2, isovalue);
                    };

                    bool const edge_bipolarity_array[12] = {
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[0][0]],
                            voxel_corner_values[edges[0][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[1][0]],
                            voxel_corner_values[edges[1][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[2][0]],
                            voxel_corner_values[edges[2][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[3][0]],
                            voxel_corner_values[edges[3][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[4][0]],
                            voxel_corner_values[edges[4][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[5][0]],
                            voxel_corner_values[edges[5][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[6][0]],
                            voxel_corner_values[edges[6][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[7][0]],
                            voxel_corner_values[edges[7][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[8][0]],
                            voxel_corner_values[edges[8][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[9][0]],
                            voxel_corner_values[edges[9][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[10][0]],
                            voxel_corner_values[edges[10][1]],
                            isovalue),
                        are_edge_scalars_bipolar(
                            voxel_corner_values[edges[11][0]],
                            voxel_corner_values[edges[11][1]],
                            isovalue),
                    };

                    // clang-format off
			        // an active voxel must have at least one bipolar edge
			        bool const is_voxel_active = edge_bipolarity_array[0] ||
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

                    std::size_t const active_cube_index = get_active_cube_index(i, j, k, grid);

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

            auto const is_lower_boundary_cube = [](auto i, auto j, auto k, grid_type const& g) {
                return (i == 0 || j == 0 || k == 0);
            };

            auto const ijk = get_ijk_from_idx(active_cube_index, grid);
            auto const i   = std::get<0>(ijk);
            auto const j   = std::get<1>(ijk);
            auto const k   = std::get<2>(ijk);

            if (is_lower_boundary_cube(i, j, k, grid))
                return;

            // clang-format off
            std::size_t const neighbor_grid_positions[6][3] = {
                {i - 1, j,     k},
                {i - 1, j - 1, k},
                {i,     j - 1, k},
                {i,     j - 1, k - 1},
                {i,     j,     k - 1},
                {i - 1, j,     k - 1}};

            point_type const voxel_corners_of_interest[4] = {
                // vertex 0
                {grid.x + i * grid.dx,       grid.y + j * grid.dy,       grid.z + k * grid.dz},
                // vertex 4                                              
                {grid.x + i * grid.dx,       grid.y + j * grid.dy,       grid.z + (k + 1) * grid.dz},
                // vertex 3                  
                {grid.x + i * grid.dx,       grid.y + (j + 1) * grid.dy, grid.z + k * grid.dz},
                // vertex 1
                {grid.x + (i + 1) * grid.dx, grid.y + j * grid.dy,       grid.z + k * grid.dz}};
            // clang-format on

            float const edge_scalar_values[3][2] = {// directed edge (0,4)
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

            for (std::size_t i = 0; i < 3; ++i)
            {
                auto const neighbor1 = get_active_cube_index(
                    neighbor_grid_positions[quad_neighbors[i][0]][0],
                    neighbor_grid_positions[quad_neighbors[i][0]][1],
                    neighbor_grid_positions[quad_neighbors[i][0]][2],
                    grid);

                auto const neighbor2 = get_active_cube_index(
                    neighbor_grid_positions[quad_neighbors[i][1]][0],
                    neighbor_grid_positions[quad_neighbors[i][1]][1],
                    neighbor_grid_positions[quad_neighbors[i][1]][2],
                    grid);

                auto const neighbor3 = get_active_cube_index(
                    neighbor_grid_positions[quad_neighbors[i][2]][0],
                    neighbor_grid_positions[quad_neighbors[i][2]][1],
                    neighbor_grid_positions[quad_neighbors[i][2]][2],
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
                    edge_scalar_values[i][1] > edge_scalar_values[i][0] ? quad_neighbor_orders[0] :
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

} // namespace isosurface
} // namespace algorithm
} // namespace pcp