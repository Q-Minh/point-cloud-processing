#include <iostream>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/algorithm/estimate_tangent_planes.hpp>
#include <pcp/algorithm/surface_nets.hpp>
#include <pcp/common/points/vertex.hpp>
#include <pcp/common/timer.hpp>
#include <pcp/common/vector3d.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>
#include <string_view>

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        std::cerr << "Usage:\n"
                  << "tangent-plane-surface-reconstruction.exe <input ply file> <output ply file> "
                     "<num cells in x axis> <num cells in y axis> <num cells in z axis> [k "
                     "neighborhood size]"
                     " [parallel | seq]\n";
        return -1;
    }

    std::uint64_t const k                 = argc >= 7 ? std::stoull(argv[6]) : 10u;
    bool const parallel                   = argc >= 8 ? std::string(argv[7]) == "parallel" : true;
    std::filesystem::path ply_point_cloud = argv[1];
    std::filesystem::path ply_mesh        = argv[2];
    std::size_t const dimx                = std::stoull(argv[3]);
    std::size_t const dimy                = std::stoull(argv[4]);
    std::size_t const dimz                = std::stoull(argv[5]);

    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using vertex_type = pcp::vertex_t;
    using plane_type  = pcp::common::plane3d_t;

    pcp::common::basic_timer_t timer;

    timer.register_op("parse ply point cloud");
    timer.start();
    auto [points, normals] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
    std::cout << "points: " << points.size() << "\n";
    timer.stop();

    timer.register_op("setup octree");
    timer.start();
    std::vector<vertex_type> vertices;
    vertices.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
        vertices.push_back(vertex_type{&points[i], i});

    pcp::basic_octree_t<vertex_type> octree{std::cbegin(vertices), std::cend(vertices)};

    timer.stop();

    auto const point_knn = [=, &octree](vertex_type const& v) {
        auto const& neighbours = octree.nearest_neighbours(v, k);
        return std::vector<point_type>(neighbours.cbegin(), neighbours.cend());
    };

    std::vector<plane_type> tangent_planes;
    tangent_planes.resize(points.size());

    timer.register_op("estimate tangent planes");
    timer.start();
    if (parallel)
    {
        pcp::algorithm::estimate_tangent_planes(
            std::execution::par,
            vertices.cbegin(),
            vertices.cend(),
            tangent_planes.begin(),
            point_knn,
            pcp::algorithm::default_plane_transform<point_type, plane_type>);
    }
    else
    {
        pcp::algorithm::estimate_tangent_planes(
            std::execution::seq,
            vertices.cbegin(),
            vertices.cend(),
            tangent_planes.begin(),
            point_knn,
            pcp::algorithm::default_plane_transform<point_type, plane_type>);
    }
    timer.stop();

    auto const vertex_knn = [=, &octree](vertex_type const& v) {
        return octree.nearest_neighbours(v, k);
    };
    auto const get_point = [&vertices](vertex_type const& v) {
        return pcp::point_t{vertices[v.id()]};
    };
    auto const get_normal = [&tangent_planes](vertex_type const& v) {
        return tangent_planes[v.id()].normal();
    };
    auto const transform_op = [&tangent_planes](vertex_type const& v, pcp::normal_t const& n) {
        tangent_planes[v.id()].normal(n);
    };

    timer.register_op("propagate normal orientations");
    timer.start();
    pcp::algorithm::propagate_normal_orientations(
        vertices.begin(),
        vertices.end(),
        vertex_knn,
        get_point,
        get_normal,
        transform_op);
    timer.stop();

    auto const signed_distance_function = [=, &octree, &tangent_planes](float x, float y, float z) {
        point_type const p{x, y, z};
        auto const nearest_neighbours = octree.nearest_neighbours(p, 1u);
        auto const& nearest_vertex    = nearest_neighbours.front();
        auto const idx                = nearest_vertex.id();
        auto const& tangent_plane     = tangent_planes[idx];
        auto const o                  = tangent_plane.point();
        auto const n                  = tangent_plane.normal();
        auto const op                 = p - o;
        auto const f                  = pcp::common::inner_product(op, n);
        return f;
    };

    timer.register_op("surface nets");
    timer.start();
    auto const grid = pcp::common::regular_grid_containing(
        octree.voxel_grid().min,
        octree.voxel_grid().max,
        {dimx, dimy, dimz});

    auto const mesh = [](auto const& sdf, auto const& grid, bool parallelize) {
        if (parallelize)
            return pcp::algorithm::isosurface::surface_nets(std::execution::par, sdf, grid);
        else
            return pcp::algorithm::isosurface::surface_nets(std::execution::seq, sdf, grid);
    };
    auto const [mesh_vertices, mesh_triangles] = mesh(signed_distance_function, grid, parallel);
    timer.stop();

    timer.register_op("write ply mesh");
    timer.start();

    pcp::io::write_ply(ply_mesh, mesh_vertices, mesh_triangles, pcp::io::ply_format_t::ascii);
    timer.stop();

    for (auto const [operation, duration] : timer.ops)
    {
        std::cout << operation << ": "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
                  << " ms"
                  << "\n";
    }

    std::cout << "vertices: " << mesh_vertices.size() << "\n"
              << "triangles: " << mesh_triangles.size() << "\n";

    return 0;
}
