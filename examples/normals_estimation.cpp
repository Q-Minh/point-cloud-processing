#include <iostream>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/vertex.hpp>
#include <pcp/common/timer.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage:\n"
                  << "normals-estimation.exe <input ply file> <output ply file> [k neighborhood "
                     "size] [parallel | seq]\n";
        return -1;
    }

    std::uint64_t const k                 = argc >= 4 ? std::stoull(argv[3]) : 10u;
    bool const parallel                   = argc >= 5 ? std::string(argv[4]) == "parallel" : false;
    std::filesystem::path ply_point_cloud = argv[1];

    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using vertex_type = pcp::vertex_t;

    pcp::common::basic_timer_t timer;

    timer.register_op("parse ply point cloud");
    timer.start();
    auto [points, normals] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
    timer.stop();

    timer.register_op("setup octree");
    timer.start();
    auto vertices = ranges::views::enumerate(points) | ranges::views::transform([](auto&& tup) {
                        auto const idx = std::get<0>(tup);
                        auto& point    = std::get<1>(tup);
                        return vertex_type{&point, idx};
                    }) |
                    ranges::to<std::vector>();

    normals.resize(points.size());

    auto const point_map = [](vertex_type const& v) {
        return *v.point();
    };
    pcp::basic_linked_octree_t<vertex_type> octree(vertices.begin(), vertices.end(), point_map);

    timer.stop();

    auto const knn = [&](vertex_type const& v) {
        return octree.nearest_neighbours(vertices[v.id()], k, point_map);
    };
    auto const get_normal = [normals_ptr = &normals](vertex_type const& v) {
        auto const& normals = *normals_ptr;
        return normals[v.id()];
    };
    auto const transform_op = [normals_ptr =
                                   &normals](vertex_type const& v, pcp::normal_t const& n) {
        auto& normals   = *normals_ptr;
        normals[v.id()] = n;
    };

    timer.register_op("estimate normals");
    timer.start();
    if (parallel)
    {
        pcp::algorithm::estimate_normals(
            std::execution::par,
            vertices.begin(),
            vertices.end(),
            normals.begin(),
            knn,
            pcp::algorithm::default_normal_transform<vertex_type, normal_type>);
    }
    else
    {
        pcp::algorithm::estimate_normals(
            std::execution::seq,
            vertices.begin(),
            vertices.end(),
            normals.begin(),
            knn,
            pcp::algorithm::default_normal_transform<vertex_type, normal_type>);
    }
    timer.stop();

    timer.register_op("propagate normal orientations");
    timer.start();
    auto const index_map = [](vertex_type const& v) {
        return v.id();
    };

    pcp::algorithm::propagate_normal_orientations(
        vertices.begin(),
        vertices.end(),
        index_map,
        knn,
        point_map,
        get_normal,
        transform_op);
    timer.stop();

    timer.register_op("write ply point cloud with estimated normals");
    timer.start();
    pcp::io::write_ply(
        std::filesystem::path(argv[2]),
        points,
        normals,
        pcp::io::ply_format_t::binary_little_endian);
    timer.stop();

    for (auto const [operation, duration] : timer.ops)
    {
        std::cout << operation << ": "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
                  << " ms"
                  << "\n";
    }

    return 0;
}
