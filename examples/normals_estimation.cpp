#include <chrono>
#include <iostream>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/vertex.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>

struct basic_timer_t
{
    using time_type     = std::chrono::high_resolution_clock::time_point;
    using duration_type = std::chrono::high_resolution_clock::duration;

    void start() { begin = std::chrono::high_resolution_clock::now(); }
    void stop()
    {
        end               = std::chrono::high_resolution_clock::now();
        ops.back().second = (end - begin);
    }
    void register_op(std::string const& op_name) { ops.push_back({op_name, {}}); }

    time_type begin;
    time_type end;
    std::vector<std::pair<std::string, duration_type>> ops;
};

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage:\n"
                  << "normals-estimation.exe <input ply file> <output ply file> [k neighborhood "
                     "size] [parallel | seq]\n";
        return -1;
    }

    std::uint64_t const k                 = argc == 4 ? std::stoull(argv[3]) : 10u;
    bool const parallel                   = argc == 5 ? std::string(argv[4]) == "parallel" : false;
    std::filesystem::path ply_point_cloud = argv[1];

    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using vertex_type = pcp::vertex_t;

    basic_timer_t timer;

    timer.register_op("parse ply point cloud");
    timer.start();
    auto [points, normals] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
    timer.stop();
    
    timer.register_op("setup octree");
    timer.start();
    std::vector<vertex_type> vertices;
    vertices.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
        vertices.push_back(vertex_type{&points[i], i});

    normals.resize(points.size());

    using iterator_type = typename decltype(points)::const_iterator;
    auto const bounding_box =
        pcp::bounding_box<iterator_type, pcp::point_t>(std::cbegin(points), std::cend(points));

    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid = bounding_box;

    pcp::basic_octree_t<vertex_type, decltype(params)> octree{
        std::cbegin(vertices),
        std::cend(vertices),
        params};

    timer.stop();

    auto const knn = [&octree, &vertices](vertex_type const& v) {
        std::uint64_t const k = 15u;
        return octree.nearest_neighbours(vertices[v.id()], k);
    };
    auto const get_point = [&vertices](vertex_type const& v) {
        return pcp::point_t{vertices[v.id()]};
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
    pcp::algorithm::propagate_normal_orientations(
        vertices.begin(),
        vertices.end(),
        knn,
        get_point,
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
