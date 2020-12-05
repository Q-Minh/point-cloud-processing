#include <algorithm>
#include <execution>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/point_view.hpp>
#include <pcp/common/sphere.hpp>
#include <pcp/common/timer.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: filter.exe <input ply file> <output ply file> [density threshold = 5] "
                     "[neighborhood radius multipler = 1] [k nearest neighbours = 15]\n";
        return 1;
    }

    std::filesystem::path input_ply{argv[1]};
    std::filesystem::path output_ply{argv[2]};
    std::size_t const density_threshold = argc >= 4 ? std::stoull(argv[3]) : 5u;
    float const radius_multiplier       = argc >= 5 ? std::stof(argv[4]) : 1.f;
    std::size_t const k                 = argc >= 6 ? std::stoull(argv[5]) : 15u;

    pcp::common::basic_timer_t timer;
    timer.register_op("parse ply point cloud");
    timer.start();
    auto [points, _] = pcp::io::read_ply<pcp::point_t, pcp::normal_t>(input_ply);
    timer.stop();

    timer.register_op("setup octree");
    timer.start();
    std::vector<pcp::point_view_t> point_views(points.size(), pcp::point_view_t{});
    std::transform(
        std::execution::par,
        points.begin(),
        points.end(),
        point_views.begin(),
        [](pcp::point_t& p) { return pcp::point_view_t(&p); });

    pcp::basic_octree_t<pcp::point_view_t> octree{point_views.cbegin(), point_views.cend()};
    timer.stop();

    timer.register_op("compute k neighborhood average radius");
    timer.start();
    std::vector<float> mean_distances(points.size(), 0.f);
    std::transform(
        std::execution::par,
        points.cbegin(),
        points.cend(),
        mean_distances.begin(),
        [&octree, &k](pcp::point_t const& p) {
            auto const& neighbours = octree.nearest_neighbours(p, k);
            float const sum        = std::accumulate(
                neighbours.cbegin(),
                neighbours.cend(),
                0.f,
                [&p](float val, pcp::point_view_t const& neighbour) {
                    auto const distance = pcp::common::norm(pcp::point_t(neighbour) - p);
                    return val + distance;
                });

            return sum / static_cast<float>(neighbours.size());
        });

    float const radius =
        std::reduce(std::execution::par, mean_distances.cbegin(), mean_distances.cend(), 0.f) /
        static_cast<float>(mean_distances.size());

    timer.stop();

    timer.register_op("remove points by density threshold");
    timer.start();
    auto it = std::remove_if(
        std::execution::par,
        points.begin(),
        points.end(),
        [&octree, &radius, &density_threshold, &radius_multiplier](pcp::point_t const& p) {
            pcp::sphere_t<pcp::point_t> ball{p, radius * radius_multiplier};
            auto const& points_in_ball = octree.range_search(ball);
            auto const density         = points_in_ball.size();
            return density < density_threshold;
        });
    points.erase(it, points.end());
    timer.stop();

    timer.register_op("write filtered point cloud to ply");
    timer.start();
    pcp::io::write_ply(output_ply, points, _, pcp::io::ply_format_t::binary_little_endian);
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