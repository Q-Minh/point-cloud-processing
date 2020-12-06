#include <algorithm>
#include <execution>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/point_view.hpp>
#include <pcp/common/timer.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>
#include <random>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage:\n"
                  << "noise.exe <input ply file> <output ply file> [k neighborhood size] [stddev "
                     "multiplier (default = 0.01)]\n";
        return 1;
    }

    std::filesystem::path input_ply(argv[1]);
    std::filesystem::path output_ply(argv[2]);
    std::size_t const k           = argc >= 4 ? std::stoull(argv[3]) : 15u;
    float const stddev_multiplier = argc >= 5 ? std::stof(argv[4]) : 0.01f;

    pcp::common::basic_timer_t timer;

    timer.register_op("parse ply point cloud");
    timer.start();
    auto [points, _] = pcp::io::read_ply<pcp::point_t, pcp::normal_t>(input_ply);
    timer.stop();

    timer.register_op("setup octree");
    timer.start();
    std::vector<pcp::point_view_t> point_views;
    point_views.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
        point_views.push_back(pcp::point_view_t{&points[i]});

    pcp::basic_octree_t<pcp::point_view_t> octree{std::cbegin(point_views), std::cend(point_views)};
    timer.stop();

    timer.register_op("compute stddev of k neighborhoods");
    timer.start();

    std::vector<float> mean_distances;
    mean_distances.resize(points.size(), 0.f);

    std::transform(
        std::execution::par,
        points.cbegin(),
        points.cend(),
        mean_distances.begin(),
        [&k, &octree, &mean_distances](pcp::point_t const& p) {
            auto const& neighbors = octree.nearest_neighbours(p, k);
            float const sum       = std::accumulate(
                neighbors.cbegin(),
                neighbors.cend(),
                0.f,
                [&p](float sum, pcp::point_view_t const& neighbor) {
                    auto const distance = pcp::common::norm(pcp::point_t{neighbor} - p);
                    return sum + distance;
                });
            float const mean_distance = sum / static_cast<float>(neighbors.size());
            return mean_distance;
        });

    auto const n   = mean_distances.size();
    float const mu = std::reduce(
        std::execution::par,
        mean_distances.cbegin(),
        mean_distances.cend(), 0.f) /
        static_cast<float>(n);
    float const var = std::reduce(
        std::execution::par,
        mean_distances.cbegin(),
        mean_distances.cend(),
        0.f,
        [=](float current, float mean) {
            auto const diff = mean - mu;
            return current + (diff * diff);
        });
    float const stddev = std::sqrt(var);
    timer.stop();

    timer.register_op("add gaussian noise to the point cloud");
    timer.start();
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<float> gaussian{0.f, stddev};
    std::transform(
        std::execution::par,
        points.cbegin(),
        points.cend(),
        points.begin(),
        [&gaussian, &gen, &stddev_multiplier](pcp::point_t const& p) {
            return p + pcp::common::basic_vector3d_t{
                           stddev_multiplier * gaussian(gen),
                           stddev_multiplier * gaussian(gen),
                           stddev_multiplier * gaussian(gen)};
        });
    timer.stop();

    timer.register_op("write noisy point cloud to ply file");
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