#include <catch2/catch.hpp>
#include <pcp/algorithm/average_distance_to_neighbors.hpp>
#include <pcp/algorithm/wlop.hpp>

SCENARIO("WLOP downsampling point cloud", "[resampling][downsampling][WLOP]")
{
    GIVEN("a random point cloud")
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-10.f, 10.f);
        auto const n = 1000u;
        std::vector<pcp::point_t> point_cloud;
        point_cloud.resize(n);
        std::generate(point_cloud.begin(), point_cloud.end(), [&dis, &gen]() {
            return pcp::point_t{dis(gen), dis(gen), dis(gen)};
        });

        std::vector<std::size_t> indices(n);
        std::iota(indices.begin(), indices.end(), 0u);

        auto const point_map = [&](std::size_t const i) {
            return point_cloud[i];
        };
        auto const coordinate_map = [&](std::size_t const i) {
            return std::array<float, 3u>{
                point_cloud[i].x(),
                point_cloud[i].y(),
                point_cloud[i].z()};
        };

        auto const get_mean_distance_to_neighbors = [&]() {
            pcp::kdtree::construction_params_t params;
            params.compute_max_depth = true;
            pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(coordinate_map)> kdtree{
                indices.begin(),
                indices.end(),
                coordinate_map,
                params};
            auto const knn_map = [&](std::size_t const i) {
                return kdtree.nearest_neighbours(i, 15u);
            };
            return pcp::algorithm::average_distance_to_neighbors(
                indices.begin(),
                indices.end(),
                point_map,
                knn_map);
        };

        WHEN("downsampling with WLOP")
        {
            pcp::algorithm::wlop::params_t params;
            params.k       = 2u;
            params.I       = n / 2;
            params.h       = get_mean_distance_to_neighbors();
            params.uniform = true;

            std::vector<pcp::point_t> downsampled_points{};
            pcp::algorithm::wlop::wlop(
                indices.begin(),
                indices.end(),
                std::back_inserter(downsampled_points),
                point_map,
                params);

            THEN("The downsampled point cloud has the correct size") 
            {
                auto const downsampled_size = downsampled_points.size();
                REQUIRE(downsampled_size == params.I);
            }
        }
    }
}