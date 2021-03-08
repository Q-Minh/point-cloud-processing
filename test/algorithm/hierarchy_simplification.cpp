#include <catch2/catch.hpp>
#include <pcp/algorithm/hierarchy_simplification.hpp>
#include <pcp/common/points/point.hpp>

SCENARIO("hierarchy simplification of point cloud", "[resampling][downsampling][hierarchy]")
{
    GIVEN("a random point cloud")
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-10.f, 10.f);
        auto const n = 1000u;
        std::vector<pcp::point_t> points;
        points.resize(n);
        std::generate(points.begin(), points.end(), [&dis, &gen]() {
            return pcp::point_t{dis(gen), dis(gen), dis(gen)};
        });

        auto const point_map = [](pcp::point_t const& p) {
            return p;
        };

        WHEN("downsampling with uniform hierarchy simplification")
        {
            pcp::algorithm::hierarchy::params_t params;
            params.cluster_size = 5u;
            params.var_max      = 1. / 3.; // uniform downsampling

            std::vector<pcp::point_t> downsampled_points{};
            pcp::algorithm::hierarchy_simplification(
                points.begin(),
                points.end(),
                std::back_inserter(downsampled_points),
                point_map,
                params);

            THEN("The downsampled point cloud has the correct size")
            {
                auto const downsampled_size          = downsampled_points.size();
                auto const expected_downsampled_size = n / params.cluster_size;
                REQUIRE(downsampled_size == expected_downsampled_size);
            }
        }
        WHEN("downsampling with non-uniform hierarchy simplification") 
        {
            pcp::algorithm::hierarchy::params_t params;
            params.cluster_size = 5u;
            params.var_max      = 0.1; // non-uniform downsampling

            std::vector<pcp::point_t> downsampled_points{};
            pcp::algorithm::hierarchy_simplification(
                points.begin(),
                points.end(),
                std::back_inserter(downsampled_points),
                point_map,
                params);

            THEN("The downsampled point cloud has the correct size")
            {
                auto const downsampled_size          = downsampled_points.size();
                auto const expected_downsampled_size = n / params.cluster_size;
                // should we make this strictly greater? 
                // given a random point cloud, there are bound to be clusters 
                // with var > 0.1
                REQUIRE(downsampled_size >= expected_downsampled_size);
            }            
        }
    }
}