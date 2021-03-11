#include <catch2/catch.hpp>
#include <pcp/algorithm/random_simplification.hpp>
#include <pcp/common/points/point.hpp>

SCENARIO("random simplification of point cloud", "[resampling][downsampling][random-simplification]")
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

        WHEN("downsampling with uniform hierarchy simplification")
        {
            std::size_t const expected_downsampled_point_size = 200u;
            WHEN("using indices shuffling") 
            {
                bool const use_indices = true;
                std::vector<pcp::point_t> downsampled_points{};
                pcp::algorithm::random_simplification(
                    points.begin(),
                    points.end(),
                    std::back_inserter(downsampled_points),
                    expected_downsampled_point_size, 
                    use_indices);

                THEN("The downsampled point cloud has the correct size")
                {
                    REQUIRE(downsampled_points.size() == expected_downsampled_point_size);
                }
            }
            WHEN("shuffling elements directly") 
            {
                bool const use_indices = false;
                std::vector<pcp::point_t> downsampled_points{};
                pcp::algorithm::random_simplification(
                    points.begin(),
                    points.end(),
                    std::back_inserter(downsampled_points),
                    expected_downsampled_point_size,
                    use_indices);

                THEN("The downsampled point cloud has the correct size") 
                {
                    REQUIRE(downsampled_points.size() == expected_downsampled_point_size);
                }
            }
        }
    }
}
