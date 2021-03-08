#include <catch2/catch.hpp>
#include <cmath>
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
                /*
                 * With uniform simplification, we should get the following relation:
                 *
                 *   n
                 * ----- <= cluster_size <===> n / cluster_size <= 2^x <===> log(n / cluster_size)
                 * <= x 2^x
                 *
                 * For a perfect size of input point cloud, we will get only clusters of exactly
                 * cluster_size or exactly (cluster_size - 1) depending on the implementation.
                 * For any other input point cloud, we will get clusters of lesser size than
                 * that, which means that we will get many more clusters. Thus, this relation
                 * only lets us find a lower bound on the number of clusters.
                 */
                auto const downsampled_size = downsampled_points.size();
                auto const power =
                    std::log2(static_cast<double>(n) / static_cast<double>(params.cluster_size));
                std::size_t const expected_lower_bound =
                    static_cast<std::size_t>(std::ceil(std::pow(2., power)));

                REQUIRE(downsampled_size >= expected_lower_bound);
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
                /*
                 * With non-uniform simplification, we should get even more
                 * clusters than in uniform simplification. This is explained
                 * by the fact that given a randomly sampled point cloud,
                 * there are bound to be many regions with isotropic variance.
                 */
                auto const downsampled_size = downsampled_points.size();
                auto const power =
                    std::log2(static_cast<double>(n) / static_cast<double>(params.cluster_size));
                std::size_t const expected_lower_bound =
                    static_cast<std::size_t>(std::ceil(std::pow(2., power)));

                // should we make this strictly greater or greater-equal?
                // given a random point cloud, there are bound to be clusters
                // with var > var_max
                REQUIRE(downsampled_size > expected_lower_bound);
            }
        }
    }
}