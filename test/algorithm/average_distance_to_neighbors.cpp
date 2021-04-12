#include <catch2/catch.hpp>
#include <pcp/algorithm/average_distance_to_neighbors.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/kdtree/linked_kdtree.hpp>

SCENARIO("Computing average distances to neighbors in a point cloud", "[algorithm][average]")
{
    GIVEN("a point cloud")
    {
        float const d = 0.1f;
        std::vector<pcp::point_t> points;

        /*
        * Points are separated into 4 clusters with each cluster having a 
        * configuration similar to this one:
        * 
        * |-- d --|-- d --|
        * 1       2       3
        * o-------o-------o
        * 
        * The distances from 1 to its neighbors 2,3 are d,2d with average 3d/2
        * The distances from 2 to its neighbors 1,3 are d,d with average d
        * The distances from 3 to its neighbors 1,2 are 2d,d with average 3d/2
        * 
        * For 4 clusters, we thus have 4*(3d/2 + d + 3d/2) as the sum of all distances to neighbors.
        * For 4 clusters of 3 points, we have 12 points. 
        * Thus, the average distance to neighbors is: 
        * mu = 4*(3d/2 + d + 3d/2) / 12
        * 
        * We can also simplify (3d/2 + d + 3d/2) = (3d + d) = 4d
        * <=>
        * mu = 4*4d / 12 = 16/12 * d
        * 
        */
        points.push_back({0.f, 0.f, 0.f});
        points.push_back({0.f, 0.f, 0.f + d});
        points.push_back({0.f, 0.f, 0.f - d});

        points.push_back({1.f, 0.f, 0.f});
        points.push_back({1.f, 0.f + d, 0.f});
        points.push_back({1.f, 0.f - d, 0.f});

        points.push_back({0.f, 1.f, 0.f});
        points.push_back({0.f + d, 1.f, 0.f});
        points.push_back({0.f - d, 1.f, 0.f});

        points.push_back({0.f, 0.f, 1.f});
        points.push_back({0.f + d, 0.f, 1.f});
        points.push_back({0.f - d, 0.f, 1.f});

        WHEN("computing the average distances to k = 2 neighbors")
        {
            auto const point_map = [](pcp::point_t const& p) {
                return p;
            };
            auto const coordinate_map = [](pcp::point_t const& p) {
                return std::array<float, 3u>{p.x(), p.y(), p.z()};
            };

            pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)> kdtree{
                points.begin(),
                points.end(),
                coordinate_map};

            auto const knn_map = [&](pcp::point_t const& p) {
                return kdtree.nearest_neighbours(p, 2u);
            };

            float const mu = pcp::algorithm::average_distance_to_neighbors(
                std::execution::seq,
                points.begin(),
                points.end(),
                point_map,
                knn_map);

            THEN("The average is correctly computed")
            {
                float const expected = (16.f / 12.f) * d;
                REQUIRE(pcp::common::floating_point_equals(mu, expected));
            }
        }
    }
}