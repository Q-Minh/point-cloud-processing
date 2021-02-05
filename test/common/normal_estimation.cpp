#include <catch2/catch.hpp>
#include <pcp/common/norm.hpp>
#include <pcp/common/normals/normal_estimation.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/vector3d_queries.hpp>

SCENARIO("normal estimation", "[normal_estimation]")
{
    GIVEN("some points in 3d space")
    {
        std::vector<pcp::point_t> points;
        points.push_back({0.f, 0.f, 0.f});
        points.push_back({-2.f, 0.f, 0.f});
        points.push_back({2.f, 0.f, 0.f});
        points.push_back({0.f, -2.f, 0.f});
        points.push_back({0.f, 2.f, 0.f});
        points.push_back({0.f, 0.f, -1.f});
        points.push_back({0.f, 0.f, 1.f});

        WHEN("estimating the normal from those points")
        {
            auto const point_map = [](pcp::point_t const& p) {
                return p;
            };
            auto const normal = pcp::estimate_normal(points.cbegin(), points.cend(), point_map);
            THEN("The estimated normal is correctly computed")
            {
                pcp::normal_t expected_normal{0.f, 0.f, 1.f};
                bool const is_valid_normal =
                    pcp::common::are_vectors_equal(normal, expected_normal) ||
                    pcp::common::are_vectors_equal(normal, -expected_normal);
                REQUIRE(is_valid_normal);
            }
            THEN("The estimated normal has unit norm") 
            {
                auto const norm = pcp::common::norm(normal);
                REQUIRE(pcp::common::floating_point_equals(norm, 1.f));
            }
        }
    }
}