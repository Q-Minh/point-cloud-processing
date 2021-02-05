#include <catch2/catch.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/plane3d.hpp>
#include <pcp/common/points/point.hpp>

SCENARIO("3d planes", "[plane3d]")
{
    GIVEN("a point and a normal")
    {
        pcp::point_t p{110.f, -120.f, 0.f};
        pcp::normal_t n{0.f, 0.f, 1.f};

        WHEN("constructing a tangent plane from them")
        {
            pcp::common::basic_plane3d_t plane{p, n};
            THEN("we can compute signed distances to other points")
            {
                auto sd = plane.signed_distance_to(pcp::point_t{0.f, 0.f, 1.f});
                REQUIRE(pcp::common::floating_point_equals(sd, 1.f));
                sd = plane.signed_distance_to(pcp::point_t{1301.f, 3123.23f, -1.f});
                REQUIRE(pcp::common::floating_point_equals(sd, -1.f));
                sd = plane.signed_distance_to(pcp::point_t{1301.f, 3123.23f, 0.f});
                REQUIRE(pcp::common::floating_point_equals(sd, 0.f));
            }
            THEN("we can query containment in the plane")
            {
                REQUIRE(plane.contains(pcp::point_t{0.f, 0.f, 0.f}));
                REQUIRE(plane.contains(pcp::point_t{-01230912.f, 124091.f, 0.f}));
                REQUIRE(!plane.contains(pcp::point_t{0.f, 0.f, -1.f}));
            }
        }
    }
    GIVEN("scattered 3d points")
    {
        std::vector<pcp::point_t> points;
        points.push_back({0.f, 0.f, 0.f});
        points.push_back({-2.f, 0.f, 0.f});
        points.push_back({2.f, 0.f, 0.f});
        points.push_back({0.f, -2.f, 0.f});
        points.push_back({0.f, 2.f, 0.f});
        points.push_back({0.f, 0.f, -1.f});
        points.push_back({0.f, 0.f, 1.f});

        WHEN("estimating their tangent plane")
        {
            auto const point_map = [](pcp::point_t const& p) {
                return p;
            };

            auto const tangent_plane = pcp::common::tangent_plane(points.cbegin(), points.cend(), point_map);

            THEN("the tangent plane has correct normal and point") 
            {
                auto const& center = tangent_plane.point();
                auto const& normal = tangent_plane.normal();

                auto const expected_center = pcp::point_t{0.f, 0.f, 0.f};
                auto const expected_normal = pcp::normal_t{0.f, 0.f, 1.f};

                bool const is_center_valid =
                    pcp::common::are_vectors_equal(center, expected_center);
                bool const is_normal_valid =
                    pcp::common::are_vectors_equal(normal, expected_normal) ||
                    pcp::common::are_vectors_equal(normal, -expected_normal);

                REQUIRE(is_center_valid);
                REQUIRE(is_normal_valid);
            }
        }
    }
}