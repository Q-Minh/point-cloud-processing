#include <catch2/catch.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/normals/tangent_plane.hpp>

SCENARIO("3d tangent planes", "[tangent_plane]") 
{
    GIVEN("a point and a normal") 
    { 
        pcp::point_t p{110.f, -120.f, 0.f};
        pcp::normal_t n{0.f, 0.f, 1.f};

        WHEN("constructing a tangent plane from them") 
        {
            pcp::common::plane3d_t plane{p, n};
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
}