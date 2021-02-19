#include "pcp/common/vector3d_queries.hpp"

#include <catch2/catch.hpp>
#include <pcp/common/axis_aligned_bounding_box.hpp>
#include <pcp/common/points/point.hpp>

SCENARIO("axis aligned bounding boxes", "[aabb]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    GIVEN("Points that are all negative, their maximum should be also negative")
    {
        std::vector<pcp::point_t> points{};
        pcp::point_t p00{-0.1f, -0.1f, -0.1f};
        pcp::point_t p01{-0.2f, -0.2f, -0.2f};

        pcp::point_t p10{-2.0f, -2.0f, -2.0f};
        pcp::point_t p11{-2.2f, -2.2f, -2.2f};

        points.assign({p00, p01, p10, p11});
        auto begin = points.begin();
        auto end   = points.end();

        WHEN("computing its kd aabb")
        {
            auto const aabb =
                pcp::kd_bounding_box<float, 3u, decltype(coordinate_map), decltype(begin)>(
                    begin,
                    end,
                    coordinate_map);

            THEN("containment queries are valid")
            {
                REQUIRE(!aabb.contains({2.1f, 2.1f, 2.1f}));
                REQUIRE(aabb.contains({-0.1f, -0.1f, -0.1f}));
                REQUIRE(!aabb.contains({0.0f, 0.0f, 0.0f}));
            }

            using aabb_point_type        = typename decltype(aabb)::point_type;
            auto const are_points_equals = [](aabb_point_type const& p1,
                                              aabb_point_type const& p2) {
                return pcp::common::floating_point_equals(p1[0], p2[0]) &&
                       pcp::common::floating_point_equals(p1[1], p2[1]) &&
                       pcp::common::floating_point_equals(p1[2], p2[2]);
            };
            THEN("nearest point from queries are valid")
            {
                auto nearest_point = aabb.nearest_point_from({-3.0f, -3.0f, -3.0f});
                REQUIRE(are_points_equals(nearest_point, {-2.2f, -2.2f, -2.2f}));
                nearest_point = aabb.nearest_point_from({0.0f, 0.0f, 0.0f});
                REQUIRE(are_points_equals(nearest_point, {-0.1f, -0.1f, -0.1f}));
            }
        }
    }
    GIVEN("Points that are all positive, their minimum should be also positive")
    {
        std::vector<pcp::point_t> points{};
        pcp::point_t p00{0.1f, 0.1f, 0.1f};
        pcp::point_t p01{0.2f, 0.2f, 0.2f};

        pcp::point_t p10{2.0f, 2.0f, 2.0f};
        pcp::point_t p11{2.2f, 2.2f, 2.2f};

        points.assign({p00, p01, p10, p11});
        auto begin = points.begin();
        auto end   = points.end();

        WHEN("computing its kd aabb")
        {
            auto const aabb =
                pcp::kd_bounding_box<float, 3u, decltype(coordinate_map), decltype(begin)>(
                    begin,
                    end,
                    coordinate_map);

            THEN("containment queries are valid")
            {
                REQUIRE(!aabb.contains({2.3f, 2.3f, 2.3f}));
                REQUIRE(aabb.contains({0.1f, 0.1f, 0.1f}));
                REQUIRE(!aabb.contains({0.0f, 0.0f, 0.0f}));
            }

            using aabb_point_type        = typename decltype(aabb)::point_type;
            auto const are_points_equals = [](aabb_point_type const& p1,
                                              aabb_point_type const& p2) {
                return pcp::common::floating_point_equals(p1[0], p2[0]) &&
                       pcp::common::floating_point_equals(p1[1], p2[1]) &&
                       pcp::common::floating_point_equals(p1[2], p2[2]);
            };
            THEN("nearest point from queries are valid")
            {
                auto nearest_point = aabb.nearest_point_from({3.0f, 3.0f, 3.0f});
                REQUIRE(are_points_equals(nearest_point, {2.2f, 2.2f, 2.2f}));
                nearest_point = aabb.nearest_point_from({0.0f, 0.0f, 0.0f});
                REQUIRE(are_points_equals(nearest_point, {0.1f, 0.1f, 0.1f}));
            }
        }
    }

    GIVEN(
        "A kd point cloud with k=3 and with 2 points in each octant of a [0, 4]^3 grid and bounds "
        "(.1, .1, .1) to (2.8, 2.8, 2.8)")
    {
        /**
         * Points live in a 4x4x4 grid [0, 4]^3
         * Octants of this grid are numbered from 0 to 7
         * in counter clockwise fashion around the z-axis,
         * and ascending in the y direction,
         * with coordinate frame:
         *
         *     z  y
         *     o o
         *     |/
         *     o---o x
         *
         * So octants 0, 1, 2, 3 are all octants around the z-axis
         * in counterclockwise order at y = [0, 2], and
         * octants 4, 5, 6, 7 are all octants around the z-axis
         * in counterclockwise order at y = [2, 4]
         *
         */
        std::vector<pcp::point_t> points{};
        pcp::point_t p00{0.1f, 0.1f, 0.1f};
        pcp::point_t p01{0.2f, 0.2f, 0.2f};

        pcp::point_t p10{2.1f, 0.3f, 0.3f};
        pcp::point_t p11{2.2f, 0.4f, 0.4f};

        pcp::point_t p20{2.3f, 2.1f, 0.5f};
        pcp::point_t p21{2.4f, 2.2f, 0.6f};

        pcp::point_t p30{0.3f, 2.3f, 0.7f};
        pcp::point_t p31{0.4f, 2.4f, 0.8f};

        pcp::point_t p40{0.5f, 0.5f, 2.1f};
        pcp::point_t p41{0.6f, 0.6f, 2.2f};

        pcp::point_t p50{2.5f, 0.7f, 2.3f};
        pcp::point_t p51{2.6f, 0.8f, 2.4f};

        pcp::point_t p60{2.7f, 2.5f, 2.5f};
        pcp::point_t p61{2.8f, 2.6f, 2.6f};

        pcp::point_t p70{0.7f, 2.7f, 2.7f};
        pcp::point_t p71{0.8f, 2.8f, 2.8f};

        points.assign(
            {p00, p10, p20, p30, p40, p50, p60, p70, p01, p11, p21, p31, p41, p51, p61, p71});

        auto begin = points.begin();
        auto end   = points.end();

        WHEN("computing its kd aabb")
        {
            auto const aabb =
                pcp::kd_bounding_box<float, 3u, decltype(coordinate_map), decltype(begin)>(
                    begin,
                    end,
                    coordinate_map);

            THEN("containment queries are valid")
            {
                REQUIRE(!aabb.contains({2.1f, -0.1f, 1.f}));
                REQUIRE(aabb.contains({0.1f, 0.4f, 0.3f}));
                REQUIRE(!aabb.contains({0.1f, 0.3f, 3.f}));
                REQUIRE(aabb.contains({0.1f, 0.1f, 2.8f}));
            }

            using aabb_point_type        = typename decltype(aabb)::point_type;
            auto const are_points_equals = [](aabb_point_type const& p1,
                                              aabb_point_type const& p2) {
                return pcp::common::floating_point_equals(p1[0], p2[0]) &&
                       pcp::common::floating_point_equals(p1[1], p2[1]) &&
                       pcp::common::floating_point_equals(p1[2], p2[2]);
            };

            THEN("nearest point from queries are valid")
            {
                auto nearest_point = aabb.nearest_point_from({-1.f, 0.1f, 0.1f});
                REQUIRE(are_points_equals(nearest_point, {.1f, .1f, .1f}));
                nearest_point = aabb.nearest_point_from({-1.f, 0.1f, 4.f});
                REQUIRE(are_points_equals(nearest_point, {.1f, .1f, 2.8f}));
                nearest_point = aabb.nearest_point_from({2.f, -1.1f, 0.1f});
                REQUIRE(are_points_equals(nearest_point, {2.f, .1f, .1f}));
                nearest_point = aabb.nearest_point_from({-1.f, 4.1f, -.1f});
                REQUIRE(are_points_equals(nearest_point, {.1f, 2.8f, .1f}));
            }
        }
    }
}
