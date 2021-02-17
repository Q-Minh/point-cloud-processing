#include <catch2/catch.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/vector3d.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/kdtree/linked_kdtree.hpp>

SCENARIO("kdtree range search", "[kdtree]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    using kdtree_type    = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    auto const max_depth = GENERATE(1u, 2u, 4u, 8u, 12u);

    GIVEN("a kdtree with 1 point in each octant")
    {
        pcp::kdtree::construction_params_t params;
        params.construction = pcp::kdtree::construction_t::nth_element;
        params.max_depth    = max_depth;

        std::vector<pcp::point_t> points;
        points.push_back({-.5f, -.5f, -.5f}); // 000
        points.push_back({.5f, -.5f, -.5f});  // 100
        points.push_back({.5f, .5f, -.5f});   // 110
        points.push_back({-.5f, .5f, -.5f});  // 010
        points.push_back({-.5f, -.5f, .5f});  // 001
        points.push_back({.5f, -.5f, .5f});   // 101
        points.push_back({.5f, .5f, .5f});    // 111
        points.push_back({-.5f, .5f, .5f});   // 011

        points.push_back({-.4f, -.3f, -.6f}); // 000
        points.push_back({.4f, -.3f, -.6f});  // 100
        points.push_back({.4f, .3f, -.6f});   // 110
        points.push_back({-.4f, .3f, -.6f});  // 010
        points.push_back({-.4f, -.3f, .6f});  // 001
        points.push_back({.4f, -.3f, .6f});   // 101
        points.push_back({.4f, .3f, .6f});    // 111
        points.push_back({-.4f, .3f, .6f});   // 011

        kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
        WHEN("searching for points that are not contained in the queried sphere")
        {
            pcp::sphere_a<float> sphere;
            sphere.position[0] = 0.0f;
            sphere.position[1] = 0.0f;
            sphere.position[2] = 0.0f;
            sphere.radius      = 0.1f;

            auto const points_in_range = kdtree.range_search(sphere);
            THEN("no points are found") { REQUIRE(points_in_range.empty()); }
        }

        WHEN("searching for points that are contained in the queried sphere")
        {
            pcp::sphere_a<float> sphere;
            sphere.position[0] = .9f;
            sphere.position[1] = .9f;
            sphere.position[2] = .9f;
            sphere.radius      = 1.f;

            auto const points_in_range = kdtree.range_search(sphere);
            THEN("the points in the range are returned")
            {
                REQUIRE(points_in_range.size() == 2u);
                REQUIRE(
                    std::count_if(
                        points_in_range.cbegin(),
                        points_in_range.cend(),
                        [](auto const& p) {
                            return pcp::common::are_vectors_equal(p, pcp::point_t{.5f, .5f, .5f});
                        }) == 1u);
                REQUIRE(
                    std::count_if(
                        points_in_range.cbegin(),
                        points_in_range.cend(),
                        [](auto const& p) {
                            return pcp::common::are_vectors_equal(p, pcp::point_t{.4f, .3f, .6f});
                        }) == 1u);
            }
        }

        WHEN("searching for points that are not contained in the queried aabb")
        {
            pcp::kd_axis_aligned_bounding_box_t<float, 3u> aabb;
            aabb.min = {1.05f, 1.05f, 1.05f};
            aabb.max = {2.f, 2.f, 2.f};

            auto const points_in_range = kdtree.range_search(aabb);
            THEN("no points are found") { REQUIRE(points_in_range.size() == 0u); }
        }
        WHEN("searching for points that are in the queried aabb")
        {
            pcp::kd_axis_aligned_bounding_box_t<float, 3u> aabb;
            aabb.min = {-2.f, -2.f, -2.f};
            aabb.max = {0.f, 0.f, 0.f};

            auto const points_in_range = kdtree.range_search(aabb);
            THEN("the points in the range are returned")
            {
                REQUIRE(points_in_range.size() == 2u);
                REQUIRE(
                    std::count_if(
                        points_in_range.cbegin(),
                        points_in_range.cend(),
                        [](auto const& p) {
                            return pcp::common::are_vectors_equal(
                                pcp::point_t{-.5f, -.5f, -.5f},
                                p);
                        }) == 1u);
                REQUIRE(
                    std::count_if(
                        points_in_range.cbegin(),
                        points_in_range.cend(),
                        [](auto const& p) {
                            return pcp::common::are_vectors_equal(
                                p,
                                pcp::point_t{-.4f, -.3f, -.6f});
                        }) == 1u);
            }
        }
    }
}
