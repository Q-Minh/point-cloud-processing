#include <catch2/catch.hpp>
#include <pcp/common/point_predicates.hpp>
#include <pcp/octree.hpp>

SCENARIO("octree deletion", "[octree]")
{
    auto node_capacity = GENERATE(1u, 2u, 3u, 4u);
    auto max_depth     = GENERATE(1u, 2u, 21u);

    GIVEN("an empty octree and a range of points")
    {
        std::vector<pcp::point_t> points{};

        pcp::point_t const first_point_to_remove{0.1f, 0.1f, 0.1f};
        pcp::point_t const second_point_to_remove{0.1f, -0.1f, 0.1f};
        pcp::point_t const third_point_to_remove{0.1f, -0.1f, -0.1f};

        // points in +x,+y,+z octant
        points.push_back(first_point_to_remove);
        points.push_back({0.2f, 0.2f, 0.2f});
        points.push_back({0.3f, 0.3f, 0.3f});
        points.push_back({0.4f, 0.4f, 0.4f});
        points.push_back({0.5f, 0.5f, 0.5f});
        points.push_back({0.6f, 0.6f, 0.6f});
        points.push_back({0.7f, 0.7f, 0.7f});
        points.push_back({0.8f, 0.8f, 0.8f});
        points.push_back({0.9f, 0.9f, 0.9f});
        // points in -x,+y,+z octant
        points.push_back({-0.1f, 0.1f, 0.1f});
        points.push_back({-0.2f, 0.2f, 0.2f});
        points.push_back({-0.3f, 0.3f, 0.3f});
        points.push_back({-0.4f, 0.4f, 0.4f});
        points.push_back({-0.5f, 0.5f, 0.5f});
        points.push_back({-0.6f, 0.6f, 0.6f});
        points.push_back({-0.7f, 0.7f, 0.7f});
        points.push_back({-0.8f, 0.8f, 0.8f});
        points.push_back({-0.9f, 0.9f, 0.9f});
        // points in +x,-y,+z octant
        points.push_back(second_point_to_remove);
        points.push_back({0.2f, -0.2f, 0.2f});
        points.push_back({0.3f, -0.3f, 0.3f});
        points.push_back({0.4f, -0.4f, 0.4f});
        points.push_back({0.5f, -0.5f, 0.5f});
        points.push_back({0.6f, -0.6f, 0.6f});
        points.push_back({0.7f, -0.7f, 0.7f});
        points.push_back({0.8f, -0.8f, 0.8f});
        points.push_back({0.9f, -0.9f, 0.9f});
        // points in +x,-y,-z octant
        points.push_back(third_point_to_remove);
        points.push_back({0.2f, -0.2f, -0.2f});
        points.push_back({0.3f, -0.3f, -0.3f});
        points.push_back({0.4f, -0.4f, -0.4f});
        points.push_back({0.5f, -0.5f, -0.5f});
        points.push_back({0.6f, -0.6f, -0.6f});
        points.push_back({0.7f, -0.7f, -0.7f});
        points.push_back({0.8f, -0.8f, -0.8f});
        points.push_back({0.9f, -0.9f, -0.9f});

        pcp::octree_parameters_t<pcp::point_t> params;
        params.voxel_grid =
            pcp::axis_aligned_bounding_box_t<pcp::point_t>{{-1.f, -1.f, -1.f}, {1.f, 1.f, 1.f}};
        params.node_capacity = node_capacity;
        params.max_depth     = static_cast<std::uint8_t>(max_depth);

        pcp::octree_t octree(points.cbegin(), points.cend(), params);

        WHEN("removing existing points one at a time")
        {
            auto const is_first_point_to_remove = [&first_point_to_remove](pcp::point_t const& p) {
                return pcp::are_points_equal(p, first_point_to_remove);
            };
            auto const is_second_point_to_remove =
                [&second_point_to_remove](pcp::point_t const& p) {
                    return pcp::are_points_equal(p, second_point_to_remove);
                };
            auto const is_third_point_to_remove = [&third_point_to_remove](pcp::point_t const& p) {
                return pcp::are_points_equal(p, third_point_to_remove);
            };

            auto it = std::find_if(octree.cbegin(), octree.cend(), is_first_point_to_remove);

            auto next = octree.erase(it);

            it = std::find_if(octree.cbegin(), octree.cend(), is_second_point_to_remove);

            next = octree.erase(it);

            it = std::find_if(octree.cbegin(), octree.cend(), is_third_point_to_remove);

            next = octree.erase(it);

            THEN("octree is resized accordingly") { REQUIRE(octree.size() == points.size() - 3u); }
            THEN("points are removed from the octree")
            {
                REQUIRE(std::none_of(octree.cbegin(), octree.cend(), is_first_point_to_remove));
                REQUIRE(std::none_of(octree.cbegin(), octree.cend(), is_second_point_to_remove));
                REQUIRE(std::none_of(octree.cbegin(), octree.cend(), is_third_point_to_remove));
                REQUIRE(next != octree.cend());
            }
        }
        WHEN("removing all points")
        {
            auto it = octree.cbegin();
            while ((it = octree.erase(it)) != octree.cend())
                ;
            THEN("octree is empty")
            {
                REQUIRE(octree.empty());
                WHEN("adding new points")
                {
                    octree.insert(points.cbegin(), points.cend());
                    THEN("octree now contains the new points")
                    {
                        REQUIRE(octree.size() == points.size());
                    }
                }
            }
        }
    }
}
