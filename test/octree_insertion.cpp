#include <catch2/catch.hpp>
#include <pcp/octree.hpp>

SCENARIO("octree insertion", "[octree]")
{
    auto const insertion = [](pcp::octree_parameters_t const& params,
                              std::vector<pcp::point_t>& points) {
        WHEN("inserting a range of points contained in the octree's voxel grid")
        {
            pcp::octree_t octree(points.cbegin(), points.cend(), params);

            THEN("the octree's size is the size of the range")
            {
                REQUIRE(octree.size() == points.size());
            }
        }
        WHEN(
            "inserting a range of points contained or not "
            "contained in the octree's voxel grid")
        {
            auto const previous_size = points.size();
            points.push_back({-2.f, 0.f, 0.f});
            points.push_back({0.f, -2.f, 0.f});
            points.push_back({0.f, 0.f, -2.f});
            points.push_back({2.f, 0.f, 0.f});
            points.push_back({0.f, 2.f, 0.f});
            points.push_back({0.f, 0.f, 2.f});

            pcp::octree_t octree(points.cbegin(), points.cend(), params);

            THEN(
                "the octree's size is the number of points of the"
                "range contained in the octree's voxel grid")
            {
                REQUIRE(octree.size() == previous_size);
            }
        }
    };

    GIVEN("an empty octree and a range of points")
    {
        std::vector<pcp::point_t> points{};

        // points in +x,+y,+z octant
        points.push_back({0.1f, 0.1f, 0.1f});
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
        points.push_back({0.1f, -0.1f, 0.1f});
        points.push_back({0.2f, -0.2f, 0.2f});
        points.push_back({0.3f, -0.3f, 0.3f});
        points.push_back({0.4f, -0.4f, 0.4f});
        points.push_back({0.5f, -0.5f, 0.5f});
        points.push_back({0.6f, -0.6f, 0.6f});
        points.push_back({0.7f, -0.7f, 0.7f});
        points.push_back({0.8f, -0.8f, 0.8f});
        points.push_back({0.9f, -0.9f, 0.9f});
        // points in +x,-y,-z octant
        points.push_back({0.1f, -0.1f, -0.1f});
        points.push_back({0.2f, -0.2f, -0.2f});
        points.push_back({0.3f, -0.3f, -0.3f});
        points.push_back({0.4f, -0.4f, -0.4f});
        points.push_back({0.5f, -0.5f, -0.5f});
        points.push_back({0.6f, -0.6f, -0.6f});
        points.push_back({0.7f, -0.7f, -0.7f});
        points.push_back({0.8f, -0.8f, -0.8f});
        points.push_back({0.9f, -0.9f, -0.9f});

        pcp::octree_parameters_t params;
        params.voxel_grid = pcp::axis_aligned_bounding_box_t{
            pcp::point_t{-1.f, -1.f, -1.f},
            pcp::point_t{1.f, 1.f, 1.f}};

        WHEN("the octree's node capacity = 1")
        {
            params.node_capacity = 1;

            WHEN("the octree's max depth = 3")
            {
                params.max_depth = 3;
                insertion(params, points);
            }
            WHEN("the octree's max depth = 1")
            {
                params.max_depth = 1;
                insertion(params, points);
            }
            WHEN("the octree's max depth = 21")
            {
                params.max_depth = 3;
                insertion(params, points);
            }
        }
        WHEN("the octree's node capacity = 7")
        {
            params.node_capacity = 7;

            WHEN("the octree's max depth = 3")
            {
                params.max_depth = 3;
                insertion(params, points);
            }
            WHEN("the octree's max depth = 1")
            {
                params.max_depth = 1;
                insertion(params, points);
            }
            WHEN("the octree's max depth = 21")
            {
                params.max_depth = 21;
                insertion(params, points);
            }
        }
    }
}
