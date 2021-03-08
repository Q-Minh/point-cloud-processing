#include <catch2/catch.hpp>
#include "pcp/octree/flat_octree.hpp"
#include <iostream>

SCENARIO("flat_octree insertion", "[flat_octree]")
{
    auto depth = static_cast<std::uint8_t>(GENERATE(1u, 4u, 9u, 13u, 17u, 21u));

    auto const insertion = [](pcp::flat_octree_parameters_t<pcp::point_t> const& params,
                              std::vector<pcp::point_t>& points) {
        auto const point_map = [](pcp::point_t const& p) {
            return p;
        };

        WHEN("inserting a range of points contained in the flat octree's voxel grid") 
        {
            pcp::flat_octree_t octree(points.cbegin(), points.cend(), point_map, params);

            THEN("the octree's size is the size of the range") 
            {
                REQUIRE(octree.size() == points.size());
            }
        }
        WHEN(
            "inserting a range of points contained or not contained "
            "in the flat octree's voxel grid")
        {
            auto const previous_size = points.size();
            points.push_back({-2.f, 0.f, 0.f});
            points.push_back({0.f, -2.f, 0.f});
            points.push_back({0.f, 0.f, -2.f});
            points.push_back({2.f, 0.f, 0.f});
            points.push_back({0.f, 2.f, 0.f});
            points.push_back({0.f, 0.f, 2.f});

            pcp::flat_octree_t octree(points.cbegin(), points.cend(), point_map, params);

            THEN(
                "the octree's size is the number of points in the "
                "range contained in flat octree's voxel grid")
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

        pcp::flat_octree_parameters_t<pcp::point_t> params;
        params.voxel_grid =
            pcp::axis_aligned_bounding_box_t<pcp::point_t>{{-1.f, -1.f, -1.f}, {1.f, 1.f, 1.f}};

        WHEN("the flat octree's depth is between 1 and 21")
        {
            params.depth = depth;
            insertion(params, points);
        }
    }
}
