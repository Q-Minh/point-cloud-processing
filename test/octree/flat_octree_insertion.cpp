#include "pcp/common/points/point.hpp"

#include <catch2/catch.hpp>
#include <iostream>
#include "pcp/octree/flat_octree.hpp"

SCENARIO("flat_octree insertion", "[flat_octree]")
{
    auto const point_map = [](pcp::point_t const& p) {
        return p;
    };

    auto const previous_size = points.size();

    std::vector<pcp::point_t> points{};

    points.push_back({0.6f, 0.6f, 0.6f});
    points.push_back({0.7f, 0.7f, 0.7f});
    points.push_back({0.8f, 0.8f, 0.8f});
    points.push_back({0.9f, 0.9f, 0.9f});
    points.push_back({1.0f, 2.0f, 3.0f});
    points.push_back({2.0f, 3.0f, 4.0f});
    points.push_back({0.1f, 0.1f, 0.1f});
    points.push_back({0.2f, 0.2f, 0.2f});
    points.push_back({0.3f, 0.3f, 0.3f});
    points.push_back({0.4f, 0.4f, 0.4f});
    points.push_back({0.5f, 0.5f, 0.5f});

    pcp::flat_octree_parameters_t<pcp::point_t> params;
    params.voxel_grid =
        pcp::axis_aligned_bounding_box_t<pcp::point_t>{{0.f, 0.f, 0.f}, {5.f, 5.f, 5.f}};
    params.depth = 8u;

    pcp::flat_octree_t octree(points.cbegin(), points.cend(), point_map, params);
    auto size    = octree.size();

    std::cout << "Expected size: " << previous_size << std::endl;
    std::cout << "Got size: " << size << std::endl;
}
