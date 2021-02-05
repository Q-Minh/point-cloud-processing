#include "pcp/common/points/point.hpp"

#include <catch2/catch.hpp>
#include <iostream>
#include <pcp/kdtree/kdtree.hpp>

SCENARIO("kdtree insertion", "[kdtree]")
{
    using element_type     = std::array<std::uint32_t, 3u>;
    using coordinates_type = std::array<std::uint32_t, 3u>;

    std::vector<element_type>
        a{{5u, 4u, 5u}, {3u, 5u, 2u}, {2u, 5u, 5u}, {3u, 3u, 3u}, {4u, 5u, 1u}, {5u, 5u, 1u}};

    auto const coordinates_map = [](element_type const& e) -> coordinates_type {
        return coordinates_type{e};
    };

    auto const less_than = [&](element_type const& e1, element_type const& e2) -> bool {
        coordinates_type coordinates1 = coordinates_map(e1);
        coordinates_type coordinates2 = coordinates_map(e2);

        for (std::uint64_t i = 0u; i < coordinates1.size(); ++i)
        {
            if (coordinates1[i] < coordinates2[i])
                return true;
            if (coordinates1[i] > coordinates2[i])
                return false;
        }
        return false;
    };

    std::sort(a.begin(), a.end(), less_than);

    std::vector<pcp::point_t> points{};
    pcp::point_t first({0.1f, 0.2f, 0.3f});

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

    // pcp::kdtree_t kd{std::begin(points), std::end(points), 3};
    // int size = kd.size();
    // kd.print_in_order();

    // std::cout << "nb of nodes " << size << '\n';
    // std::cout << "nb of points " << points.size() << '\n';
}
