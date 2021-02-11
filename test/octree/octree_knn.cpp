#include <catch2/catch.hpp>
#include <pcp/octree/linked_octree.hpp>

SCENARIO("KNN searches on the octree", "[octree]")
{
    auto node_capacity = GENERATE(1u, 2u, 3u, 4u);
    auto max_depth     = GENERATE(1u, 3u, 21u);

    auto const point_map = [](pcp::point_t const& p) {
        return p;
    };

    GIVEN("an octree with 1 point in each octant")
    {
        pcp::octree_parameters_t<pcp::point_t> params;
        params.node_capacity = node_capacity;
        params.max_depth     = static_cast<std::uint8_t>(max_depth);
        params.voxel_grid    = pcp::axis_aligned_bounding_box_t<pcp::point_t>{
            pcp::point_t{-1.f, -1.f, -1.f},
            pcp::point_t{1.f, 1.f, 1.f}};

        pcp::linked_octree_t octree(params);
        octree.insert({-.5f, -.5f, -.5f}, point_map); // 000
        octree.insert({.5f, -.5f, -.5f}, point_map);  // 100
        octree.insert({.5f, .5f, -.5f}, point_map);   // 110
        octree.insert({-.5f, .5f, -.5f}, point_map);  // 010
        octree.insert({-.5f, -.5f, .5f}, point_map);  // 001
        octree.insert({.5f, -.5f, .5f}, point_map);   // 101
        octree.insert({.5f, .5f, .5f}, point_map);    // 111
        octree.insert({-.5f, .5f, .5f}, point_map);   // 011

        WHEN("searching for k nearest neighbors with k=1")
        {
            auto const k = 1u;

            auto const reference111 = pcp::point_t{.51f, .51f, .51f};
            auto const& nearest_neighbors_octant111 =
                octree.nearest_neighbours(reference111, k, point_map);

            auto const reference000 = pcp::point_t{-.51f, -.51f, -.51f};
            auto const& nearest_neighbors_octant000 =
                octree.nearest_neighbours(reference000, k, point_map);

            auto const reference110 = pcp::point_t{.51f, .51f, -.51f};
            auto const& nearest_neighbors_octant110 =
                octree.nearest_neighbours(reference110, k, point_map);

            auto const reference011 = pcp::point_t{-.51f, .51f, .51f};
            auto const& nearest_neighbors_octant011 =
                octree.nearest_neighbours(reference011, k, point_map);

            THEN("returns the nearest neighbor")
            {
                REQUIRE(nearest_neighbors_octant111.size() == k);
                REQUIRE(nearest_neighbors_octant000.size() == k);
                REQUIRE(nearest_neighbors_octant110.size() == k);
                REQUIRE(nearest_neighbors_octant011.size() == k);
            }
        }
    }

    GIVEN(
        "an octree with 1 point in 7 octants "
        "and 4 points in the other octant")
    {
        pcp::octree_parameters_t<pcp::point_t> params;
        params.node_capacity = node_capacity;
        params.max_depth     = static_cast<std::uint8_t>(max_depth);
        params.voxel_grid    = pcp::axis_aligned_bounding_box_t<pcp::point_t>{
            pcp::point_t{-1.f, -1.f, -1.f},
            pcp::point_t{1.f, 1.f, 1.f}};

        pcp::linked_octree_t octree(params);

        octree.insert({-.5f, -.5f, -.5f}, point_map); // 000
        octree.insert({.5f, -.5f, -.5f}, point_map);  // 100
        octree.insert({-.5f, .5f, -.5f}, point_map);  // 010
        octree.insert({-.5f, -.5f, .5f}, point_map);  // 001
        octree.insert({.5f, -.5f, .5f}, point_map);   // 101
        octree.insert({.5f, .5f, .5f}, point_map);    // 111
        octree.insert({-.5f, .5f, .5f}, point_map);   // 011

        pcp::point_t const reference = {.5f, .5f, -.5f};

        pcp::point_t const first_nearest{.51f, .51f, -.51f};
        pcp::point_t const second_nearest{.61f, .51f, -.51f};
        pcp::point_t const third_nearest{.41f, .31f, -.51f};
        pcp::point_t const fourth_nearest{.71f, .21f, -.51f};

        octree.insert(first_nearest, point_map); // 110
        octree.insert(second_nearest, point_map);
        octree.insert(third_nearest, point_map);
        octree.insert(fourth_nearest, point_map);

        WHEN("searching for k nearest neighbors with k=4 in the octant with 4 points")
        {
            auto const k                  = 4u;
            auto const& nearest_neighbors = octree.nearest_neighbours(reference, k, point_map);

            THEN("nearest points are the 4 points in that same octant")
            {
                REQUIRE(nearest_neighbors.size() == k);
                for (auto const& neighbor :
                     {first_nearest, second_nearest, third_nearest, fourth_nearest})
                {
                    bool const has_nearest_neighbour =
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[0]) ||
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[1]) ||
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[2]) ||
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[3]);

                    REQUIRE(has_nearest_neighbour);
                }
            }
        }
        WHEN("searching for k nearest neighbors with k=3 in the octant with 4 points")
        {
            auto const k                  = 3u;
            auto const& nearest_neighbors = octree.nearest_neighbours(reference, k, point_map);

            THEN("nearest points are the 3 nearest points in that same octant")
            {
                REQUIRE(nearest_neighbors.size() == k);
                for (auto const& neighbor : {first_nearest, second_nearest, third_nearest})
                {
                    bool const has_nearest_neighbour =
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[0]) ||
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[1]) ||
                        pcp::common::are_vectors_equal(neighbor, nearest_neighbors[2]);

                    REQUIRE(has_nearest_neighbour);
                }
            }
        }
    }
    GIVEN("a randomly constructed octree")
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> coordinate_distribution(-0.95f, 0.95f);
        std::uniform_real_distribution<float> near_coordinate_distribution(-.99f, -0.96f);
        std::uniform_real_distribution<float> far_coordinate_distribution(0.96f, .99f);
        std::uniform_int_distribution<std::size_t> size_distribution(1'000, 100'000);
        std::uniform_int_distribution<std::size_t> k_distribution(1u, 10u);

        pcp::octree_parameters_t<pcp::point_t> params;
        params.node_capacity = node_capacity;
        params.max_depth     = static_cast<std::uint8_t>(max_depth);
        params.voxel_grid    = pcp::axis_aligned_bounding_box_t<pcp::point_t>{
            pcp::point_t{-2.f, -2.f, -2.f},
            pcp::point_t{2.f, 2.f, 2.f}};

        pcp::linked_octree_t octree(params);

        auto const size = size_distribution(gen);
        for (std::size_t i = 0; i < size; ++i)
        {
            octree.insert(
                pcp::point_t{
                    coordinate_distribution(gen),
                    coordinate_distribution(gen),
                    coordinate_distribution(gen)},
                point_map);
        }

        REQUIRE(octree.size() == size);

        WHEN("inserting in the octree k nearest points to the reference point")
        {
            pcp::point_t const reference = {-1.f, 1.f, 1.f};
            std::vector<pcp::point_t> k_inserted_points;

            auto const k = k_distribution(gen);
            for (std::size_t i = 0; i < k; ++i)
            {
                k_inserted_points.push_back(pcp::point_t{
                    near_coordinate_distribution(gen),
                    far_coordinate_distribution(gen),
                    far_coordinate_distribution(gen)});
            }

            octree.insert(k_inserted_points.cbegin(), k_inserted_points.cend(), point_map);

            THEN("k nearest neighbour search returns those k points")
            {
                std::vector<pcp::point_t> nearest_neighbours =
                    octree.nearest_neighbours(reference, k, point_map);

                REQUIRE(nearest_neighbours.size() == k);

                auto const contains = [&k_inserted_points](pcp::point_t const& p) -> bool {
                    return std::find_if(
                               k_inserted_points.cbegin(),
                               k_inserted_points.cend(),
                               [&p](auto const& other) {
                                   return pcp::common::are_vectors_equal(p, other);
                               }) != k_inserted_points.cend();
                };

                bool const found_nearest_points =
                    std::all_of(nearest_neighbours.cbegin(), nearest_neighbours.cend(), contains);

                REQUIRE(found_nearest_points);
            }
        }
    }
}
