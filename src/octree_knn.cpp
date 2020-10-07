#include <catch.hpp>

#include <octree.hpp>

SCENARIO("KNN searches on the octree", "[octree]") {
	auto node_capacity = GENERATE(1u, 2u, 3u, 4u);
	auto max_depth     = GENERATE(1u, 3u, 21u);

	GIVEN("an octree with 1 point in each octant") {
		pcp::octree_parameters_t params;
		params.node_capacity = node_capacity;
		params.max_depth = max_depth;
		params.voxel_grid = pcp::axis_aligned_bounding_box_t
		{
			pcp::point_t{ -1.f, -1.f, -1.f },
			pcp::point_t{  1.f,  1.f,  1.f }
		};

		pcp::octree_t octree(params);
		octree.insert({ -.5f, -.5f, -.5f }); // 000
		octree.insert({  .5f, -.5f, -.5f }); // 100
		octree.insert({  .5f,  .5f, -.5f }); // 110
		octree.insert({ -.5f,  .5f, -.5f }); // 010
		octree.insert({ -.5f, -.5f,  .5f }); // 001
		octree.insert({  .5f, -.5f,  .5f }); // 101
		octree.insert({  .5f,  .5f,  .5f }); // 111
		octree.insert({ -.5f,  .5f,  .5f }); // 011

		WHEN("searching for k nearest neighbors with k=1") {
			auto const k = 1u;
			auto const reference111 = pcp::point_t
			{
				.5f, .5f, .5f
			};
			auto const& nearest_neighbors_octant111 = octree.nearest_neighbours(reference111, k);

			auto const reference000 = pcp::point_t
			{
				-.5f, -.5f, -.5f
			};
			auto const& nearest_neighbors_octant000 = octree.nearest_neighbours(reference000, k);

			auto const reference110 = pcp::point_t
			{
				.5f, .5f, -.5f
			};
			auto const& nearest_neighbors_octant110 = octree.nearest_neighbours(reference110, k);

			auto const reference011 = pcp::point_t
			{
				-.5f, .5f, .5f
			};
			auto const& nearest_neighbors_octant011 = octree.nearest_neighbours(reference011, k);

			THEN("returns the nearest neighbor") {
				REQUIRE(nearest_neighbors_octant111.size() == k);
				REQUIRE(*nearest_neighbors_octant111.cbegin() == reference111);

				REQUIRE(nearest_neighbors_octant000.size() == k);
				REQUIRE(*nearest_neighbors_octant000.cbegin() == reference000);

				REQUIRE(nearest_neighbors_octant110.size() == k);
				REQUIRE(*nearest_neighbors_octant110.cbegin() == reference110);

				REQUIRE(nearest_neighbors_octant011.size() == k);
				REQUIRE(*nearest_neighbors_octant011.cbegin() == reference011);
			}
		}
	}

	GIVEN("an octree with 1 point in 7 octants "
		  "and 4 points in the other octant") {
		pcp::octree_parameters_t params;
		params.node_capacity = node_capacity;
		params.max_depth = max_depth;
		params.voxel_grid = pcp::axis_aligned_bounding_box_t
		{
			pcp::point_t{ -1.f, -1.f, -1.f },
			pcp::point_t{  1.f,  1.f,  1.f }
		};

		pcp::octree_t octree(params);

		octree.insert({ -.5f, -.5f, -.5f }); // 000
		octree.insert({ .5f, -.5f, -.5f }); // 100
		octree.insert({ -.5f,  .5f, -.5f }); // 010
		octree.insert({ -.5f, -.5f,  .5f }); // 001
		octree.insert({ .5f, -.5f,  .5f }); // 101
		octree.insert({ .5f,  .5f,  .5f }); // 111
		octree.insert({ -.5f,  .5f,  .5f }); // 011

		pcp::point_t const reference =
		{
			.5f, .5f, -.5f
		};

		pcp::point_t const  first_nearest{ .5f, .5f, -.5f };
		pcp::point_t const second_nearest{ .6f, .5f, -.5f };
		pcp::point_t const  third_nearest{ .4f, .3f, -.5f };
		pcp::point_t const fourth_nearest{ .7f, .2f, -.5f };

		octree.insert(first_nearest); // 110
		octree.insert(second_nearest);
		octree.insert(third_nearest);
		octree.insert(fourth_nearest);

		WHEN("searching for k nearest neighbors with k=4 in the octant with 4 points") {
			auto const k = 4u;
			auto const& nearest_neighbors = octree.nearest_neighbours(reference, k);

			THEN("nearest points are the 4 points in that same octant") {
				REQUIRE(nearest_neighbors.size() == k);
				REQUIRE(nearest_neighbors[0] == first_nearest);
				REQUIRE(nearest_neighbors[1] == second_nearest);
				REQUIRE(nearest_neighbors[2] == third_nearest);
				REQUIRE(nearest_neighbors[3] == fourth_nearest);
			}
		}
		WHEN("searching for k nearest neighbors with k=3 in the octant with 4 points") {
			auto const k = 3u;
			auto const& nearest_neighbors = octree.nearest_neighbours(reference, k);

			THEN("nearest points are the 3 nearest points in that same octant") {
				REQUIRE(nearest_neighbors.size() == k);
				REQUIRE(nearest_neighbors[0] == first_nearest);
				REQUIRE(nearest_neighbors[1] == second_nearest);
				REQUIRE(nearest_neighbors[2] == third_nearest);
			}
		}
	}
	GIVEN("a randomly constructed octree") {
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> coordinate_distribution(-0.95f, 0.95f);
		std::uniform_real_distribution<float> near_coordinate_distribution(-1.f, -0.96f);
		std::uniform_real_distribution<float> far_coordinate_distribution(0.96f, 1.f);
		std::uniform_int_distribution<>       size_distribution(1'000, 100'000);
		std::uniform_int_distribution<>       k_distribution(1u, 10u);

		pcp::octree_parameters_t params;
		params.node_capacity = node_capacity;
		params.max_depth = max_depth;
		params.voxel_grid = pcp::axis_aligned_bounding_box_t
		{
			pcp::point_t{ -2.f, -2.f, -2.f },
			pcp::point_t{  2.f,  2.f,  2.f }
		};

		pcp::octree_t octree(params);

		auto const size = size_distribution(gen);
		for (auto i = 0; i < size; ++i)
		{
			octree.insert(pcp::point_t{
				coordinate_distribution(gen),
				coordinate_distribution(gen),
				coordinate_distribution(gen)
				});
		}

		REQUIRE(octree.size() == size);

		WHEN("inserting in the octree k nearest points to the reference point") {
			pcp::point_t const reference =
			{
				-1.f, 1.f, 1.f
			};
			std::vector<pcp::point_t> k_inserted_points;

			auto const k = k_distribution(gen);
			for (auto i = 0; i < k; ++i)
			{
				k_inserted_points.push_back(
					pcp::point_t
					{
						near_coordinate_distribution(gen),
						far_coordinate_distribution(gen),
						far_coordinate_distribution(gen)
					}
				);
			}

			octree.insert(k_inserted_points.cbegin(), k_inserted_points.cend());

			THEN("k nearest neighbour search returns those k points") {
				std::vector<pcp::point_t> nearest_neighbours = octree.nearest_neighbours(reference, k);

				REQUIRE(nearest_neighbours.size() == k);

				auto const contains = [&k_inserted_points](pcp::point_t const& p) -> bool
				{
					return std::find(k_inserted_points.cbegin(), k_inserted_points.cend(), p) != k_inserted_points.cend();
				};

				bool const found_nearest_points = std::all_of(
					nearest_neighbours.cbegin(), nearest_neighbours.cend(),
					contains
				);

				REQUIRE(found_nearest_points);
			}
		}
	}
}
