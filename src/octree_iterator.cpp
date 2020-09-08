#include <catch.hpp>

#include <octree.hpp>

SCENARIO("octree iterators are valid LegacyForwardIterator types", "[octree]") {
	auto offset = GENERATE(1.f, 2.f);
	auto node_capacity = GENERATE(1u, 2u, 4u);
	auto max_depth = GENERATE(1u, 2u, 21u);

	std::vector<point_t> points{};

	points.push_back({ -(offset + 0.9f), -(offset + 0.9f), -(offset + 0.9f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f) });
	points.push_back({ -(offset + 0.1f), -(offset + 0.1f), -(offset + 0.1f) });
	points.push_back({   offset + 0.1f ,   offset + 0.1f ,   offset + 0.1f  });

	point_t const test_point{ offset + 0.2f, offset + 0.2f ,offset + 0.2f };
	auto test_point_count = points.size();
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	points.push_back(test_point);
	test_point_count = points.size() - test_point_count;

	points.push_back({   offset + 0.9f ,   offset + 0.9f ,   offset + 0.9f  });

	octree_parameters_t params;
	params.voxel_grid = axis_aligned_bounding_box_t
	{
		point_t{ -(offset + 1.f), -(offset + 1.f), -(offset + 1.f) },
		point_t{   offset + 1.f ,   offset + 1.f ,   offset + 1.f }
	};
	params.node_capacity = node_capacity;
	params.max_depth     = max_depth;

	GIVEN("an octree") {
		octree_t octree(points.cbegin(), points.cend(), params);

		WHEN("using its octree iterators") {
			auto const begin = octree.cbegin();
			auto const end   = octree.cend();

			THEN("std::distance is valid") {
				auto const size = std::distance(begin, end);
				REQUIRE(points.size() == size);
				REQUIRE(octree.size() == size);
			}
			THEN("non-modifying sequence operations from stl algorithms are valid") {
				float const erroneous_coordinate = 100.f;
				REQUIRE(
					std::none_of(begin, end, [t = erroneous_coordinate](point_t const& p) { return p.x == t || p.y == t || p.z == t; })
				);
				REQUIRE(
					std::all_of(begin, end, [t = erroneous_coordinate](point_t const& p) { return p.x < t&& p.y < t&& p.z < t; })
				);

				auto const vector_sum = std::accumulate(points.cbegin(), points.cend(), point_t{ 0.f, });
				auto const octree_sum = std::accumulate(begin, end, point_t{ 0.f, });

				REQUIRE(vector_sum == octree_sum);

				auto const xless = [](point_t const& p1, point_t const& p2) -> bool { return p1.x < p2.x; };
				auto const yless = [](point_t const& p1, point_t const& p2) -> bool { return p1.y < p2.y; };
				auto const zless = [](point_t const& p1, point_t const& p2) -> bool { return p1.z < p2.z; };

				auto const vector_minmax_point_x = std::minmax_element(
					points.cbegin(), points.cend(), 
					xless
				);
				auto const vector_minmax_point_y = std::minmax_element(
					points.cbegin(), points.cend(),
					yless
				);
				auto const vector_minmax_point_z = std::minmax_element(
					points.cbegin(), points.cend(),
					zless
				);

				auto const octree_minmax_point_x = std::minmax_element(
					begin, end,
					xless
				);
				auto const octree_minmax_point_y = std::minmax_element(
					begin, end,
					yless
				);
				auto const octree_minmax_point_z = std::minmax_element(
					begin, end,
					zless
				);

				auto const minvpx = *vector_minmax_point_x.first;
				auto const maxvpx = *vector_minmax_point_x.second;
				auto const minvpy = *vector_minmax_point_y.first;
				auto const maxvpy = *vector_minmax_point_y.second;
				auto const minvpz = *vector_minmax_point_z.first;
				auto const maxvpz = *vector_minmax_point_z.second;

				auto const minopx = *octree_minmax_point_x.first;
				auto const maxopx = *octree_minmax_point_x.second;
				auto const minopy = *octree_minmax_point_y.first;
				auto const maxopy = *octree_minmax_point_y.second;
				auto const minopz = *octree_minmax_point_z.first;
				auto const maxopz = *octree_minmax_point_z.second;

				REQUIRE(minvpx == minopx);
				REQUIRE(maxvpx == maxopx);
				REQUIRE(minvpy == minopy);
				REQUIRE(maxvpy == maxopy);
				REQUIRE(minvpz == minopz);
				REQUIRE(maxvpz == maxopz);

				REQUIRE(std::count(begin, end, test_point) == test_point_count);
			}
		}
	}
}