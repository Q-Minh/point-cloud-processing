#include <catch.hpp>

#include <octree.hpp>

SCENARIO("range searches on the octree", "[octree]") {
	GIVEN("a hand constructed octree of node capacity = 1 with 1 point in each octant") {
		octree_parameters_t params;
		params.node_capacity = 1u;
		params.max_depth = 21u;
		params.voxel_grid = axis_aligned_bounding_box_t
		{
			point_t{ -1.f, -1.f, -1.f },
			point_t{  1.f,  1.f,  1.f }
		};

		octree_t octree(params);
		octree.insert({ -.5f, -.5f, -.5f }); // 000
		octree.insert({  .5f, -.5f, -.5f }); // 100
		octree.insert({  .5f,  .5f, -.5f }); // 110
		octree.insert({ -.5f,  .5f, -.5f }); // 010
		octree.insert({ -.5f, -.5f,  .5f }); // 001
		octree.insert({  .5f, -.5f,  .5f }); // 101
		octree.insert({  .5f,  .5f,  .5f }); // 111
		octree.insert({ -.5f,  .5f,  .5f }); // 011

		octree.insert({ -.4f, -.3f, -.6f }); // 000
		octree.insert({  .4f, -.3f, -.6f }); // 100
		octree.insert({  .4f,  .3f, -.6f }); // 110
		octree.insert({ -.4f,  .3f, -.6f }); // 010
		octree.insert({ -.4f, -.3f,  .6f }); // 001
		octree.insert({  .4f, -.3f,  .6f }); // 101
		octree.insert({  .4f,  .3f,  .6f }); // 111
		octree.insert({ -.4f,  .3f,  .6f }); // 011

		WHEN("searching for points that are not contained in the queried sphere") {
			sphere_t sphere;
			sphere.position = { 0.f, 0.f, 0.f };
			sphere.radius = 0.1f;

			auto const points_in_range = octree.range_search(sphere);
			THEN("no points are found") {
				REQUIRE(points_in_range.empty());
			}
		}
		WHEN("searching for points that are contained in the queried sphere") {
			sphere_t sphere;
			sphere.position = { .9f, .9f, .9f };
			sphere.radius = 1.f;

			auto const points_in_range = octree.range_search(sphere);
			THEN("the points in the range are returned") {
				REQUIRE(points_in_range.size() == 2u);
				REQUIRE(std::count(points_in_range.cbegin(), points_in_range.cend(), point_t{ .5f, .5f, .5f }) == 1u);
				REQUIRE(std::count(points_in_range.cbegin(), points_in_range.cend(), point_t{ .4f, .3f, .6f }) == 1u);
			}
		}
		WHEN("searching for points that are not contained in the queried aabb") {
			axis_aligned_bounding_box_t aabb;
			aabb.min = { 1.05f, 1.05f, 1.05f };
			aabb.max = { 2.f, 2.f, 2.f };

			auto const points_in_range = octree.range_search(aabb);
			THEN("no points are found") {
				REQUIRE(points_in_range.size() == 0u);
			}
		}
		WHEN("searching for points that are in the queried aabb") {
			axis_aligned_bounding_box_t aabb;
			aabb.min = { -2.f, -2.f, -2.f };
			aabb.max = { 0.f, 0.f, 0.f };

			auto const points_in_range = octree.range_search(aabb);
			THEN("the points in the range are returned") {
				REQUIRE(points_in_range.size() == 2u);
				REQUIRE(std::count(points_in_range.cbegin(), points_in_range.cend(), point_t{ -.5f, -.5f, -.5f }) == 1u);
				REQUIRE(std::count(points_in_range.cbegin(), points_in_range.cend(), point_t{ -.4f, -.3f, -.6f }) == 1u);
			}
		}
	}
}
