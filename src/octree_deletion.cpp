#include <catch.hpp>

#include <octree.hpp>

SCENARIO("octree size is coherent with insertion", "[octree]") {
	GIVEN("an empty octree and a range of points") {
		std::vector<point_t> points{};

		// points in +x,+y,+z octant
		points.push_back({ 0.1f, 0.1f, 0.1f });
		points.push_back({ 0.2f, 0.2f, 0.2f });
		points.push_back({ 0.3f, 0.3f, 0.3f });
		points.push_back({ 0.4f, 0.4f, 0.4f });
		points.push_back({ 0.5f, 0.5f, 0.5f });
		points.push_back({ 0.6f, 0.6f, 0.6f });
		points.push_back({ 0.7f, 0.7f, 0.7f });
		points.push_back({ 0.8f, 0.8f, 0.8f });
		points.push_back({ 0.9f, 0.9f, 0.9f });
		// points in -x,+y,+z octant
		points.push_back({ -0.1f, 0.1f, 0.1f });
		points.push_back({ -0.2f, 0.2f, 0.2f });
		points.push_back({ -0.3f, 0.3f, 0.3f });
		points.push_back({ -0.4f, 0.4f, 0.4f });
		points.push_back({ -0.5f, 0.5f, 0.5f });
		points.push_back({ -0.6f, 0.6f, 0.6f });
		points.push_back({ -0.7f, 0.7f, 0.7f });
		points.push_back({ -0.8f, 0.8f, 0.8f });
		points.push_back({ -0.9f, 0.9f, 0.9f });
		// points in +x,-y,+z octant
		points.push_back({ 0.1f, -0.1f, 0.1f });
		points.push_back({ 0.2f, -0.2f, 0.2f });
		points.push_back({ 0.3f, -0.3f, 0.3f });
		points.push_back({ 0.4f, -0.4f, 0.4f });
		points.push_back({ 0.5f, -0.5f, 0.5f });
		points.push_back({ 0.6f, -0.6f, 0.6f });
		points.push_back({ 0.7f, -0.7f, 0.7f });
		points.push_back({ 0.8f, -0.8f, 0.8f });
		points.push_back({ 0.9f, -0.9f, 0.9f });
		// points in +x,-y,-z octant
		points.push_back({ 0.1f, -0.1f, -0.1f });
		points.push_back({ 0.2f, -0.2f, -0.2f });
		points.push_back({ 0.3f, -0.3f, -0.3f });
		points.push_back({ 0.4f, -0.4f, -0.4f });
		points.push_back({ 0.5f, -0.5f, -0.5f });
		points.push_back({ 0.6f, -0.6f, -0.6f });
		points.push_back({ 0.7f, -0.7f, -0.7f });
		points.push_back({ 0.8f, -0.8f, -0.8f });
		points.push_back({ 0.9f, -0.9f, -0.9f });

		octree_parameters_t params;
		params.voxel_grid = axis_aligned_bounding_box_t
		{
			point_t{ -1.f, -1.f, -1.f },
			point_t{  1.f,  1.f,  1.f}
		};


	}
}
