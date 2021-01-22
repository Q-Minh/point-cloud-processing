#include <catch2/catch.hpp>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_tangent_planes.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/point_view.hpp>
#include <pcp/octree/octree.hpp>

SCENARIO("computing point cloud tangent planes", "[tangent_plane]")
{
    GIVEN("a random point cloud")
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-10.f, 10.f);
        auto const n = 1000u;
        std::vector<pcp::point_t> point_cloud;
        point_cloud.resize(n);
        std::generate(point_cloud.begin(), point_cloud.end(), [&dis, &gen]() {
            return pcp::point_t{dis(gen), dis(gen), dis(gen)};
        });

        auto const point_map = [](pcp::point_t const& p) {
            return p;
        };

        pcp::octree_parameters_t<pcp::point_t> params;
        params.voxel_grid = {{-10.f, -10.f, -10.f}, {10.f, 10.f, 10.f}};
        pcp::linked_octree_t octree(point_cloud.begin(), point_cloud.end(), point_map, params);
        std::uint64_t const k = 5u;

        WHEN("computing the point cloud's tangent planes")
        {
            std::vector<pcp::common::plane3d_t> tangent_planes;
            auto const knn = [=, &octree](pcp::point_t const& p) {
                return octree.nearest_neighbours(p, k, point_map);
            };
            pcp::algorithm::estimate_tangent_planes(
                point_cloud.cbegin(),
                point_cloud.cend(),
                std::back_inserter(tangent_planes),
                knn,
                pcp::algorithm::default_plane_transform<pcp::point_t, pcp::common::plane3d_t>);

            THEN(
                "the tangent planes are correctly described by the center of geometry of knn "
                "neighbors and their estimated normal")
            {
                REQUIRE(tangent_planes.size() == point_cloud.size());
                std::size_t valid_normals_count = 0u;
                std::size_t valid_points_count  = 0u;
                for (std::size_t i = 0u; i < n; ++i)
                {
                    auto const& neighbors = octree.nearest_neighbours(point_cloud[i], k, point_map);
                    pcp::normal_t const expected_normal =
                        pcp::estimate_normal(neighbors.cbegin(), neighbors.cend());
                    pcp::point_t const expected_point =
                        pcp::common::center_of_geometry(neighbors.cbegin(), neighbors.cend());

                    if (pcp::common::are_vectors_equal(
                            tangent_planes[i].normal(),
                            expected_normal) ||
                        pcp::common::are_vectors_equal(
                            tangent_planes[i].normal(),
                            -expected_normal))
                        ++valid_normals_count;

                    if (pcp::common::are_vectors_equal(tangent_planes[i].point(), expected_point))
                        ++valid_points_count;
                }
                bool const are_normals_valid = valid_normals_count == n;
                REQUIRE(are_normals_valid);
                bool const are_points_valid = valid_points_count == n;
                REQUIRE(are_points_valid);
            }
        }
    }
}
