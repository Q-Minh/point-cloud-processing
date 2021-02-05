#include <catch2/catch.hpp>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/normals/normal_estimation.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/point_view.hpp>
#include <pcp/common/points/vertex.hpp>
#include <pcp/octree/octree.hpp>

SCENARIO("computing point cloud normals", "[normals]")
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

        WHEN("computing the point cloud's normals")
        {
            std::vector<pcp::normal_t> normals;
            auto const knn = [=, &octree](pcp::point_t const& p) {
                return octree.nearest_neighbours(p, k, point_map);
            };
            pcp::algorithm::estimate_normals(
                point_cloud.cbegin(),
                point_cloud.cend(),
                std::back_inserter(normals),
                point_map,
                knn,
                pcp::algorithm::default_normal_transform<pcp::point_t, pcp::normal_t>);

            THEN(
                "the normals are at least as precise as performing tangent plane normal estimation")
            {
                REQUIRE(normals.size() == point_cloud.size());
                std::size_t valid_normals_count = 0u;
                for (std::size_t i = 0u; i < n; ++i)
                {
                    auto const& neighbors = octree.nearest_neighbours(point_cloud[i], k, point_map);
                    pcp::normal_t const expected =
                        pcp::estimate_normal(neighbors.cbegin(), neighbors.cend(), point_map);
                    if (pcp::common::are_vectors_equal(normals[i], expected) ||
                        pcp::common::are_vectors_equal(normals[i], -expected))
                        ++valid_normals_count;
                }
                bool const are_normals_valid = valid_normals_count == n;
                REQUIRE(are_normals_valid);
            }
        }
    }
    GIVEN("a point cloud with normals of inconsistent orientations")
    {
        std::vector<pcp::point_t> point_cloud;
        std::vector<pcp::normal_t> normals;
        /*
         * Points:
         *
         * 1 2           3
         *   o
         *  / \----------o
         * o             |\---o 4
         *               -----o 5
         *
         * Normals:
         *
         * 1 2 3 4 5
         *
         * | ^ ^ | |
         * Y | | Y Y
         *
         */
        point_cloud.push_back({-1.f, -1.f, -.1f});
        point_cloud.push_back({-.9f, -1.f, .2f});
        point_cloud.push_back({.9f, .9f, .1f});
        point_cloud.push_back({1.1f, 1.1f, -.2f});
        point_cloud.push_back({1.1f, 1.1f, -.3f});
        normals.push_back({0.f, 0.f, -1.f});
        normals.push_back({0.f, 0.f, 1.f});
        normals.push_back({0.f, 0.f, 1.f});
        normals.push_back({0.f, 0.f, -1.f});
        normals.push_back({0.f, 0.f, -1.f});

        WHEN("computing normal orientations")
        {
            using vertex_type = pcp::vertex_t;
            std::vector<vertex_type> vertices;
            vertices.reserve(point_cloud.size());
            for (std::uint32_t i = 0; i < point_cloud.size(); ++i)
                vertices.push_back(vertex_type{&point_cloud[i], i});

            // prepare property maps
            auto const point_map = [&point_cloud](vertex_type const& v) {
                return point_cloud[v.id()];
            };
            auto const index_map = [](vertex_type const& v) {
                return v.id();
            };
            auto const normal_map = [&normals](vertex_type const& v) {
                return normals[v.id()];
            };
            auto const transform_op = [&normals](vertex_type const& v, pcp::normal_t const& n) {
                normals[v.id()] = n;
            };

            pcp::octree_parameters_t<pcp::point_t> params;
            params.voxel_grid = {{-2.f, -2.f, -2.f}, {2.f, 2.f, 2.f}};
            pcp::basic_linked_octree_t<vertex_type, decltype(params)> octree(
                vertices.cbegin(),
                vertices.cend(),
                point_map,
                params);

            auto const knn = [&](vertex_type const& v) {
                std::uint64_t const k = 2u;
                return octree.nearest_neighbours(vertices[v.id()], k, point_map);
            };

            pcp::algorithm::propagate_normal_orientations(
                vertices.begin(),
                vertices.end(),
                index_map,
                knn,
                point_map,
                normal_map,
                transform_op);

            THEN("the estimated normal orientations are more consistent")
            {
                bool const are_normal_orientations_consistent =
                    std::all_of(normals.cbegin(), normals.cend(), [](auto const& n) {
                        pcp::normal_t const expected{0.f, 0.f, 1.f};
                        return pcp::common::are_vectors_equal(n, expected);
                    });

                REQUIRE(are_normal_orientations_consistent);
            }
        }
    }
}
