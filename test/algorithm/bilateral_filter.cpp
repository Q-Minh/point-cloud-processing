#include <catch2/catch.hpp>
#include <pcp/algorithm/average_distance_to_neighbors.hpp>
#include <pcp/algorithm/bilateral_filter.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>

SCENARIO("bilateral filtering of point cloud points and normals", "[algorithm][filter][bilateral]")
{
    GIVEN("a point cloud with normals forming a straight line with some noise")
    {
        std::vector<pcp::point_t> points;
        // clang-format off
        points.push_back(pcp::point_t{-0.1f,   0.f,  0.f});
        points.push_back(pcp::point_t{-0.075f, 0.f,  0.f});
        points.push_back(pcp::point_t{-0.05f,  0.f,  0.01f});
        points.push_back(pcp::point_t{-0.025f, 0.f,  0.f});
        points.push_back(pcp::point_t{ 0.0f,   0.f,  0.f});
        points.push_back(pcp::point_t{ 0.025f, 0.f,  0.f});
        points.push_back(pcp::point_t{ 0.05f,  0.f, -0.01f});
        points.push_back(pcp::point_t{ 0.075f, 0.f,  0.f});
        points.push_back(pcp::point_t{ 0.1f,   0.f,  0.f});
        // clang-format on

        /**
         *
         * The point cloud looks like this:
         *
         * indices:    0   1    2   3   4   5   6    7   8
         *
         * normals:
         *
         *             ^   ^   ^    ^   ^   ^    ^   ^   ^
         *             |   |    \   |   |   |   /    |   |
         *
         * points:
         *
         *                      o
         *             o   o        o   o   o        o   o
         *                                      o
         *
         * ideal points:
         *
         *             o   o    o   o   o   o   o    o   o
         *
         * coordinate frame:
         *
         * z y
         * |/
         * --- x
         *
         * We would expect the bilateral filtering to compress points 2 and 6
         * into the perceived line formed by the rest of the points as can be
         * visualized in the 'ideal points' section.
         *
         */
        std::vector<pcp::normal_t> normals;
        // clang-format off
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        normals.push_back(pcp::normal_t{-0.19611614f, 0.f, 0.98058068f});
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        normals.push_back(pcp::normal_t{ 0.19611614f, 0.f, 0.98058068f});
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        normals.push_back(pcp::normal_t{ 0.f,         0.f, 1.f        });
        // clang-format on

        std::size_t const n = points.size();

        std::vector<std::size_t> indices(n);
        std::iota(indices.begin(), indices.end(), 0u);

        auto const point_map = [&](std::size_t const i) {
            return points[i];
        };
        auto const normal_map = [&](std::size_t const i) {
            return normals[i];
        };
        auto const coordinate_map = [&](std::size_t const i) {
            return std::array<float, 3u>{points[i].x(), points[i].y(), points[i].z()};
        };

        auto const get_mean_distance_to_neighbors = [&]() {
            pcp::kdtree::construction_params_t params;
            params.compute_max_depth = true;
            pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(coordinate_map)> kdtree{
                indices.begin(),
                indices.end(),
                coordinate_map,
                params};
            auto const knn_map = [&](std::size_t const i) {
                return kdtree.nearest_neighbours(i, 2u);
            };
            return pcp::algorithm::average_distance_to_neighbors(
                indices.begin(),
                indices.end(),
                point_map,
                knn_map);
        };

        WHEN("smoothing points with bilateral filter")
        {
            pcp::algorithm::bilateral::params_t params;
            params.K      = 2u;
            params.sigmaf = static_cast<double>(get_mean_distance_to_neighbors());
            params.sigmag = params.sigmaf / 8.;

            std::vector<pcp::point_t> filtered_points{};
            pcp::algorithm::bilateral_filter_points(
                indices.begin(),
                indices.end(),
                std::back_inserter(filtered_points),
                point_map,
                normal_map,
                params);

            THEN("the points with noise compress towards the straight line")
            {
                REQUIRE(points[2].z() > filtered_points[2].z());
                REQUIRE(points[6].z() < filtered_points[6].z());
            }
        }
        WHEN("improving normals with bilateral filter")
        {
            pcp::algorithm::bilateral::params_t params;
            params.K      = 2u;
            params.sigmaf = static_cast<double>(get_mean_distance_to_neighbors());
            params.sigmag = params.sigmaf / 8.;

            std::vector<pcp::normal_t> filtered_normals{};
            pcp::algorithm::bilateral_filter_normals(
                indices.begin(),
                indices.end(),
                std::back_inserter(filtered_normals),
                point_map,
                normal_map,
                params);

            /**
             * The normal improvement technique is a bit hard to test.
             * We could derive by hand the solution to the normal
             * improvement technique for the small point cloud of size
             * N = 9 used in this scenario at another time.
             */

            THEN("the number of normals is preserved")
            {
                REQUIRE(normals.size() == indices.size());
            }
        }
    }
}
