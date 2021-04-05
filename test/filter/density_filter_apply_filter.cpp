#include <algorithm>
#include <catch2/catch.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/filter/density_filter.hpp>
#include <pcp/kdtree/linked_kdtree.hpp>

SCENARIO("density_filter", "[density_filter]")
{
    auto const point_map = [](pcp::point_t const& p) {
        return p;
    };
    auto const coordinate_map = [&](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    auto const construct = [](std::vector<pcp::point_t>& point_cloud) {
        point_cloud.push_back(pcp::point_t{0.1f, 0.1f, 0.1f});
        point_cloud.push_back(pcp::point_t{0.2f, 0.2f, 0.2f});
        point_cloud.push_back(pcp::point_t{2.1f, 0.3f, 0.3f});
        point_cloud.push_back(pcp::point_t{2.2f, 0.4f, 0.4f});
        point_cloud.push_back(pcp::point_t{2.3f, 2.1f, 0.5f});
        point_cloud.push_back(pcp::point_t{2.4f, 2.2f, 0.6f});
        point_cloud.push_back(pcp::point_t{0.3f, 2.3f, 0.7f});
        point_cloud.push_back(pcp::point_t{0.4f, 2.4f, 0.8f});
        point_cloud.push_back(pcp::point_t{0.5f, 0.5f, 2.1f});
        point_cloud.push_back(pcp::point_t{0.6f, 0.6f, 2.2f});
        point_cloud.push_back(pcp::point_t{2.5f, 0.7f, 2.3f});
        point_cloud.push_back(pcp::point_t{2.6f, 0.8f, 2.4f});
        point_cloud.push_back(pcp::point_t{2.7f, 2.5f, 2.5f});
        point_cloud.push_back(pcp::point_t{2.8f, 2.6f, 2.6f});
        point_cloud.push_back(pcp::point_t{0.7f, 2.7f, 2.7f});
        point_cloud.push_back(pcp::point_t{0.8f, 2.8f, 2.8f});
    };
    pcp::point_t p_noise1{-300.0f, -300.0f, -300.0f};
    pcp::point_t p_noise2{600.0f, 600.0, 600.0f};
    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    GIVEN("points and a kdtree")
    {
        std::vector<pcp::point_t> points{};

        pcp::kdtree::construction_params_t params{};
        params.max_depth    = 4u;
        params.construction = pcp::kdtree::construction_t::nth_element;

        pcp::density_filter::construction_params_t<float> filter_params{};
        filter_params.density_threshold_ = 16.0f;
        filter_params.radius_multiplier_ = 1.0f;

        WHEN("calculating density filter with one noisy point")
        {
            construct(points);
            points.push_back(p_noise1);
            kdtree_type kdtree(points.begin(), points.end(), coordinate_map, params);

            auto const range_search_map = [&](pcp::point_t p, float radius) {
                pcp::sphere_a<float> ball;
                ball.position[0] = coordinate_map(p)[0];
                ball.position[1] = coordinate_map(p)[1];
                ball.position[2] = coordinate_map(p)[2];
                ball.radius      = radius;
                return kdtree.range_search(ball);
            };

            auto const k       = 3u;
            auto const knn_map = [&](pcp::point_t const& p) {
                std::size_t num_neighbors = static_cast<std::size_t>(k);
                return kdtree.nearest_neighbours(p, num_neighbors);
            };

            pcp::basic_density_filter_t<
                pcp::point_t,
                float,
                decltype(point_map),
                decltype(knn_map),
                decltype(range_search_map)>
                density_filter(
                    points.begin(),
                    points.end(),
                    point_map,
                    knn_map,
                    range_search_map,
                    filter_params);
            auto it =
                std::remove_if(std::execution::seq, points.begin(), points.end(), density_filter);

            points.erase(it, points.end());

            THEN(
                "there should be less points due to the fact that the outlier points should "
                "have "
                "been removed")
            {
                REQUIRE(points.size() == 16u);
                REQUIRE(
                    std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{0.1f, 0.1f, 0.1f}, p);
                    }) == 1u);
                REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                            return pcp::common::are_vectors_equal(
                                pcp::point_t{600.0f, 600.0f, 600.0f},
                                p);
                        }) == 0u);
            }
        }
        WHEN("calculating density filter with two noisy points")
        {
            construct(points);
            points.push_back(p_noise1);
            points.push_back(p_noise2);
            kdtree_type kdtree(points.begin(), points.end(), coordinate_map, params);

            auto const range_search_map = [&](pcp::point_t p, float radius) {
                pcp::sphere_a<float> ball;
                ball.position[0] = coordinate_map(p)[0];
                ball.position[1] = coordinate_map(p)[1];
                ball.position[2] = coordinate_map(p)[2];
                ball.radius      = radius;
                return kdtree.range_search(ball);
            };

            auto const k       = 3u;
            auto const knn_map = [&](pcp::point_t const& p) {
                std::size_t num_neighbors = static_cast<std::size_t>(k);
                return kdtree.nearest_neighbours(p, num_neighbors);
            };

            pcp::basic_density_filter_t<
                pcp::point_t,
                float,
                decltype(point_map),
                decltype(knn_map),
                decltype(range_search_map)>
                density_filter(
                    points.begin(),
                    points.end(),
                    point_map,
                    knn_map,
                    range_search_map,
                    filter_params);
            auto it =
                std::remove_if(std::execution::par, points.begin(), points.end(), density_filter);

            points.erase(it, points.end());

            THEN(
                "there should be less points due to the fact that the outlier points should "
                "have "
                "been removed")
            {
                REQUIRE(points.size() == 16u);
                REQUIRE(
                    std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.8f, 2.6f, 2.6f}, p);
                    }) == 1u);
                REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                            return pcp::common::are_vectors_equal(
                                pcp::point_t{600.0f, 600.0, 600.0f},
                                p);
                        }) == 0u);
                REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                            return pcp::common::are_vectors_equal(
                                pcp::point_t{-300.0f, -300.0f, -300.0f},
                                p);
                        }) == 0u);
            }
        }
    }
}
