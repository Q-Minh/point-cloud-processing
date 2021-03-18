#include <catch2/catch.hpp>
#include <pcp/filter/density_filter.hpp>

SCENARIO("density_filter", "[density_filter]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    using filter      = pcp::basic_density_filter_t<pcp::point_t, float, decltype(coordinate_map)>;
    GIVEN("points and a kdtree")
    {
        std::vector<pcp::point_t> points{};
        points.push_back(pcp::point_t{0.1f, 0.1f, 0.1f});
        points.push_back(pcp::point_t{0.2f, 0.2f, 0.2f});
        points.push_back(pcp::point_t{2.1f, 0.3f, 0.3f});
        points.push_back(pcp::point_t{2.2f, 0.4f, 0.4f});
        points.push_back(pcp::point_t{2.3f, 2.1f, 0.5f});
        points.push_back(pcp::point_t{2.4f, 2.2f, 0.6f});
        points.push_back(pcp::point_t{0.3f, 2.3f, 0.7f});
        points.push_back(pcp::point_t{0.4f, 2.4f, 0.8f});
        points.push_back(pcp::point_t{0.5f, 0.5f, 2.1f});
        points.push_back(pcp::point_t{0.6f, 0.6f, 2.2f});
        points.push_back(pcp::point_t{2.5f, 0.7f, 2.3f});
        points.push_back(pcp::point_t{2.6f, 0.8f, 2.4f});
        points.push_back(pcp::point_t{2.7f, 2.5f, 2.5f});
        points.push_back(pcp::point_t{2.8f, 2.6f, 2.6f});
        points.push_back(pcp::point_t{0.7f, 2.7f, 2.7f});
        points.push_back(pcp::point_t{0.8f, 2.8f, 2.8f});

        pcp::point_t p_noise1{-300.0f, -300.0f, -300.0f};
        pcp::point_t p_noise2{600.0f, 600.0, 600.0f};
        //points.push_back(p_noise1);
        //points.push_back(p_noise2);

        pcp::kdtree::construction_params_t params{};
        params.max_depth    = 4u;
        params.construction = pcp::kdtree::construction_t::nth_element;

        kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
        pcp::density_filter::construction_params_t<float> filter_params{};
        filter_params.k_                 = 3u;
        filter_params.density_threshold_ = 16.0f;
        filter_params.radius_multiplier_ = 1.0f;

        WHEN("calculating density filter with one noisy point")
        {
            points.push_back(p_noise1);
            using filter =
                pcp::basic_density_filter_t<pcp::point_t, float, decltype(coordinate_map)>;

            filter
                density_filter(points.begin(), points.end(), coordinate_map, kdtree, filter_params);
            auto it =
                std::remove_if(std::execution::seq, points.begin(), points.end(), density_filter);

            points.erase(it, points.end());

            THEN(
                "there should be less points due to the fact that the outlier points should have "
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
            points.push_back(p_noise1);
            points.push_back(p_noise2);
            filter
                density_filter(points.begin(), points.end(), coordinate_map, kdtree, filter_params);
            auto it =
                std::remove_if(std::execution::seq, points.begin(), points.end(), density_filter);

            points.erase(it, points.end());

            THEN(
                "there should be less points due to the fact that the outlier points should have "
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
