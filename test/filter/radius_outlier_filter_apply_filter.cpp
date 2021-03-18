#include <algorithm>
#include <catch2/catch.hpp>
#include <pcp/filter/radius_outlier_filter.hpp>

SCENARIO("radious_outlier_filter", "[radious_outlier_filter]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    using filter =
        pcp::basic_radius_outlier_filter_t<pcp::point_t, float, decltype(coordinate_map)>;
    std::vector<pcp::point_t> points{};
    points.push_back(pcp::point_t{1.5f, 1.0f, 0.1f});
    points.push_back(pcp::point_t{2.2f, 2.1f, 0.4f});
    points.push_back(pcp::point_t{1.0f, 1.0f, 0.1f});
    points.push_back(pcp::point_t{0.2f, 1.2f, 0.2f});
    points.push_back(pcp::point_t{2.1f, 1.9f, 0.3f});

    points.push_back(pcp::point_t{600.0f, 600.0f, 600.0f});
    points.push_back(pcp::point_t{-300.0f, -300.0f, -300.0f});
    pcp::kdtree::construction_params_t params{};
    params.max_depth    = 4u;
    params.construction = pcp::kdtree::construction_t::nth_element;

    kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
    pcp::radius_outlier_filter::construction_params_t<float> filter_params{};
    filter_params.radius_ = 1.0f;
    filter_params.min_neighbors_in_radius_ = 2;
    WHEN("calculating radius outlier filter with two noisy points")
    {
        filter radius_outlier_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            filter_params);
        auto it = std::remove_if(
            std::execution::seq,
            points.begin(),
            points.end(),
            radius_outlier_filter);

        points.erase(it, points.end());

        THEN("there should be only 5 points left after the filter")
        {
            REQUIRE(points.size() == 5u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{1.5f, 1.0f, 0.1f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.2f, 2.1f, 0.4f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{1.0f, 1.0f, 0.1f}, p);
                    }) == 1u);

            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{0.2f, 1.2f, 0.2f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.1f, 1.9f, 0.3f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                    return pcp::common::are_vectors_equal(pcp::point_t{600.0f, 600.0f, 600.0f}, p);
                    }) == 0u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                    return pcp::common::are_vectors_equal(pcp::point_t{-300.0f, -300.0f, -300.0f}, p);
                    }) == 0u);
        }
    }
}
