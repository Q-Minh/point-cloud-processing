#include <catch2/catch.hpp>
#include <pcp/filter/statistical_outlier_filter.hpp>

SCENARIO("statistical_outlier_filter", "[statistical_outlier_filter]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    using filter =
        pcp::basic_statistical_outlier_filter_t<pcp::point_t, float, decltype(coordinate_map)>;

    WHEN("calculating statistical outlier filter with two points (outside the distance threshold)")
    {
        std::vector<pcp::point_t> points{};
        points.push_back(pcp::point_t{3.0f, 3.0f, -3.0f});      // mean distance should be 5.01959
        points.push_back(pcp::point_t{1.0f, 1.0f, 0.0f});       // mean distance should be 3.09557
        points.push_back(pcp::point_t{3.0f, 0.0f, 3.0f});       // mean distance should be 2.57794
        points.push_back(pcp::point_t{2.0f, 0.0f, 2.0f});       // mean distance should be  1.93185
        points.push_back(pcp::point_t{-2.0f, 3.0f, 2.0f});      // mean distance should be 4.56155
        points.push_back(pcp::point_t{99.0f, 99.0f, 99.0f});    // should be rejected
        points.push_back(pcp::point_t{-99.0f, -99.0f, -99.0f}); // should be rejected

        pcp::kdtree::construction_params_t params{};
        params.max_depth    = 4u;
        params.construction = pcp::kdtree::construction_t::nth_element;

        kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
        pcp::statistical_outlier_filter::construction_params_t<float> filter_params{};
        filter_params.mean_k_                       = 2u;
        filter_params.std_dev_multiplier_threshold_ = 1.0;
        filter statistical_outlier_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            filter_params);
        auto it = std::remove_if(
            std::execution::seq,
            points.begin(),
            points.end(),
            statistical_outlier_filter);

        points.erase(it, points.end());

        THEN("there should be only 5 points left after the filter")
        {
            pcp::kdtree::construction_params_t params{};
            params.max_depth    = 4u;
            params.construction = pcp::kdtree::construction_t::nth_element;

            kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
            pcp::statistical_outlier_filter::construction_params_t<float> filter_params{};
            REQUIRE(points.size() == 5u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{3.0f, 3.0f, -3.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{1.0f, 1.0f, 0.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{3.0f, 0.0f, 3.0f}, p);
                    }) == 1u);

            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.0f, 0.0f, 2.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{-2.0f, 3.0f, 2.0f}, p);
                    }) == 1u);

            REQUIRE(
                std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                    return pcp::common::are_vectors_equal(pcp::point_t{-99.0f, -99.0f, -99.0f}, p);
                }) == 0u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{99.0f, 99.0f, 99.0f}, p);
                    }) == 0u);
        }
    }
    WHEN(
        "calculating statistical outlier filter  with all the points being inside the distance "
        "threshold due to having a larger multiplier")
    {
        std::vector<pcp::point_t> points{};
        points.push_back(pcp::point_t{3.0f, 3.0f, -3.0f});      // mean distance should be 5.01959
        points.push_back(pcp::point_t{1.0f, 1.0f, 0.0f});       // mean distance should be 3.09557
        points.push_back(pcp::point_t{3.0f, 0.0f, 3.0f});       // mean distance should be 2.57794
        points.push_back(pcp::point_t{2.0f, 0.0f, 2.0f});       // mean distance should be  1.93185
        points.push_back(pcp::point_t{-2.0f, 3.0f, 2.0f});      // mean distance should be 4.56155
        points.push_back(pcp::point_t{99.0f, 99.0f, 99.0f});    // should be rejected
        points.push_back(pcp::point_t{-99.0f, -99.0f, -99.0f}); // should be rejected

        pcp::kdtree::construction_params_t params{};
        params.max_depth    = 4u;
        params.construction = pcp::kdtree::construction_t::nth_element;

        kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
        pcp::statistical_outlier_filter::construction_params_t<float> filter_params{};
        filter_params.mean_k_                       = 2u;
        filter_params.std_dev_multiplier_threshold_ = 5.0;
        filter statistical_outlier_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            filter_params);
        auto it = std::remove_if(
            std::execution::seq,
            points.begin(),
            points.end(),
            statistical_outlier_filter);

        points.erase(it, points.end());

        THEN("there should be 7 points left after the filter, no points should be removed")
        {
            REQUIRE(points.size() == 7u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{3.0f, 3.0f, -3.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{1.0f, 1.0f, 0.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{3.0f, 0.0f, 3.0f}, p);
                    }) == 1u);

            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.0f, 0.0f, 2.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{-2.0f, 3.0f, 2.0f}, p);
                    }) == 1u);

            REQUIRE(
                std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                    return pcp::common::are_vectors_equal(pcp::point_t{-99.0f, -99.0f, -99.0f}, p);
                }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{99.0f, 99.0f, 99.0f}, p);
                    }) == 1u);
        }
    }
    WHEN(
        "calculating statistical outlier filter  with all the points being inside the distance "
        "with a normal multiplier")
    {
        std::vector<pcp::point_t> points{};
        points.push_back(pcp::point_t{3.0f, 3.0f, -3.0f});      // mean distance should be 5.01959
        points.push_back(pcp::point_t{1.0f, 1.0f, 0.0f});       // mean distance should be 3.09557
        points.push_back(pcp::point_t{3.0f, 0.0f, 3.0f});       // mean distance should be 2.57794
        points.push_back(pcp::point_t{2.0f, 0.0f, 2.0f});       // mean distance should be  1.93185
        points.push_back(pcp::point_t{-2.0f, 3.0f, 2.0f});      // mean distance should be 4.56155
        points.push_back(pcp::point_t{99.0f, 99.0f, 99.0f});    // should be rejected
        points.push_back(pcp::point_t{-99.0f, -99.0f, -99.0f}); // should be rejected

        pcp::kdtree::construction_params_t params{};
        params.max_depth    = 4u;
        params.construction = pcp::kdtree::construction_t::nth_element;

        kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
        pcp::statistical_outlier_filter::construction_params_t<float> filter_params{};
        filter_params.mean_k_                       = 2u;
        filter_params.std_dev_multiplier_threshold_ = 1.0f;
        filter statistical_outlier_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            filter_params);
        auto it = std::remove_if(
            std::execution::seq,
            points.begin(),
            points.end(),
            statistical_outlier_filter);

        points.erase(it, points.end());

        THEN("there should be 5 points left after the filter, no points should be removed")
        {
            REQUIRE(points.size() == 5u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{3.0f, 3.0f, -3.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{1.0f, 1.0f, 0.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{3.0f, 0.0f, 3.0f}, p);
                    }) == 1u);

            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.0f, 0.0f, 2.0f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{-2.0f, 3.0f, 2.0f}, p);
                    }) == 1u);
        }
    }
}
