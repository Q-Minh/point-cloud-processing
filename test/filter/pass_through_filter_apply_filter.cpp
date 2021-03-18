#include <catch2/catch.hpp>
#include <pcp/filter/pass_through_filter.hpp>

SCENARIO("pass_through_filter", "[pass_through_filter]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    using filter = pcp::basic_pass_through_filter_t<pcp::point_t, float, decltype(coordinate_map)>;
    std::vector<pcp::point_t> points{};
    points.push_back(pcp::point_t{0.1f, 0.4f, 0.1f});
    points.push_back(pcp::point_t{2.2f, 2.1f, 0.4f});

    points.push_back(pcp::point_t{0.1f, 1.0f, 0.1f});
    points.push_back(pcp::point_t{0.2f, 1.2f, 0.2f});
    points.push_back(pcp::point_t{2.1f, 1.9f, 0.3f});

    pcp::kdtree::construction_params_t params{};
    params.max_depth    = 4u;
    params.construction = pcp::kdtree::construction_t::nth_element;

    kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
    pcp::pass_through_filter::construction_params_t<float> filter_params{};
    filter_params.index_to_filter_  = 1;
    filter_params.filter_limit_max_ = 2.0;
    filter_params.filter_limit_min_ = 0.5;
    WHEN("calculating pass_through filter with two noisy points (outside the limits)")
    {
        filter pass_through_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            filter_params);
        auto it =
            std::remove_if(std::execution::seq, points.begin(), points.end(), pass_through_filter);

        points.erase(it, points.end());

        THEN("there should be only 3 points left after the filter")
        {
            REQUIRE(points.size() == 3u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{0.1f, 1.0f, 0.1f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{0.2f, 1.2f, 0.2f}, p);
                    }) == 1u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.1f, 1.9f, 0.3f}, p);
                    }) == 1u);

            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{0.1f, 0.4f, 0.1f}, p);
                    }) == 0u);
            REQUIRE(std::count_if(points.cbegin(), points.cend(), [](auto const& p) {
                        return pcp::common::are_vectors_equal(pcp::point_t{2.2f, 2.1f, 0.4f}, p);
                    }) == 0u);
        }
    }
}
