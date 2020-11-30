#include <catch2/catch.hpp>
#include <pcp/octree/octree.hpp>

/*
 * Overload STL algorithms to optimize certain operations
 */
namespace std {

template <>
inline pcp::octree_t::const_iterator find<pcp::octree_t::const_iterator, pcp::point_t>(
    pcp::octree_t::const_iterator begin,
    pcp::octree_t::const_iterator end,
    pcp::point_t const& value)
{
    return pcp::find(begin, end, value);
}

template <>
inline auto count<pcp::octree_t::const_iterator, pcp::point_t>(
    pcp::octree_t::const_iterator begin,
    pcp::octree_t::const_iterator end,
    pcp::point_t const& value) -> pcp::octree_t::const_iterator::difference_type
{
    return pcp::count(begin, end, value);
}

} // namespace std

SCENARIO("octree iterators are valid LegacyForwardIterator types", "[octree]")
{
    auto offset        = GENERATE(1.f, 2.f);
    auto node_capacity = GENERATE(1u, 2u, 4u);
    auto max_depth     = GENERATE(1u, 2u, 21u);

    std::vector<pcp::point_t> points{};

    points.push_back({-(offset + 0.9f), -(offset + 0.9f), -(offset + 0.9f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.2f), -(offset + 0.2f), -(offset + 0.2f)});
    points.push_back({-(offset + 0.1f), -(offset + 0.1f), -(offset + 0.1f)});
    points.push_back({offset + 0.1f, offset + 0.1f, offset + 0.1f});

    pcp::point_t const test_point{offset + 0.2f, offset + 0.2f, offset + 0.2f};
    auto test_point_count = points.size();
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    points.push_back(test_point);
    test_point_count = points.size() - test_point_count;

    points.push_back({offset + 0.9f, offset + 0.9f, offset + 0.9f});

    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid = pcp::axis_aligned_bounding_box_t<pcp::point_t>{
        {-(offset + 1.f), -(offset + 1.f), -(offset + 1.f)},
        {offset + 1.f, offset + 1.f, offset + 1.f}};
    params.node_capacity = node_capacity;
    params.max_depth     = static_cast<std::uint8_t>(max_depth);

    GIVEN("an octree")
    {
        pcp::octree_t octree(points.cbegin(), points.cend(), params);

        WHEN("using its octree iterators")
        {
            auto const begin = octree.cbegin();
            auto const end   = octree.cend();

            THEN("std::distance is valid")
            {
                auto const size = std::distance(begin, end);
                REQUIRE(points.size() == size);
                REQUIRE(octree.size() == size);
            }
            THEN("non-modifying sequence operations from stl algorithms are valid")
            {
                float const erroneous_coordinate = 100.f;
                REQUIRE(std::none_of(begin, end, [t = erroneous_coordinate](pcp::point_t const& p) {
                    return pcp::common::are_vectors_equal(p, pcp::point_t{t, t, t});
                }));
                REQUIRE(std::all_of(begin, end, [t = erroneous_coordinate](pcp::point_t const& p) {
                    return p.x() < t && p.y() < t && p.z() < t;
                }));

                auto const vector_sum =
                    std::accumulate(points.cbegin(), points.cend(), pcp::point_t{0.f, 0.f, 0.f});
                auto const octree_sum = std::accumulate(begin, end, pcp::point_t{0.f, 0.f, 0.f});

                REQUIRE(pcp::common::are_vectors_equal(vector_sum, octree_sum));

                auto const xless = [](pcp::point_t const& p1, pcp::point_t const& p2) -> bool {
                    return p1.x() < p2.x();
                };
                auto const yless = [](pcp::point_t const& p1, pcp::point_t const& p2) -> bool {
                    return p1.y() < p2.y();
                };
                auto const zless = [](pcp::point_t const& p1, pcp::point_t const& p2) -> bool {
                    return p1.z() < p2.z();
                };

                auto const vector_minmax_point_x =
                    std::minmax_element(points.cbegin(), points.cend(), xless);
                auto const vector_minmax_point_y =
                    std::minmax_element(points.cbegin(), points.cend(), yless);
                auto const vector_minmax_point_z =
                    std::minmax_element(points.cbegin(), points.cend(), zless);

                auto const octree_minmax_point_x = std::minmax_element(begin, end, xless);
                auto const octree_minmax_point_y = std::minmax_element(begin, end, yless);
                auto const octree_minmax_point_z = std::minmax_element(begin, end, zless);

                auto const minvpx = *vector_minmax_point_x.first;
                auto const maxvpx = *vector_minmax_point_x.second;
                auto const minvpy = *vector_minmax_point_y.first;
                auto const maxvpy = *vector_minmax_point_y.second;
                auto const minvpz = *vector_minmax_point_z.first;
                auto const maxvpz = *vector_minmax_point_z.second;

                auto const minopx = *octree_minmax_point_x.first;
                auto const maxopx = *octree_minmax_point_x.second;
                auto const minopy = *octree_minmax_point_y.first;
                auto const maxopy = *octree_minmax_point_y.second;
                auto const minopz = *octree_minmax_point_z.first;
                auto const maxopz = *octree_minmax_point_z.second;

                REQUIRE(pcp::common::are_vectors_equal(minvpx, minopx));
                REQUIRE(pcp::common::are_vectors_equal(maxvpx, maxopx));
                REQUIRE(pcp::common::are_vectors_equal(minvpy, minopy));
                REQUIRE(pcp::common::are_vectors_equal(maxvpy, maxopy));
                REQUIRE(pcp::common::are_vectors_equal(minvpz, minopz));
                REQUIRE(pcp::common::are_vectors_equal(maxvpz, maxopz));

                REQUIRE(
                    std::count<pcp::octree_t::const_iterator, pcp::point_t>(
                        begin,
                        end,
                        test_point) == test_point_count);
            }
        }
    }
}