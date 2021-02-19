#include <catch2/catch.hpp>
#include <pcp/kdtree/linked_kdtree.hpp>

SCENARIO("kdtree iterators are valid LegacyForwardIterator types", "[kdtree]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;

    auto offset    = GENERATE(1.f, 2.f);
    auto max_depth = GENERATE(1u, 2u, 4u, 12u);

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

    // params.voxel_grid = pcp::axis_aligned_bounding_box_t<pcp::point_t>{
    //    {-(offset + 1.f), -(offset + 1.f), -(offset + 1.f)},
    //    {offset + 1.f, offset + 1.f, offset + 1.f}};
    pcp::kdtree::construction_params_t params;
    params.construction = pcp::kdtree::construction_t::nth_element;
    params.max_depth    = max_depth;
    params.max_depth    = static_cast<std::uint8_t>(max_depth);

    GIVEN("an kdtree")
    {
        kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};

        WHEN("using its kdtree iterators")
        {
            auto const begin = kdtree.cbegin();
            auto const end   = kdtree.cend();

            THEN("std::distance is valid")
            {
                auto const size = static_cast<std::size_t>(std::distance(begin, end));
                REQUIRE(points.size() == size);
                REQUIRE(kdtree.size() == size);
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
                auto const kdtree_sum = std::accumulate(begin, end, pcp::point_t{0.f, 0.f, 0.f});

                REQUIRE(pcp::common::are_vectors_equal(vector_sum, kdtree_sum));

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

                auto const kdtree_minmax_point_x = std::minmax_element(begin, end, xless);
                auto const kdtree_minmax_point_y = std::minmax_element(begin, end, yless);
                auto const kdtree_minmax_point_z = std::minmax_element(begin, end, zless);

                auto const minvpx = *vector_minmax_point_x.first;
                auto const maxvpx = *vector_minmax_point_x.second;
                auto const minvpy = *vector_minmax_point_y.first;
                auto const maxvpy = *vector_minmax_point_y.second;
                auto const minvpz = *vector_minmax_point_z.first;
                auto const maxvpz = *vector_minmax_point_z.second;

                auto const minopx = *kdtree_minmax_point_x.first;
                auto const maxopx = *kdtree_minmax_point_x.second;
                auto const minopy = *kdtree_minmax_point_y.first;
                auto const maxopy = *kdtree_minmax_point_y.second;
                auto const minopz = *kdtree_minmax_point_z.first;
                auto const maxopz = *kdtree_minmax_point_z.second;

                REQUIRE(pcp::common::are_vectors_equal(minvpx, minopx));
                REQUIRE(pcp::common::are_vectors_equal(maxvpx, maxopx));
                REQUIRE(pcp::common::are_vectors_equal(minvpy, minopy));
                REQUIRE(pcp::common::are_vectors_equal(maxvpy, maxopy));
                REQUIRE(pcp::common::are_vectors_equal(minvpz, minopz));
                REQUIRE(pcp::common::are_vectors_equal(maxvpz, maxopz));

                std::size_t const kdtree_test_point_count = static_cast<std::size_t>(
                    std::count_if(begin, end, [&test_point](auto const& p) {
                        return pcp::common::are_vectors_equal(test_point, p);
                    }));
                REQUIRE(kdtree_test_point_count == test_point_count);
            }
        }
    }
}