#include <catch2/catch.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/sphere.hpp>
#include <pcp/traits/identity_map.hpp>
#include <pcp/traits/index_map.hpp>
#include <pcp/traits/knn_map.hpp>
#include <pcp/traits/normal_map.hpp>
#include <pcp/traits/point_map.hpp>
#include <pcp/traits/property_map_traits.hpp>
#include <pcp/traits/range_neighbor_map.hpp>
#include <pcp/traits/signed_distance_map.hpp>

struct dummy_type
{
};

#if defined(__clang__) || defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined(_MSC_VER)
    #pragma warning(push)
    #pragma warning(disable : 4100)
#endif
unsigned int fidentity_map(dummy_type dummy)
{
    return 0u;
}

unsigned int findex_map(dummy_type dummy)
{
    return 0u;
}

pcp::point_t fpoint_map(dummy_type dummy)
{
    return pcp::point_t{};
}

pcp::normal_t fnormal_map(dummy_type dummy)
{
    return pcp::normal_t{};
}

std::vector<pcp::point_t> fknn_map(dummy_type dummy)
{
    return std::vector<pcp::point_t>{};
}

std::vector<pcp::point_t> frange_neighbor_map(dummy_type dummy, pcp::sphere_t<pcp::point_t> sphere)
{
    return std::vector<pcp::point_t>();
}

float fsigned_distance_map(dummy_type dummy)
{
    return float{};
}

void fbad_identity_map(dummy_type dummy){}

float fbad_index_map(dummy_type dummy)
{
    return 0.f;
}

pcp::sphere_t<pcp::point_t> fbad_point_map(dummy_type dummy)
{
    return pcp::sphere_t<pcp::point_t>{};
}

dummy_type fbad_normal_map(pcp::normal_t normal)
{
    return dummy_type{};
}

pcp::point_t fbad_knn_map(dummy_type dummy)
{
    return pcp::point_t{};
}

void fbad_range_neighbor_map(dummy_type dummy, pcp::sphere_t<pcp::point_t> sphere){}

void fbad_signed_distance_map(dummy_type dummy){}

#if defined(_MSC_VER)
    #pragma warning(push)
    #pragma warning(disable : 26444)
#endif
TEMPLATE_TEST_CASE("property maps", "[type][property-maps]", int, float, dummy_type, pcp::point_t)
{
    pcp::point_t point{};
    pcp::normal_t normal{};

    auto const identity_map = [](dummy_type dummy) {
        return 0u;
    };
    auto const bad_identity_map = [](dummy_type dummy) {
    };
    std::function<unsigned int(dummy_type)> identity_map_f = identity_map;
    std::function<void(dummy_type)> bad_identity_map_f     = bad_identity_map;

    auto const index_map = [](dummy_type dummy) {
        return 0u;
    };
    auto const bad_index_map = [](dummy_type dummy) -> char const* {
        return "bla";
    };
    std::function<unsigned int(dummy_type)> index_map_f    = index_map;
    std::function<char const*(dummy_type)> bad_index_map_f = bad_index_map;

    auto const point_map = [&](dummy_type dummy) {
        return point;
    };
    auto const bad_point_map = [](pcp::point_t p) {
        return dummy_type{};
    };
    std::function<pcp::point_t(dummy_type)> point_map_f     = point_map;
    std::function<dummy_type(pcp::point_t)> bad_point_map_f = bad_point_map;

    auto const normal_map = [&](dummy_type dummy) {
        return normal;
    };
    auto const bad_normal_map = [](pcp::normal_t n) {
        return dummy_type{};
    };
    std::function<pcp::normal_t(dummy_type)> normal_map_f     = normal_map;
    std::function<dummy_type(pcp::normal_t)> bad_normal_map_f = bad_normal_map;

    auto const knn_map = [](dummy_type dummy) {
        return std::vector<pcp::point_t>{};
    };
    auto const bad_knn_map = [](dummy_type dummy) {
        return pcp::point_t{};
    };
    std::function<std::vector<pcp::point_t>(dummy_type)> knn_map_f = knn_map;
    std::function<pcp::point_t(dummy_type)> bad_knn_map_f          = bad_knn_map;

    auto const range_neighbor_map = [](dummy_type dummy, pcp::sphere_t<pcp::point_t> sphere) {
        return std::vector<pcp::point_t>();
    };
    auto const bad_range_neighbor_map = [](dummy_type dummy, pcp::sphere_t<pcp::point_t> sphere) {
        return pcp::point_t{};
    };
    std::function<std::vector<pcp::point_t>(dummy_type, pcp::sphere_t<pcp::point_t>)>
        range_neighbor_map_f = range_neighbor_map;
    std::function<pcp::point_t(dummy_type, pcp::sphere_t<pcp::point_t>)> bad_range_neighbor_map_f =
        bad_range_neighbor_map;

    auto const signed_distance_map = [](dummy_type dummy) {
        return float{};
    };
    auto const bad_signed_distance_map = [](dummy_type dummy) {
        return "bla";
    };
    std::function<float(dummy_type)> signed_distance_map_f           = signed_distance_map;
    std::function<char const*(dummy_type)> bad_signed_distance_map_f = bad_signed_distance_map;

    GIVEN("property maps as std::function")
    {
        WHEN("property maps are valid")
        {
            THEN("property map traits return true")
            {
                REQUIRE(pcp::traits::is_identity_map_v<decltype(identity_map_f), dummy_type>);
                REQUIRE(pcp::traits::is_index_map_v<decltype(index_map_f), dummy_type>);
                REQUIRE(pcp::traits::is_point_map_v<decltype(point_map_f), dummy_type>);
                REQUIRE(pcp::traits::is_normal_map_v<decltype(normal_map_f), dummy_type>);
                REQUIRE(pcp::traits::is_knn_map_v<decltype(knn_map_f), dummy_type>);
                REQUIRE(pcp::traits::is_range_neighbor_map_v<
                        decltype(range_neighbor_map_f),
                        dummy_type,
                        pcp::sphere_t<pcp::point_t>>);
                REQUIRE(pcp::traits::
                            is_signed_distance_map_v<decltype(signed_distance_map_f), dummy_type>);
            }
        }
        WHEN("property maps are invalid")
        {
            THEN("property map traits return false")
            {
                REQUIRE(!pcp::traits::is_identity_map_v<decltype(bad_identity_map_f), dummy_type>);
                REQUIRE(!pcp::traits::is_index_map_v<decltype(bad_index_map_f), dummy_type>);
                REQUIRE(!pcp::traits::is_point_map_v<decltype(bad_point_map_f), pcp::point_t>);
                REQUIRE(!pcp::traits::is_normal_map_v<decltype(bad_normal_map_f), pcp::normal_t>);
                REQUIRE(!pcp::traits::is_knn_map_v<decltype(bad_knn_map_f), dummy_type>);
                REQUIRE(!pcp::traits::is_range_neighbor_map_v<
                        decltype(bad_range_neighbor_map_f),
                        dummy_type,
                        pcp::sphere_t<pcp::point_t>>);
                REQUIRE(
                    !pcp::traits::
                        is_signed_distance_map_v<decltype(bad_signed_distance_map_f), dummy_type>);
            }
        }
    }
    GIVEN("property maps as lambda")
    {
        WHEN("property maps are valid")
        {
            THEN("property map traits return true")
            {
                REQUIRE(pcp::traits::is_identity_map_v<decltype(identity_map), dummy_type>);
                REQUIRE(pcp::traits::is_index_map_v<decltype(index_map), dummy_type>);
                REQUIRE(pcp::traits::is_point_map_v<decltype(point_map), dummy_type>);
                REQUIRE(pcp::traits::is_normal_map_v<decltype(normal_map), dummy_type>);
                REQUIRE(pcp::traits::is_knn_map_v<decltype(knn_map), dummy_type>);
                REQUIRE(pcp::traits::is_range_neighbor_map_v<
                        decltype(range_neighbor_map),
                        dummy_type,
                        pcp::sphere_t<pcp::point_t>>);
                REQUIRE(pcp::traits::
                            is_signed_distance_map_v<decltype(signed_distance_map), dummy_type>);
            }
        }
        WHEN("property maps are invalid")
        {
            THEN("property map traits return false")
            {
                REQUIRE(!pcp::traits::is_identity_map_v<decltype(bad_identity_map), dummy_type>);
                REQUIRE(!pcp::traits::is_index_map_v<decltype(bad_index_map), dummy_type>);
                REQUIRE(!pcp::traits::is_point_map_v<decltype(bad_point_map), pcp::point_t>);
                REQUIRE(!pcp::traits::is_normal_map_v<decltype(bad_normal_map), pcp::normal_t>);
                REQUIRE(!pcp::traits::is_knn_map_v<decltype(bad_knn_map), dummy_type>);
                REQUIRE(!pcp::traits::is_range_neighbor_map_v<
                        decltype(bad_range_neighbor_map),
                        dummy_type,
                        pcp::sphere_t<pcp::point_t>>);
                REQUIRE(
                    !pcp::traits::
                        is_signed_distance_map_v<decltype(bad_signed_distance_map), dummy_type>);
            }
        }
    }
    GIVEN("property maps as function")
    {
        WHEN("property maps are valid")
        {
            THEN("property map traits return true")
            {
                REQUIRE(pcp::traits::is_identity_map_v<decltype(fidentity_map), dummy_type>);
                REQUIRE(pcp::traits::is_index_map_v<decltype(findex_map), dummy_type>);
                REQUIRE(pcp::traits::is_point_map_v<decltype(fpoint_map), dummy_type>);
                REQUIRE(pcp::traits::is_normal_map_v<decltype(fnormal_map), dummy_type>);
                REQUIRE(pcp::traits::is_knn_map_v<decltype(fknn_map), dummy_type>);
                REQUIRE(pcp::traits::is_range_neighbor_map_v<
                        decltype(frange_neighbor_map),
                        dummy_type,
                        pcp::sphere_t<pcp::point_t>>);
                REQUIRE(pcp::traits::
                            is_signed_distance_map_v<decltype(fsigned_distance_map), dummy_type>);
            }
        }
        WHEN("property maps are invalid")
        {
            THEN("property map traits return false")
            {
                REQUIRE(!pcp::traits::is_identity_map_v<decltype(fbad_identity_map), dummy_type>);
                REQUIRE(!pcp::traits::is_index_map_v<decltype(fbad_index_map), dummy_type>);
                REQUIRE(!pcp::traits::is_point_map_v<decltype(fbad_point_map), dummy_type>);
                REQUIRE(!pcp::traits::is_normal_map_v<decltype(fbad_normal_map), pcp::normal_t>);
                REQUIRE(!pcp::traits::is_knn_map_v<decltype(fbad_knn_map), dummy_type>);
                REQUIRE(!pcp::traits::is_range_neighbor_map_v<
                        decltype(fbad_range_neighbor_map),
                        dummy_type,
                        pcp::sphere_t<pcp::point_t>>);
                REQUIRE(
                    !pcp::traits::
                        is_signed_distance_map_v<decltype(fbad_signed_distance_map), dummy_type>);
            }
        }
    }
}
#if defined(__clang__) || defined(__GNUC__)
    #pragma GCC diagnostic pop
#elif defined(_MSC_VER)
    #pragma warning(pop) // warning C4100
    #pragma warning(pop) // warning C26444
#endif
