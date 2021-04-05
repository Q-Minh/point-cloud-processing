#include <catch2/catch.hpp>
#include <pcp/algorithm/edge_aware_upsampling.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <random>

SCENARIO("edge aware upsampling", "[algorithm][upsampling]")
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10.f, 10.f);
    auto const n = 1000u;
    std::vector<pcp::point_t> points;
    points.resize(n);
    std::generate(points.begin(), points.end(), [&dis, &gen]() {
        pcp::common::basic_vector3d_t v{dis(gen), dis(gen), dis(gen)};
        auto const len = pcp::common::norm(v);
        v              = v / len;
        return pcp::point_t{v.x(), v.y(), v.z()};
    });

    GIVEN("a randomly sampled sphere point cloud")
    {
        pcp::algorithm::ear::params_t params{};
        params.edge_sensitivity = 5.;
        params.output_point_count = 1020u;
        
    }
}
