#include <benchmark/benchmark.h>
#include <iostream>
#include <pcp/algorithm/algorithm.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/kdtree/kdtree.hpp>

std::array<std::vector<pcp::point_t>, 4u> point_clouds;

static void bm_average_distance_seq(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };
    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    for (auto _ : state)
    {
        float const mean = pcp::algorithm::average_distance_to_neighbors(
            std::execution::seq,
            indices.begin(),
            indices.end(),
            point_map,
            knn_map);

        benchmark::DoNotOptimize(mean);
    }
}

static void bm_average_distance_par(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };
    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    for (auto _ : state)
    {
        float const mean = pcp::algorithm::average_distance_to_neighbors(
            std::execution::par,
            indices.begin(),
            indices.end(),
            point_map,
            knn_map);

        benchmark::DoNotOptimize(mean);
    }
}

static void bm_normal_estimation_seq(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };
    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    for (auto _ : state)
    {
        std::vector<normal_type> normals{};
        normals.resize(point_clouds[idx].size());

        pcp::algorithm::estimate_normals(
            std::execution::seq,
            indices.begin(),
            indices.end(),
            normals.begin(),
            point_map,
            knn_map,
            [](auto, normal_type const& n) { return n; });

        benchmark::DoNotOptimize(normals);
    }
}

static void bm_normal_estimation_par(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };
    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    for (auto _ : state)
    {
        std::vector<normal_type> normals{};
        normals.resize(point_clouds[idx].size());

        pcp::algorithm::estimate_normals(
            std::execution::par,
            indices.begin(),
            indices.end(),
            normals.begin(),
            point_map,
            knn_map,
            [](auto, normal_type const& n) { return n; });

        benchmark::DoNotOptimize(normals);
    }
}

static void bm_point_bilateral_filter_seq(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };

    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    std::vector<normal_type> normals{};
    normals.resize(indices.size());
    pcp::algorithm::estimate_normals(
        std::execution::par,
        indices.begin(),
        indices.end(),
        normals.begin(),
        point_map,
        knn_map,
        [](auto, normal_type const& n) { return n; });

    auto const normal_map = [&](std::uint32_t element) {
        return normals[element];
    };

    float const mean = pcp::algorithm::average_distance_to_neighbors(
        std::execution::par,
        indices.begin(),
        indices.end(),
        point_map,
        knn_map);

    for (auto _ : state)
    {
        std::vector<point_type> new_points{};
        new_points.resize(point_clouds[idx].size());

        pcp::algorithm::bilateral::params_t bilateral_params{};
        bilateral_params.K      = 1u;
        bilateral_params.sigmaf = 4.f * mean;
        bilateral_params.sigmag = 1.f * mean;

        pcp::algorithm::bilateral_filter_points(
            std::execution::seq,
            indices.begin(),
            indices.end(),
            new_points.begin(),
            point_map,
            normal_map,
            bilateral_params);

        benchmark::DoNotOptimize(new_points);
    }
}

static void bm_point_bilateral_filter_par(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };

    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    std::vector<normal_type> normals{};
    normals.resize(indices.size());
    pcp::algorithm::estimate_normals(
        std::execution::par,
        indices.begin(),
        indices.end(),
        normals.begin(),
        point_map,
        knn_map,
        [](auto, normal_type const& n) { return n; });

    auto const normal_map = [&](std::uint32_t element) {
        return normals[element];
    };
    float const mean = pcp::algorithm::average_distance_to_neighbors(
        std::execution::par,
        indices.begin(),
        indices.end(),
        point_map,
        knn_map);

    for (auto _ : state)
    {
        std::vector<point_type> new_points{};
        new_points.resize(point_clouds[idx].size());

        pcp::algorithm::bilateral::params_t bilateral_params{};
        bilateral_params.K      = 1u;
        bilateral_params.sigmaf = 4.f * mean;
        bilateral_params.sigmag = 1.f * mean;

        pcp::algorithm::bilateral_filter_points(
            std::execution::par,
            indices.begin(),
            indices.end(),
            new_points.begin(),
            point_map,
            normal_map,
            bilateral_params);

        benchmark::DoNotOptimize(new_points);
    }
}

static void bm_wlop_seq(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };

    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    float const mean = pcp::algorithm::average_distance_to_neighbors(
        std::execution::par,
        indices.begin(),
        indices.end(),
        point_map,
        knn_map);

    for (auto _ : state)
    {
        pcp::algorithm::wlop::params_t wlop_params{};
        wlop_params.h       = 8.f * mean;
        wlop_params.I       = static_cast<std::size_t>(0.10f * indices.size());
        wlop_params.k       = 1u;
        wlop_params.uniform = true;

        std::vector<point_type> new_points{};
        new_points.resize(wlop_params.I);

        pcp::algorithm::wlop::wlop(
            std::execution::seq,
            indices.begin(),
            indices.end(),
            new_points.begin(),
            point_map,
            wlop_params);

        benchmark::DoNotOptimize(new_points);
    }
}

static void bm_wlop_par(benchmark::State& state)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::int64_t const idx = state.range(0u);

    std::vector<std::uint32_t> indices{};
    indices.resize(point_clouds[idx].size());
    std::iota(indices.begin(), indices.end(), 0u);

    auto const point_map = [&](std::uint32_t element) {
        return point_clouds[idx][element];
    };

    auto const coordinate_map = [&](std::uint32_t element) {
        point_type const& p = point_map(element);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth                   = true;
    params.min_element_count_for_parallel_exec = point_clouds[idx].size();

    pcp::basic_linked_kdtree_t<std::uint32_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn_map = [&](std::uint32_t element) {
        return kdtree.nearest_neighbours(element, 15u);
    };

    float const mean = pcp::algorithm::average_distance_to_neighbors(
        std::execution::par,
        indices.begin(),
        indices.end(),
        point_map,
        knn_map);

    for (auto _ : state)
    {
        pcp::algorithm::wlop::params_t wlop_params{};
        wlop_params.h       = 8.f * mean;
        wlop_params.I       = static_cast<std::size_t>(0.10f * indices.size());
        wlop_params.k       = 1u;
        wlop_params.uniform = true;

        std::vector<point_type> new_points{};
        new_points.resize(wlop_params.I);

        pcp::algorithm::wlop::wlop(
            std::execution::par,
            indices.begin(),
            indices.end(),
            new_points.begin(),
            point_map,
            wlop_params);

        benchmark::DoNotOptimize(new_points);
    }
}

BENCHMARK(bm_average_distance_seq)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_average_distance_par)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_normal_estimation_seq)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_normal_estimation_par)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_point_bilateral_filter_seq)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_point_bilateral_filter_par)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_wlop_seq)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;
BENCHMARK(bm_wlop_par)
    ->Unit(benchmark::kMillisecond)
    ->Args({0u})
    ->Args({1u})
    ->Args({2u})
    //->Args({3u});
    ;

int main(int argc, char** argv)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    auto [detergent_points, detergent_normals, detergent_colors] =
        pcp::io::read_ply<point_type, normal_type>(
            std::filesystem::path{"./examples/data/detergent.ply"});
    auto [bunny_points, bunny_normals, bunny_colors] = pcp::io::read_ply<point_type, normal_type>(
        std::filesystem::path{"./examples/data/stanford_bunny.ply"});
    auto [happy_buddha_points, happy_buddha_normals, happy_buddha_colors] =
        pcp::io::read_ply<point_type, normal_type>(
            std::filesystem::path{"./examples/data/happy_buddha.ply"});
    //auto [dragon_points, dragon_normals, dragon_colors] =
    //    pcp::io::read_ply<point_type, normal_type>(
    //        std::filesystem::path{"./examples/data/xyzrgb_dragon.ply"});

    point_clouds[0] = std::move(detergent_points);
    point_clouds[1] = std::move(bunny_points);
    point_clouds[2] = std::move(happy_buddha_points);
    //point_clouds[3] = std::move(dragon_points);

    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
}