#include <benchmark/benchmark.h>
#include <pcp/kdtree/kdtree.hpp>
#include <pcp/octree/octree.hpp>
#include <random>

auto const default_point_map = [](pcp::point_t const& p) {
    return p;
};

auto const default_coordinate_map = [](pcp::point_t const& p) {
    return std::array<float, 3u>{p.x(), p.y(), p.z()};
};

float constexpr get_bm_min()
{
    return -100.f;
}
float constexpr get_bm_max()
{
    return 100.f;
}

static std::vector<pcp::point_t>
get_vector_of_points(std::uint64_t num_points, float const min, float const max)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> coordinate_distribution(min, max);

    std::vector<pcp::point_t> points;
    std::uint64_t const size = num_points;
    points.reserve(size);
    for (std::uint64_t i = 0; i < size; ++i)
    {
        points.push_back(pcp::point_t{
            coordinate_distribution(gen),
            coordinate_distribution(gen),
            coordinate_distribution(gen)});
    }

    return points;
}

static pcp::axis_aligned_bounding_box_t<pcp::point_t> get_range(float const min, float const max)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> coordinate_distribution(min + 1.f, max - 1.f);
    std::uniform_real_distribution<float> min_bound(-1.f, 0.f);
    std::uniform_real_distribution<float> max_bound(0.f, 1.f);

    auto const x = coordinate_distribution(gen);
    auto const y = coordinate_distribution(gen);
    auto const z = coordinate_distribution(gen);

    return pcp::axis_aligned_bounding_box_t<pcp::point_t>{
        pcp::point_t{min_bound(gen) + x, min_bound(gen) + y, min_bound(gen) + z},
        pcp::point_t{max_bound(gen) + x, max_bound(gen) + y, max_bound(gen) + z}};
}

static pcp::point_t get_reference_point(float const min, float const max)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> coordinate_distribution(min, max);
    return pcp::point_t{
        coordinate_distribution(gen),
        coordinate_distribution(gen),
        coordinate_distribution(gen)};
}

static void bm_vector_construction(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> const points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);
    for (auto _ : state)
    {
        std::vector<pcp::point_t> v(points.cbegin(), points.cend());
        benchmark::DoNotOptimize(points.data());
    }
}

static void bm_linked_octree_construction(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> const points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);
    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid = pcp::axis_aligned_bounding_box_t<pcp::point_t>{
        pcp::point_t{min, min, min},
        pcp::point_t{max, max, max}};
    params.node_capacity = static_cast<std::uint32_t>(state.range(1));
    params.max_depth     = static_cast<decltype(params.max_depth)>(state.range(2));

    for (auto _ : state)
    {
        pcp::linked_octree_t octree(points.cbegin(), points.cend(), default_point_map, params);
        benchmark::DoNotOptimize(octree.size());
    }
}

static void bm_linked_kdtree_construction(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> const points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);

    pcp::kdtree::construction_params_t params;
    params.max_depth                           = static_cast<std::size_t>(state.range(1));
    params.construction                        = pcp::kdtree::construction_t::nth_element;

    for (auto _ : state)
    {
        pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(default_coordinate_map)> kdtree{
            points.begin(),
            points.end(),
            default_coordinate_map,
            params};
        benchmark::DoNotOptimize(kdtree.size());
    }
}

static void bm_vector_range_search(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);
    for (auto _ : state)
    {
        auto const range = get_range(min, max);
        std::vector<pcp::point_t> found_points;
        for (auto const& p : points)
        {
            if (range.contains(p))
                found_points.push_back(p);
        }
        benchmark::DoNotOptimize(found_points.data());
    }
}

static void bm_linked_octree_range_search(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);

    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid =
        pcp::axis_aligned_bounding_box_t<pcp::point_t>{{min, min, min}, {max, max, max}};
    params.node_capacity = static_cast<std::uint32_t>(state.range(1));
    params.max_depth     = static_cast<decltype(params.max_depth)>(state.range(2));
    pcp::linked_octree_t octree(points.cbegin(), points.cend(), default_point_map, params);

    for (auto _ : state)
    {
        pcp::axis_aligned_bounding_box_t range = get_range(min, max);
        std::vector<pcp::point_t> found_points = octree.range_search(range, default_point_map);
        benchmark::DoNotOptimize(found_points.data());
    }
}

static void bm_vector_knn_search(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);
    std::int64_t const k = state.range(1);
    for (auto _ : state)
    {
        pcp::point_t const reference = get_reference_point(min, max);

        auto const distance = [](pcp::point_t const& p1, pcp::point_t const& p2) -> float {
            auto const dx = p2.x() - p1.x();
            auto const dy = p2.y() - p1.y();
            auto const dz = p2.z() - p1.z();
            return dx * dx + dy * dy + dz * dz;
        };
        auto const less_than = [reference,
                                distance](pcp::point_t const& p1, pcp::point_t const& p2) -> bool {
            return distance(p1, reference) < distance(p2, reference);
        };

        std::sort(points.begin(), points.end(), less_than);
        std::vector<pcp::point_t> knn(points.cbegin(), points.cbegin() + k);
        benchmark::DoNotOptimize(knn.data());
    }
}

static void bm_linked_octree_knn_search(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);

    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid =
        pcp::axis_aligned_bounding_box_t<pcp::point_t>{{min, min, min}, {max, max, max}};
    params.node_capacity = static_cast<std::uint32_t>(state.range(1));
    params.max_depth     = static_cast<decltype(params.max_depth)>(state.range(2));

    pcp::linked_octree_t octree(points.cbegin(), points.cend(), default_point_map, params);
    std::uint64_t const k = static_cast<std::uint64_t>(state.range(3));
    for (auto _ : state)
    {
        auto const reference          = get_reference_point(min, max);
        std::vector<pcp::point_t> knn = octree.nearest_neighbours(reference, k, default_point_map);
        benchmark::DoNotOptimize(knn.data());
    }
}

static void bm_vector_iterator_traversal(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);

    for (auto _ : state)
    {
        bool const all = std::all_of(std::cbegin(points), std::cend(points), [](auto const& p) {
            return pcp::common::are_vectors_equal(p, p);
        });
        benchmark::DoNotOptimize(all);
    }
}

static void bm_linked_octree_iterator_traversal(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);

    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid =
        pcp::axis_aligned_bounding_box_t<pcp::point_t>{{min, min, min}, {max, max, max}};
    params.node_capacity = static_cast<std::uint32_t>(state.range(1));
    params.max_depth     = static_cast<decltype(params.max_depth)>(state.range(2));

    pcp::linked_octree_t octree(points.cbegin(), points.cend(), default_point_map, params);
    for (auto _ : state)
    {
        bool const all = std::all_of(octree.cbegin(), octree.cend(), [](auto const& p) {
            return pcp::common::are_vectors_equal(p, p);
        });
        benchmark::DoNotOptimize(all);
    }
}

static void bm_linked_kdtree_iterator_traversal(benchmark::State& state)
{
    auto constexpr min = get_bm_min();
    auto constexpr max = get_bm_max();
    std::vector<pcp::point_t> points =
        get_vector_of_points(static_cast<std::uint64_t>(state.range(0)), min, max);

    pcp::kdtree::construction_params_t params;
    params.max_depth                           = static_cast<std::size_t>(state.range(1));
    params.construction                        = pcp::kdtree::construction_t::nth_element;

    pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(default_coordinate_map)> kdtree(
        points.begin(),
        points.end(),
        default_coordinate_map,
        params);
    for (auto _ : state)
    {
        bool const all = std::all_of(kdtree.cbegin(), kdtree.cend(), [](auto const& p) {
            return pcp::common::are_vectors_equal(p, p);
        });
        benchmark::DoNotOptimize(all);
    }
}

BENCHMARK(bm_vector_construction)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12})
    ->Args({1 << 16})
    ->Args({1 << 20})
    ->Args({1 << 24});
BENCHMARK(bm_linked_octree_construction)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 32u, 11u})
    ->Args({1 << 16, 32u, 11u})
    ->Args({1 << 20, 32u, 11u})
    ->Args({1 << 24, 32u, 11u})
    ->Args({1 << 12, 32u, 21u})
    ->Args({1 << 16, 32u, 21u})
    ->Args({1 << 20, 32u, 21u})
    ->Args({1 << 24, 32u, 21u})
    ->Args({1 << 12, 512u, 21u})
    ->Args({1 << 16, 512u, 21u})
    ->Args({1 << 20, 512u, 21u})
    ->Args({1 << 24, 512u, 21u});
BENCHMARK(bm_linked_kdtree_construction)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 11u})
    ->Args({1 << 16, 11u})
    ->Args({1 << 20, 11u})
    ->Args({1 << 24, 11u})
    ->Args({1 << 12, 21u})
    ->Args({1 << 16, 21u})
    ->Args({1 << 20, 21u})
    ->Args({1 << 24, 21u});
BENCHMARK(bm_vector_range_search)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12})
    ->Args({1 << 16})
    ->Args({1 << 20})
    ->Args({1 << 24});
BENCHMARK(bm_linked_octree_range_search)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 32u, 21u})
    ->Args({1 << 16, 32u, 21u})
    ->Args({1 << 20, 32u, 21u})
    ->Args({1 << 24, 32u, 21u})
    ->Args({1 << 12, 512u, 21u})
    ->Args({1 << 16, 512u, 21u})
    ->Args({1 << 20, 512u, 21u})
    ->Args({1 << 24, 512u, 21u});
BENCHMARK(bm_vector_knn_search)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 10u})
    ->Args({1 << 16, 10u})
    ->Args({1 << 20, 10u})
    ->Args({1 << 24, 10u});
BENCHMARK(bm_linked_octree_knn_search)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 32u, 21u, 10u})
    ->Args({1 << 16, 32u, 21u, 10u})
    ->Args({1 << 20, 32u, 21u, 10u})
    ->Args({1 << 24, 32u, 21u, 10u})
    ->Args({1 << 12, 512u, 21u, 10u})
    ->Args({1 << 16, 512u, 21u, 10u})
    ->Args({1 << 20, 512u, 21u, 10u})
    ->Args({1 << 24, 512u, 21u, 10u});
BENCHMARK(bm_vector_iterator_traversal)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12})
    ->Args({1 << 16})
    ->Args({1 << 20})
    ->Args({1 << 24});
BENCHMARK(bm_linked_octree_iterator_traversal)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 32u, 21u})
    ->Args({1 << 16, 32u, 21u})
    ->Args({1 << 20, 32u, 21u})
    ->Args({1 << 24, 32u, 21u})
    ->Args({1 << 12, 512u, 21u})
    ->Args({1 << 16, 512u, 21u})
    ->Args({1 << 20, 512u, 21u})
    ->Args({1 << 24, 512u, 21u});
BENCHMARK(bm_linked_kdtree_iterator_traversal)
    ->Unit(benchmark::kMillisecond)
    ->Args({1 << 12, 21u})
    ->Args({1 << 16, 21u})
    ->Args({1 << 20, 21u})
    ->Args({1 << 24, 21u});

int main(int argc, char** argv)
{
    ::benchmark::Initialize(&argc, argv);
    if (::benchmark::ReportUnrecognizedArguments(argc, argv))
        return 1;
    ::benchmark::RunSpecifiedBenchmarks();
}
