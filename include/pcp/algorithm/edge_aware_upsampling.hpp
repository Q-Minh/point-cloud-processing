#ifndef PCP_ALGORITHM_EDGE_AWARE_UPSAMPLING_HPP
#define PCP_ALGORITHM_EDGE_AWARE_UPSAMPLING_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/axis_aligned_bounding_box.hpp"
#include "pcp/common/sphere.hpp"
#include "pcp/common/vector3d.hpp"
#include "pcp/octree/linked_octree.hpp"
#include "pcp/traits/normal_map.hpp"
#include "pcp/traits/output_iterator_traits.hpp"
#include "pcp/traits/point_map.hpp"

#include <algorithm>
#include <cstddef>
#include <execution>
#include <type_traits>

namespace pcp {
namespace algorithm {
namespace ear {

struct params_t
{
    double edge_sensitivity                      = 5.;
    std::size_t output_point_count               = 0u;
    double sharpness_angle                       = 0.261799;
    double neighborhood_radius                   = 0.;
    std::size_t max_iteration_count              = 20u;
    double priority_estimation_subset_proportion = 0.05;
};

namespace detail {

template <class SpatialAccelerationStructure, class PointMap, class NormalMap, class ScalarType>
std::pair<std::size_t, ScalarType> compute_base_point_with_priority(
    std::size_t const i,
    ScalarType const neighborhood_radius,
    SpatialAccelerationStructure const& spatial_searcher,
    PointMap const& point_map,
    NormalMap const& normal_map,
    ScalarType const edge_sensitivity)
{
    using point_type  = std::invoke_result_t<PointMap, std::size_t>;
    using normal_type = std::invoke_result_t<NormalMap, std::size_t>;
    using scalar_type = ScalarType;

    point_type const& pi = point_map(i);
    normal_type const ni = normal_map(i);

    sphere_t<point_type> ball{};
    ball.position = pi;
    ball.radius   = neighborhood_radius;

    auto const neighbours = spatial_searcher.range_search(ball, point_map);

    if (neighbours.empty())
        return {i, scalar_type{0.}};

    auto const normal_inner_product = [](normal_type const& n1, normal_type const& n2) {
        return n1.nx() * n2.nx() + n1.ny() * n2.ny() + n1.nz() * n2.nz();
    };
    auto const evaluate_plane_function = [](normal_type const& n, point_type const& p) {
        return n.nx() * p.x() + n.ny() * p.y() + n.nz() * p.z();
    };

    auto begin = neighbours.begin();
    auto end   = neighbours.end();

    scalar_type constexpr half{0.5};
    scalar_type constexpr two{2.};

    /**
     * These nested loops can be optimized. Pairwise tests can be n(n+1)/2 instead of n^2
     * if each pair is only tested once. These nested loops will test pairs twice currently.
     */

    // the base of this neighborhood is the solution to
    // argmax argmin (density metric * distance to plane metric) w.r.t. base
    scalar_type max  = std::numeric_limits<scalar_type>::lowest();
    std::size_t base = i;
    for (auto outer_it = begin; outer_it != end; ++outer_it)
    {
        std::size_t const k   = *outer_it;
        point_type const& pk  = point_map(k);
        normal_type const& nk = normal_map(k);

        common::basic_vector3d_t<scalar_type> const diff = pk - pi;
        point_type const midpoint                        = pi + half * diff;

        scalar_type const orthogonality        = normal_inner_product(ni, nk);
        scalar_type const priority_coefficient = std::pow(two - orthogonality, edge_sensitivity);

        scalar_type min = std::numeric_limits<scalar_type>::max();
        for (auto inner_it = begin; inner_it != end; ++inner_it)
        {
            std::size_t const j   = *inner_it;
            point_type const& pj  = point_map(j);
            normal_type const& nj = normal_map(j);

            common::basic_vector3d_t<scalar_type> const diff_j = midpoint - pj;
            scalar_type const d =
                evaluate_plane_function(nj, point_type{diff_j.x(), diff_j.y(), diff_j.z()});
            auto const point_to_plane_vector =
                d * common::basic_vector3d_t<scalar_type>{nj.nx(), nj.ny(), nj.nz()};

            common::basic_vector3d_t<scalar_type> const error = diff_j - point_to_plane_vector;
            scalar_type const quadratic_error                 = common::norm(error);

            if (quadratic_error < min)
                min = quadratic_error;
        }
        min *= priority_coefficient;

        if (min > max)
        {
            max  = min;
            base = k;
        }
    }

    return {base, max};
}

template <
    class SpatialAccelerationStructure,
    class PointMap,
    class NormalMap,
    class Theta,
    class Psi,
    class ScalarType>
std::pair<std::invoke_result_t<PointMap, std::size_t>, std::invoke_result_t<NormalMap, std::size_t>>
insert_point(
    std::invoke_result_t<PointMap, std::size_t> const& base_point,
    std::size_t const father,
    std::size_t const mother,
    ScalarType const neighborhood_radius,
    SpatialAccelerationStructure const& spatial_searcher,
    PointMap const& point_map,
    NormalMap const& normal_map,
    Theta const& theta,
    Psi const& psi)
{
    using scalar_type = ScalarType;
    using point_type  = std::invoke_result_t<PointMap, std::size_t>;
    using normal_type = std::invoke_result_t<NormalMap, std::size_t>;
    using vector_type = common::basic_vector3d_t<scalar_type>;

    point_type const& father_point   = point_map(father);
    normal_type const& father_normal = normal_map(father);
    point_type const& mother_point   = point_map(mother);
    normal_type const& mother_normal = normal_map(mother);

    auto const evaluate_plane_function = [](normal_type const& n, vector_type const& p) {
        return n.nx() * p.x() + n.ny() * p.y() + n.nz() * p.z();
    };

    auto const dk_nk = [&](point_type const& bl, normal_type const& nl) {
        sphere_t<point_type> radial_support_region{};
        radial_support_region.position = bl;
        radial_support_region.radius   = neighborhood_radius;

        auto const neighbours = spatial_searcher.range_search(radial_support_region, point_map);

        scalar_type dk{0.};
        vector_type nk{0., 0., 0.};
        scalar_type normalization_coefficient{0.};
        for (auto const& neighbour : neighbours)
        {
            point_type const& pi  = point_map(neighbour);
            normal_type const& ni = normal_map(neighbour);

            vector_type const diff   = base_point - pi;
            scalar_type const r      = common::norm(diff);
            scalar_type const d      = evaluate_plane_function(nl, diff);
            scalar_type const wtheta = theta(r);
            scalar_type const wpsi   = psi(nl, ni);
            scalar_type const coeff  = wtheta * wpsi;

            dk += d * coeff;
            nk = nk + coeff * vector_type{ni.nx(), ni.ny(), ni.nz()};
            normalization_coefficient += coeff;
        }
        dk /= normalization_coefficient;
        nk = nk / normalization_coefficient;
        return std::make_pair(dk, normal_type{nk.x(), nk.y(), nk.z()});
    };

    auto const [dfather, nfather] = dk_nk(base_point, father_normal);
    auto const [dmother, nmother] = dk_nk(base_point, mother_normal);

    scalar_type const d   = (dfather < dmother) ? dfather : dmother;
    normal_type const& nk = (dfather < dmother) ? nfather : nmother;
    point_type const& pk  = base_point + d * vector_type{nk.nx(), nk.ny(), nk.nz()};

    return {pk, nk};
}

} // namespace detail
} // namespace ear

template <
    class RandomAccessIter,
    class PointOutputIter,
    class NormalOutputIter,
    class PointMap,
    class NormalMap>
std::pair<PointOutputIter, NormalOutputIter> edge_aware_upsampling(
    RandomAccessIter begin,
    RandomAccessIter end,
    PointOutputIter point_out_begin,
    NormalOutputIter normal_out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    ear::params_t const& params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using input_normal_type  = std::invoke_result_t<NormalMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_point_type  = typename xstd::output_iterator_traits<PointOutputIter>::value_type;
    using output_normal_type = typename xstd::output_iterator_traits<NormalOutputIter>::value_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    static_assert(
        traits::is_normal_map_v<NormalMap, input_element_type>,
        "normal_map must satisfy NormalMap concept");

    std::size_t const N = static_cast<std::size_t>(std::distance(begin, end));
    std::vector<std::size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0u);

    octree_parameters_t<input_point_type> octree_params{};
    octree_params.max_depth     = 12u;
    octree_params.node_capacity = 64u;
    octree_params.voxel_grid    = bounding_box(begin, end, point_map);

    std::vector<output_point_type> points{};
    points.reserve(N);
    std::transform(begin, end, std::back_inserter(points), [&](input_element_type const& e) {
        input_point_type const& p = point_map(e);
        return output_point_type{p.x(), p.y(), p.z()};
    });
    std::vector<output_normal_type> normals{};
    normals.reserve(N);
    std::transform(begin, end, std::back_inserter(normals), [&](input_element_type const& e) {
        input_normal_type const& n = normal_map(e);
        return output_normal_type{n.nx(), n.ny(), n.nz()};
    });

    auto const internal_point_map = [&](std::size_t const i) {
        return points[i];
    };
    auto const internal_normal_map = [&](std::size_t const i) {
        return normals[i];
    };

    scalar_type const edge_sensitivity    = static_cast<scalar_type>(params.edge_sensitivity);
    std::size_t const K                   = params.max_iteration_count;
    std::size_t const output_point_count  = params.output_point_count;
    scalar_type const sharpness_angle     = static_cast<scalar_type>(params.sharpness_angle);
    scalar_type const neighborhood_radius = static_cast<scalar_type>(params.neighborhood_radius);
    scalar_type const priority_estimation_subset_proportion =
        static_cast<scalar_type>(params.priority_estimation_subset_proportion);

    auto const theta = [sigma = neighborhood_radius](scalar_type const r) {
        scalar_type const r2    = r * r;
        scalar_type const s2    = sigma * sigma;
        scalar_type const power = -(r2 / s2);
        return std::exp(power);
    };

    auto const psi =
        [sigma = sharpness_angle](input_normal_type const& ni, input_normal_type const& nip) {
            scalar_type constexpr one{1.};
            scalar_type const num =
                one - common::inner_product(
                          common::basic_vector3d_t<scalar_type>{ni.x(), ni.y(), ni.z()},
                          common::basic_vector3d_t<scalar_type>{nip.x(), nip.y(), nip.z()});
            scalar_type const den    = one - std::cos(sigma);
            scalar_type const power  = num / den;
            scalar_type const power2 = power * power;
            scalar_type const wpsi   = std::exp(-power2);
            return wpsi;
        };

    basic_linked_octree_t<std::size_t, decltype(octree_params)> octree{
        indices.begin(),
        indices.end(),
        internal_point_map};

    auto const estimate_point_cloud_mean_priority =
        [&](scalar_type const sample_subset_proportion) {
            std::size_t const subset_sample_count = sample_subset_proportion * N;
            scalar_type const priority_sum        = std::accumulate(
                indices.begin(),
                indices.end(),
                scalar_type{0.},
                [&](scalar_type const accumulator, std::size_t const i) {
                    auto const [_, priority] = ear::detail::compute_base_point_with_priority(
                        i,
                        neighborhood_radius,
                        octree,
                        internal_point_map,
                        internal_normal_map,
                        edge_sensitivity);
                    return accumulator + priority;
                });
            scalar_type const priority_estimation =
                priority_sum / static_cast<scalar_type>(subset_sample_count);
            return priority_estimation;
        };

    scalar_type priority_threshold =
        estimate_point_cloud_mean_priority(priority_estimation_subset_proportion);

    for (std::size_t iter = 0u; iter < K; ++iter)
    {
        std::vector<bool> is_not_priority(points.size());

        auto const should_keep_inserting = [&](std::size_t const i) {
            return i < points.size() && points.size() < output_point_count;
        };

        scalar_type priority_threshold_sum{0.};
        scalar_type priority_threshold_normalization_coefficient{1.};

        std::size_t const previous_point_count = points.size();
        for (std::size_t i = 0u; should_keep_inserting(i); ++i)
        {
            auto const [base, priority] = ear::detail::compute_base_point_with_priority(
                i,
                neighborhood_radius,
                octree,
                internal_point_map,
                internal_normal_map,
                edge_sensitivity);

            if (priority < priority_threshold)
            {
                is_not_priority[i] = true;
                continue;
            }

            priority_threshold_sum += priority;
            priority_threshold_normalization_coefficient += scalar_type{1.};

            std::size_t const father = i;
            std::size_t const mother = base;

            output_point_type const& neighborhood_point = internal_point_map(father);
            output_point_type const& base_point         = internal_point_map(mother);

            common::basic_vector3d_t<scalar_type> const diff = base_point - neighborhood_point;
            output_point_type const midpoint = neighborhood_point + scalar_type{0.5} * diff;
            std::size_t const k              = points.size();

            auto const [pk, nk] = ear::detail::insert_point(
                midpoint,
                father,
                mother,
                neighborhood_radius,
                octree,
                internal_point_map,
                internal_normal_map,
                theta,
                psi);

            indices.push_back(k);
            points.push_back(pk);
            normals.push_back(nk);
            octree.insert(k, internal_point_map);
            is_not_priority.push_back(false);
        }
        std::size_t const point_count = points.size();

        if (point_count > previous_point_count)
        {
            priority_threshold = scalar_type{0.68} * priority_threshold_sum /
                                 priority_threshold_normalization_coefficient;
        }
        else
        {
            // priority threshold may have been too high, decrease it
            priority_threshold *= scalar_type{0.68};
        }
    }

    auto point_it  = std::copy(points.begin(), points.end(), point_out_begin);
    auto normal_it = std::copy(normals.begin(), normals.end(), normal_out_begin);

    return {point_it, normal_it};
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_EDGE_AWARE_UPSAMPLING_HPP