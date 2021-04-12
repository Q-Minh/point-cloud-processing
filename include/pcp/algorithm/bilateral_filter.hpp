#ifndef PCP_ALGORITHM_BILATERAL_FILTER_HPP
#define PCP_ALGORITHM_BILATERAL_FILTER_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/norm.hpp"
#include "pcp/common/vector3d.hpp"
#include "pcp/kdtree/linked_kdtree.hpp"
#include "pcp/traits/output_iterator_traits.hpp"
#include "pcp/traits/point_map.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <execution>
#include <vector>

namespace pcp {
namespace algorithm {
namespace bilateral {

/**
 * @brief
 * Parameters for the bilateral filtering algorithm
 */
struct params_t
{
    double sigmaf = 1.;  ///< Standard deviation for the support region of spatial weight function f
    double sigmag = 0.1; ///< Standard deviation for the influence weight function g
    std::size_t K = 1u;  ///< Number of iterations of bilateral filtering
};

namespace detail {

template <
    class KdTree,
    class CoordinateMap,
    class PointMap,
    class NormalMap,
    class SpatialWeightFunction,
    class InfluenceWeightFunction,
    class ProjectionFunction,
    class ScalarType>
std::invoke_result_t<PointMap, std::size_t> compute_pi(
    std::size_t const i,
    ScalarType const sigmaf,
    ScalarType const sigmag,
    KdTree const& kdtree,
    CoordinateMap const& coordinate_map,
    PointMap const& point_map,
    NormalMap const& normal_map,
    SpatialWeightFunction const& f,
    InfluenceWeightFunction const& g,
    ProjectionFunction const& projection)
{
    using scalar_type = ScalarType;
    using point_type  = std::invoke_result_t<PointMap, std::size_t>;
    using normal_type = std::invoke_result_t<NormalMap, std::size_t>;

    scalar_type constexpr two  = scalar_type{2.};
    scalar_type constexpr zero = scalar_type{0.};

    auto const s  = point_map(i);
    auto const ci = coordinate_map(i);

    sphere_a<scalar_type> support_region{};
    support_region.position = ci;
    support_region.radius   = two * sigmaf;

    auto const neighbors = kdtree.range_search(support_region);

    scalar_type k = zero;
    point_type sprime{zero, zero, zero};
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
    {
        point_type const p           = point_map(*it);
        normal_type const np         = normal_map(*it);
        point_type const s_projected = projection(p, np, s);

        scalar_type const rf = common::norm(s - p);
        scalar_type const rg = common::norm(s_projected - s);

        scalar_type const wf = f(sigmaf, rf);
        scalar_type const wg = g(sigmag, rg);

        scalar_type const w = wf * wg;
        k += w;
        common::basic_vector3d_t<scalar_type> translation{
            w * s_projected.x(),
            w * s_projected.y(),
            w * s_projected.z()};
        sprime = sprime + translation;
    }
    sprime = sprime / k;

    return sprime;
}

template <
    class KdTree,
    class CoordinateMap,
    class PointMap,
    class NormalMap,
    class SpatialWeightFunction,
    class SpatialWeightFunctionDerivative,
    class InfluenceWeightFunction,
    class InfluenceWeightFunctionDerivative,
    class ProjectionFunction,
    class ScalarType>
std::invoke_result_t<NormalMap, std::size_t> compute_ni(
    std::size_t const i,
    ScalarType const sigmaf,
    ScalarType const sigmag,
    KdTree const& kdtree,
    CoordinateMap const& coordinate_map,
    PointMap const& point_map,
    NormalMap const& normal_map,
    SpatialWeightFunction const& f,
    SpatialWeightFunctionDerivative const& df,
    InfluenceWeightFunction const& g,
    InfluenceWeightFunctionDerivative const& dg,
    ProjectionFunction const& projection)
{
    using scalar_type           = ScalarType;
    using point_type            = std::invoke_result_t<PointMap, std::size_t>;
    using normal_type           = std::invoke_result_t<NormalMap, std::size_t>;
    using matrix_3d_type        = Eigen::Matrix<scalar_type, 3, 3>;
    using column_vector_3d_type = Eigen::Matrix<scalar_type, 3, 1>;
    using row_vector_3d_type    = Eigen::Matrix<scalar_type, 1, 3>;

    scalar_type constexpr two  = scalar_type{2.};
    scalar_type constexpr zero = scalar_type{0.};

    point_type const pcp_s = point_map(i);
    column_vector_3d_type const s{pcp_s.x(), pcp_s.y(), pcp_s.z()};
    auto const ci = coordinate_map(i);

    sphere_a<scalar_type> support_region{};
    support_region.position = ci;
    support_region.radius   = two * sigmaf;

    auto const neighbors = kdtree.range_search(support_region);

    /**
     * Jacobian of sum of:
     *
     * projection(s) * f(||s - p||) * g(||projection(s) - s||)
     */
    matrix_3d_type J_pi_f_g;
    J_pi_f_g.setZero();
    /**
     * sum of:
     *
     * projection(s) * f(||s - p||) * g(||projection(s) - s||)
     */
    column_vector_3d_type pi_f_g;
    pi_f_g.setZero();
    /**
     * Gradient of k(s)
     */
    row_vector_3d_type grad_k;
    grad_k.setZero();
    /**
     * k(s)
     */
    scalar_type k = zero;

    for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
    {
        point_type const pcp_p           = point_map(*it);
        normal_type const pcp_np         = normal_map(*it);
        point_type const pcp_s_projected = projection(pcp_p, pcp_np, pcp_s);

        column_vector_3d_type const p{pcp_p.x(), pcp_p.y(), pcp_p.z()};
        column_vector_3d_type const np{pcp_np.nx(), pcp_np.ny(), pcp_np.nz()};
        column_vector_3d_type const s_projected{
            pcp_s_projected.x(),
            pcp_s_projected.y(),
            pcp_s_projected.z()};

        column_vector_3d_type const sp  = s - p;
        column_vector_3d_type const sps = s_projected - s;
        scalar_type const rf            = sp.norm();
        scalar_type const rg            = sps.norm();

        // f(||s - p||)
        scalar_type const wf = f(sigmaf, rf);
        // g(||projection(s) - s||)
        scalar_type const wg = g(sigmag, rg);

        scalar_type const w = wf * wg;

        // k(s) = sum f(||s - p||) * g(||projection(s) - s||)
        k += w;

        // projection(s) * f(||s - p||) * g(||projection(s) - s||)
        column_vector_3d_type translation{
            w * pcp_s_projected.x(),
            w * pcp_s_projected.y(),
            w * pcp_s_projected.z()};
        pi_f_g += translation;

        // derivative df/dr | r=||s-p||
        scalar_type const wdf            = df(sigmaf, rf);
        row_vector_3d_type const sp_unit = sp.normalized().transpose();
        // grad(f) = (s - p) / ||s - p|| * (df/dr | r=||s-p||)
        row_vector_3d_type const grad_f = sp_unit * wdf;

        // Jacobian of projection(s)
        matrix_3d_type Jpi;
        Jpi(0, 0) = 1 - (np.x() * np.x());
        Jpi(1, 1) = 1 - (np.y() * np.y());
        Jpi(2, 2) = 1 - (np.z() * np.z());
        Jpi(0, 1) = np.x() * np.y();
        Jpi(0, 2) = np.x() * np.z();
        Jpi(1, 2) = np.y() * np.z();
        Jpi(1, 0) = Jpi(0, 1);
        Jpi(2, 0) = Jpi(0, 2);
        Jpi(2, 1) = Jpi(1, 2);

        // derivative dg/dr | r = ||projection(s) - s||
        scalar_type const wdg             = dg(sigmag, rg);
        row_vector_3d_type const sps_unit = sps.normalized().transpose();
        /**
         * Let sps_unit = (projection(s) - s) / ||projection(s) - s||
         *
         * grad(g) =
         * (sps_unit * Jacobian(projection(s)) - sps_unit) *
         * (dg/dr | r = ||projection(s) - s||)
         */
        row_vector_3d_type const grad_g = (sps_unit * Jpi - sps_unit) * wdg;

        /**
         * Product rule grad(f(||s - p||) * g(||projection(s) - s||))
         */
        grad_k += (grad_f * wg) + (wf * grad_g);
        /**
         * Product rule grad(projection(s) * f(||s - p||) * g(||projection(s) - s||))
         */
        J_pi_f_g += (Jpi * wf * wg) + (sps * grad_f * wg) + (sps * wf * grad_g);
    }

    /**
     * Quotient rule grad(u/v) = (1/v^2) * (grad(u)*v - u*grad(v))
     */
    scalar_type const ks2_inv = scalar_type{1.} / (k * k);
    matrix_3d_type const J    = ks2_inv * (J_pi_f_g * k - pi_f_g * grad_k);

    normal_type const pcp_ns = normal_map(i);
    column_vector_3d_type const ns{pcp_ns.nx(), pcp_ns.ny(), pcp_ns.nz()};

    /**
     * In the original normal improvement paper, they use the inverse transpose
     * of the jacobian. In our case, we directly use the jacobian
     * of the filter F(s), because it is in line with the intuition
     * of using the local spatial deformation of the field F(s) to
     * adjust normals.
     */
    // ns' = J^(-T) * ns
    // matrix_3d_type const adj       = J.adjoint();
    column_vector_3d_type ns_prime = J * ns;
    ns_prime.normalize();
    return normal_type{ns_prime.x(), ns_prime.y(), ns_prime.z()};
}

} // namespace detail
} // namespace bilateral

/**
 * @ingroup smoothing-algorithm
 * @brief
 * Uses a bilateral filter to smooth an input point cloud.
 *
 * The bilateral filter is defined in the same way as in
 * 'Jones, Thouis R., Fredo Durand, and Matthias Zwicker. "Normal improvement for point rendering."
 * IEEE Computer Graphics and Applications 24.4 (2004): 53-56.'
 *
 * Internally, the filtering needs a temporary copy of the points p(k)
 * to compute p(k+1), the points at the next iteration. At each iteration,
 * a kd-tree must be built over the points p(k) to support optimized
 * range search queries.
 *
 * The transformation F (the bilateral filter) over our points P is done in parallel at each
 * iteration k.
 *
 * @tparam ExecutionPolicy The execution policy type
 * @tparam RandomAccessIter Iterator type satisfying Random Access requirements
 * @tparam OutputIter Iterator type dereferenceable to a type satisfying Point concept
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam NormalMap Type satisfying NormalMap concept
 * @param begin Start iterator of input point cloud
 * @param end End iterator of input point cloud
 * @param out_begin Start iterator of output points
 * @param point_map The point map property map
 * @param normal_map The normal map property map
 * @param params The bilateral filter algorithm's parameters
 * @return End iterator of output sequence
 */
template <
    class ExecutionPolicy,
    class RandomAccessIter,
    class OutputIter,
    class PointMap,
    class NormalMap>
OutputIter bilateral_filter_points(
    ExecutionPolicy&& policy,
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    bilateral::params_t const& params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using input_normal_type  = std::invoke_result_t<NormalMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_point_type  = typename xstd::output_iterator_traits<OutputIter>::value_type;
    // using difference_type    = typename std::iterator_traits<RandomAccessIter>::difference_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    static_assert(
        traits::is_point_v<output_point_type>,
        "OutputIter must be dereferenceable to a type satisfying Point concept");

    std::size_t const N        = static_cast<std::size_t>(std::distance(begin, end));
    scalar_type const sigmaf   = static_cast<scalar_type>(params.sigmaf);
    scalar_type const sigmag   = static_cast<scalar_type>(params.sigmag);
    std::size_t const K        = params.K;
    scalar_type constexpr pi   = static_cast<scalar_type>(3.14159265358979323846);
    scalar_type constexpr zero = scalar_type{0.};

    assert(K > 0u);
    assert(N > 0u);
    assert(sigmaf > zero);
    assert(sigmag > zero);

    std::vector<std::size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0u);

    std::vector<input_point_type> points(N);
    std::transform(begin, end, points.begin(), [&](input_element_type const& e) {
        return point_map(e);
    });

    std::vector<input_point_type> temporary_points(N);

    std::vector<input_normal_type> normals(N);
    std::transform(begin, end, normals.begin(), [&](input_element_type const& e) {
        return normal_map(e);
    });

    auto const internal_point_map = [&](std::size_t const i) {
        return points[i];
    };
    auto const internal_normal_map = [&](std::size_t const i) {
        return normals[i];
    };

    auto const gaussian = [=](scalar_type const sigma, scalar_type const r) -> scalar_type {
        scalar_type const s2      = sigma * sigma;
        scalar_type const r2      = r * r;
        scalar_type const power   = -r2 / (2 * s2);
        scalar_type constexpr one = scalar_type{1.};
        scalar_type constexpr two = scalar_type{2.};
        scalar_type const coeff   = one / (sigma * std::sqrt(two * pi));
        return coeff * std::exp(power);
    };

    auto const projection = [&](input_point_type const& p,
                                input_normal_type const& np,
                                input_point_type const& s) -> input_point_type {
        auto const sp = p - s;
        common::basic_vector3d_t<scalar_type> const n{np.nx(), np.ny(), np.nz()};
        auto const d = common::inner_product(sp, n);
        return s + d * n;
    };

    using coordinates_type = std::array<scalar_type, 3u>;

    auto const coordinate_map = [&](std::size_t const pe) {
        auto const p = internal_point_map(pe);
        return coordinates_type{p.x(), p.y(), p.z()};
    };

    kdtree::construction_params_t kdtree_params;
    kdtree_params.compute_max_depth     = true;
    kdtree_params.construction          = kdtree::construction_t::nth_element;
    kdtree_params.max_elements_per_leaf = 64u;
    kdtree_params.min_element_count_for_parallel_exec = indices.size();

    for (std::size_t k = 0u; k < K; ++k)
    {
        basic_linked_kdtree_t<input_element_type, 3u, decltype(coordinate_map)> kdtree{
            indices.begin(),
            indices.end(),
            coordinate_map,
            kdtree_params};

        std::transform(
            policy,
            indices.begin(),
            indices.end(),
            temporary_points.begin(),
            [&](std::size_t const i) {
                return bilateral::detail::compute_pi(
                    i,
                    sigmaf,
                    sigmag,
                    kdtree,
                    coordinate_map,
                    internal_point_map,
                    internal_normal_map,
                    gaussian,
                    gaussian,
                    projection);
            });

        std::copy(temporary_points.begin(), temporary_points.end(), points.begin());
    }

    return std::copy(points.begin(), points.end(), out_begin);
}

/**
 * @ingroup smoothing-algorithm
 * @brief
 * Deforms the normal field of an input point cloud using the local deformation field of the
 * 3d bilateral filter (in other words, its Jacobian at a point p).
 *
 * The bilateral filter is defined in the same way as in
 * 'Jones, Thouis R., Fredo Durand, and Matthias Zwicker. "Normal improvement for point rendering."
 * IEEE Computer Graphics and Applications 24.4 (2004): 53-56.'
 *
 * This method should be used only for point rendering. It does not smooth input normals as one
 * would expect for surface reconstruction. For normal smoothing, one should rather look at
 * techniques such as EAR (edge aware resampling):
 *
 * 'Huang, Hui, et al. "Edge-aware point set resampling." ACM transactions on graphics (TOG) 32.1
 * (2013): 1-12.'
 *
 * @tparam ExecutionPolicy The execution policy type
 * @tparam RandomAccessIter Iterator type satisfying Random Access requirements
 * @tparam OutputIter Iterator type dereferenceable to a type satisfying Normal concept
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam NormalMap Type satisfying NormalMap concept
 * @param begin Start iterator of input point cloud
 * @param end End iterator of input point cloud
 * @param out_begin Start iterator of output points
 * @param point_map The point map property map
 * @param normal_map The normal map property map
 * @param params The bilateral filter algorithm's parameters
 * @return End iterator of output sequence
 */
template <
    class ExecutionPolicy,
    class RandomAccessIter,
    class OutputIter,
    class PointMap,
    class NormalMap>
OutputIter bilateral_filter_normals(
    ExecutionPolicy const& policy,
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    bilateral::params_t const& params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using input_normal_type  = std::invoke_result_t<NormalMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_normal_type = typename xstd::output_iterator_traits<OutputIter>::value_type;
    using difference_type    = typename std::iterator_traits<RandomAccessIter>::difference_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    std::size_t const N        = static_cast<std::size_t>(std::distance(begin, end));
    scalar_type const sigmaf   = static_cast<scalar_type>(params.sigmaf);
    scalar_type const sigmag   = static_cast<scalar_type>(params.sigmag);
    std::size_t const K        = params.K;
    scalar_type constexpr pi   = static_cast<scalar_type>(3.14159265358979323846);
    scalar_type constexpr zero = scalar_type{0.};

    assert(K > 0u);
    assert(N > 0u);
    assert(sigmaf > zero);
    assert(sigmag > zero);

    std::vector<std::size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0u);

    std::vector<input_normal_type> normals(N);
    std::transform(begin, end, normals.begin(), [&](input_element_type const& e) {
        return normal_map(e);
    });

    std::vector<input_normal_type> temporary_normals(N);

    auto const internal_point_map = [&](std::size_t const i) -> input_point_type {
        return point_map(*std::next(begin, static_cast<difference_type>(i)));
    };
    auto const internal_normal_map = [&](std::size_t const i) -> input_normal_type {
        return normals[i];
    };

    auto const gaussian = [=](scalar_type const sigma, scalar_type const r) {
        scalar_type const s2      = sigma * sigma;
        scalar_type const r2      = r * r;
        scalar_type const power   = -r2 / (2 * s2);
        scalar_type constexpr one = scalar_type{1.};
        scalar_type constexpr two = scalar_type{2.};
        scalar_type const coeff   = one / (sigma * std::sqrt(two * pi));
        return coeff * std::exp(power);
    };

    auto const dgaussian = [=](scalar_type const sigma, scalar_type const r) {
        scalar_type const s2      = sigma * sigma;
        scalar_type const s3      = sigma * s2;
        scalar_type const r2      = r * r;
        scalar_type const power   = -r2 / (2 * s2);
        scalar_type constexpr two = scalar_type{2.};
        scalar_type const coeff   = -r / (s3 * std::sqrt(two * pi));
        return coeff * std::exp(power);
    };

    auto const projection = [&](input_point_type const& p,
                                input_normal_type const& np,
                                input_point_type const& s) -> input_point_type {
        auto const sp = p - s;
        common::basic_vector3d_t<scalar_type> const n{np.nx(), np.ny(), np.nz()};
        auto const d = common::inner_product(sp, n);
        return s + d * n;
    };

    using coordinates_type = std::array<scalar_type, 3u>;

    auto const coordinate_map = [&](std::size_t const i) {
        auto const p = internal_point_map(i);
        return coordinates_type{p.x(), p.y(), p.z()};
    };

    kdtree::construction_params_t kdtree_params;
    kdtree_params.compute_max_depth                   = true;
    kdtree_params.construction                        = kdtree::construction_t::nth_element;
    kdtree_params.max_elements_per_leaf               = 64u;
    kdtree_params.min_element_count_for_parallel_exec = indices.size();

    basic_linked_kdtree_t<input_element_type, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        kdtree_params};

    for (std::size_t k = 0u; k < K; ++k)
    {
        std::transform(
            policy,
            indices.begin(),
            indices.end(),
            temporary_normals.begin(),
            [&](std::size_t const i) {
                return bilateral::detail::compute_ni(
                    i,
                    sigmaf,
                    sigmag,
                    kdtree,
                    coordinate_map,
                    internal_point_map,
                    internal_normal_map,
                    gaussian,
                    dgaussian,
                    gaussian,
                    dgaussian,
                    projection);
            });

        std::copy(temporary_normals.begin(), temporary_normals.end(), normals.begin());
    }

    return std::copy(normals.begin(), normals.end(), out_begin);
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_BILATERAL_FILTER_HPP
