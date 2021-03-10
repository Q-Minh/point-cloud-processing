#ifndef PCP_ALGORITHM_COVARIANCE_HPP
#define PCP_ALGORITHM_COVARIANCE_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/vector3d_queries.hpp"
#include "pcp/traits/point_map.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <utility>

namespace pcp {
namespace algorithm {

/**
 * @ingroup algorithm
 * @brief Computes the covariance matrix of a point set
 * @tparam ForwardIter Iterator type of input point set
 * @tparam PointMap Type satisfying PointMap concept
 * @param begin Start iterator to input point set
 * @param end End iterator to input point set
 * @param point_map The point map property map
 * @return A pair of the mean and covariance matrix [mu, Cov]
 */
template <class ForwardIter, class PointMap>
std::pair<
    Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        1>,
    Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        3>>
covariance(ForwardIter begin, ForwardIter end, PointMap const& point_map)
{
    using element_type = typename std::iterator_traits<ForwardIter>::value_type;

    static_assert(
        traits::is_point_map_v<PointMap, element_type>,
        "point_map must satisfy PointMap concept");

    using point_type  = std::invoke_result_t<PointMap, element_type>;
    using scalar_type = typename point_type::coordinate_type;
    using vector_type = Eigen::Matrix<scalar_type, 3, 1>;
    using matrix_type = Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        3>;

    /**
     * element 0 : x*x
     * element 1 : y*y
     * element 2 : z*z
     * element 3 : x*y
     * element 4 : x*z
     * element 5 : y*z
     */
    std::array<scalar_type, 6u> cov{0.};

    point_type const mu = common::center_of_geometry(begin, end, point_map);
    std::for_each(begin, end, [&](element_type const& e) {
        auto const& p = point_map(e);
        auto const pp = p - mu;

        auto const xx = pp.x() * pp.x();
        auto const yy = pp.y() * pp.y();
        auto const zz = pp.z() * pp.z();
        auto const xy = pp.x() * pp.y();
        auto const xz = pp.x() * pp.z();
        auto const yz = pp.y() * pp.z();

        cov[0] += xx;
        cov[1] += yy;
        cov[2] += zz;
        cov[3] += xy;
        cov[4] += xz;
        cov[5] += yz;
    });

    matrix_type Cov;
    Cov(0, 0) = cov[0];
    Cov(1, 1) = cov[1];
    Cov(2, 2) = cov[2];

    Cov(0, 1) = cov[3];
    Cov(0, 2) = cov[4];
    Cov(1, 2) = cov[5];
    Cov(1, 0) = Cov(0, 1);
    Cov(2, 0) = Cov(0, 2);
    Cov(2, 1) = Cov(1, 2);

    return {vector_type{mu.x(), mu.y(), mu.z()}, Cov};
}

/**
 * @ingroup algorithm
 * @brief Sorts eigenvalues and eigenvectors in increasing order
 * @tparam ScalarType Coefficient type
 * @param lambda Vector of eigen values
 * @param v Matrix of eigen vectors
 * @return a pair = (sorted eigen values, sorted eigen vectors)
 */
template <class ScalarType>
std::pair<Eigen::Matrix<ScalarType, 3, 1>, Eigen::Matrix<ScalarType, 3, 3>> eigen_sorted(
    Eigen::Matrix<ScalarType, 3, 1> const& lambda,
    Eigen::Matrix<ScalarType, 3, 3> const& v)
{
    using vector_type = Eigen::Matrix<ScalarType, 3, 1>;
    using matrix_type = Eigen::Matrix<ScalarType, 3, 3>;

    std::array<int, 3u> indices{0, 1, 2};
    std::sort(indices.begin(), indices.end(), [&](int const i, int const j) {
        return lambda(i) < lambda(j);
    });

    vector_type const eigen_values{lambda(indices[0]), lambda(indices[1]), lambda(indices[2])};
    matrix_type eigen_vectors(3, 3);
    eigen_vectors.col(0) = v.col(indices[0]);
    eigen_vectors.col(1) = v.col(indices[1]);
    eigen_vectors.col(2) = v.col(indices[2]);

    return {eigen_values, eigen_vectors};
}

/**
 * @ingroup algorithm
 * @brief Sorts eigenvalues and eigenvectors in increasing order
 * @tparam ScalarType Coefficient type
 * @param Cov The covariance matrix
 * @return a pair = (sorted eigen values, sorted eigen vectors)
 */
template <class ScalarType>
std::pair<Eigen::Matrix<ScalarType, 3, 1>, Eigen::Matrix<ScalarType, 3, 3>>
eigen_sorted(Eigen::Matrix<ScalarType, 3, 3> const& Cov)
{
    using vector_type = Eigen::Matrix<ScalarType, 3, 1>;
    using matrix_type = Eigen::Matrix<ScalarType, 3, 3>;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ScalarType, 3, 3>> A(Cov);
    vector_type const& lambda = A.eigenvalues();
    matrix_type const& v      = A.eigenvectors();

    return eigen_sorted(lambda, v);
}

/**
 * @ingroup algorithm
 * @brief Returns the sorted eigen values and eigen vectors of the covariance matrix of a point set
 * @tparam ForwardIter Iterator type of input point set
 * @tparam PointMap Type satisfying PointMap concept
 * @param begin Start iterator to the input point set
 * @param end End iterator to the input point set
 * @param point_map The point map property map
 * @return A pair = (sorted eigen values, sorted eigen vectors)
 */
template <class ForwardIter, class PointMap>
std::pair<
    Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        1>,
    Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        3>>
pca(ForwardIter begin, ForwardIter end, PointMap const& point_map)
{
    using matrix_type = Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        3>;

    using vector_type = Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap,
            typename std::iterator_traits<ForwardIter>::value_type>::coordinate_type,
        3,
        1>;

    auto const [mu, sigma] = covariance(begin, end, point_map);
    matrix_type const& Cov = sigma;

    return eigen_sorted(Cov);
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_COVARIANCE_HPP
