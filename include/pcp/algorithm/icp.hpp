#ifndef PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
#define PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "Eigen/Eigen"
#include "pcp/common/vector3d_queries.hpp"
#include "pcp/traits/point_map.hpp"

#include <algorithm>

namespace pcp {
namespace algorithm {

/**
 * @brief
 * Calculates the RMS(root mean square) error between two matrices.
 * @tparam ScalarType
 * @param A Matrix A
 * @param B Matrix B
 * @return RMS error between A and B.
 */
template <class ScalarType>
ScalarType icp_error(Eigen::MatrixXf const& A, Eigen::MatrixXf const& B)
{
    const auto a_size = A.rows();
    const auto b_size = B.rows();
    assert(a_size == b_size);
    ScalarType err = 0;
    //#pragma omp parallel reduction(+ : somme)
    for (int i = 0; i < a_size; i++)
    {
        auto const diff_x = (A(i, 0)) - (B(i, 0));
        auto const diff_y = (A(i, 1)) - (B(i, 1));
        auto const diff_z = (A(i, 2)) - (B(i, 2));
        err += sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    }
    return err;
}

/**
 * @brief
 * Returns the nearest neighbor of a point given a knn map
 * @tparam ElementType
 * @tparam KnnMap Type satisfying KnnViewMap concept
 * @param knn_map The knn map property map
 * @param point point which we want to find its nearest neighbor
 * @return the nearest neighbor of a point
 */
template <class ElementType, class KnnMap>
ElementType icp_nearest_neighbor(KnnMap knn_map, ElementType point)
{
    return knn_map(point);
}

/**
 * @brief
 * Computes the best rotation matrix (3x3)
 * and translation vector (3x1) to align Matrix B on A, returns both
 * the rotation matrix and the translation vector.
 * @tparam PointMap1 PointMap Type satisfying PointMap concept for matrix A
 * @tparam ForwardIterator1
 * @tparam ForwardIterator2
 * @tparam PointMap2 PointMap Type satisfying PointMap concept for matrix B
 * @param begin_a Begin iterator to the elements of matrix A
 * @param end_a End iterator to the elements of matrix A
 * @param begin_b end_b Begin iterator to the elements of matrix B
 * @param end_b End iterator to the elements of matrix B
 * @param point_map_a The point_map property map for matrix A
 * @param point_map_b The point_map property map for matrix B
 * @return pair of rotation matrix(3x3) and translation vector(3x1).
 */
template <class ForwardIterator1, class ForwardIterator2, class PointMap1, class PointMap2>
std::pair<
    Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap1,
            typename std::iterator_traits<ForwardIterator1>::value_type>::coordinate_type,
        3,
        3>,
    Eigen::Matrix<
        typename std::invoke_result_t<
            PointMap1,
            typename std::iterator_traits<ForwardIterator1>::value_type>::coordinate_type,
        3,
        1>>
icp_best_fit_transform(
    ForwardIterator1 begin_a,
    ForwardIterator1 end_a,
    ForwardIterator2 begin_b,
    ForwardIterator2 end_b,
    PointMap1 const& point_map_a,
    PointMap2 const& point_map_b)
{
    using element_type_a = typename std::iterator_traits<ForwardIterator1>::value_type;
    using element_type_b = typename std::iterator_traits<ForwardIterator2>::value_type;

    using point_type_a = std::invoke_result_t<PointMap1, element_type_a>;
    using point_type_b = std::invoke_result_t<PointMap2, element_type_b>;

    using scalar_type = typename point_type_a::coordinate_type;

    using matrix_3_type = Eigen::Matrix<scalar_type, 3, 3>;
    using vector_3_type = Eigen::Matrix<scalar_type, 3, 1>;

    auto const a_size = static_cast<std::size_t>(std::distance(begin_a, end_a));
    auto const b_size = static_cast<std::size_t>(std::distance(begin_b, end_b));
    assert(a_size == b_size);

    auto const center_a = common::center_of_geometry(begin_a, end_a, point_map_a);
    auto const center_b = common::center_of_geometry(begin_b, end_b, point_map_b);

    std::array<scalar_type, 6u> cov{0.};

    for (size_t i = 0u; i < a_size; ++i)
    {
        auto it_a = std::next(begin_a, i);
        auto it_b = std::next(begin_a, i);

        auto const a = point_map_a(*it_a);
        auto const b = point_map_b(*it_b);

        scalar_type ax, ay, az, bx, by, bz;
        ax = a.x() - center_a.x();
        ay = a.y() - center_a.y();
        az = a.z() - center_a.z();
        bx = b.x() - center_b.x();
        by = b.y() - center_b.y();
        bz = b.z() - center_b.z();

        cov[0] += ax * bx;
        cov[1] += ay * by;
        cov[2] += az * bz;
        cov[3] += ax * by;
        cov[4] += ax * bz;
        cov[5] += ay * bz;
    }

    matrix_3_type Covariance;
    Covariance(0, 0) = cov[0];
    Covariance(1, 1) = cov[1];
    Covariance(2, 2) = cov[2];
    Covariance(0, 1) = cov[3];
    Covariance(0, 2) = cov[4];
    Covariance(1, 2) = cov[5];
    Covariance(1, 0) = Covariance(0, 1);
    Covariance(2, 0) = Covariance(0, 2);
    Covariance(2, 1) = Covariance(1, 2);

    Eigen::JacobiSVD<matrix_3_type> SVD(Covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    matrix_3_type U = SVD.matrixU();
    matrix_3_type V = SVD.matrixV();

    matrix_3_type R = V * (U.transpose());

    // if it's a reflection
    if (R.determinant() == -1)
    {
        // inverse the last column
        V(0, 2) = -1 * V(0, 2);
        V(1, 2) = -1 * V(1, 2);
        V(2, 2) = -1 * V(2, 2);
        R       = V * U.transpose();
    }

    vector_3_type t = vector_3_type(center_a.x(), center_a.y(), center_a.z()) -
                      R * vector_3_type(center_b.x(), center_b.y(), center_b.z());

    return {R, t};
}
} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
