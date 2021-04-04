#ifndef PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
#define PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "Eigen/Eigen"
#include "pcp/common/norm.hpp"
#include "pcp/traits/point_map.hpp"

#include <algorithm>
#include <execution>

namespace pcp {
namespace algorithm {

template <class ScalarType>
ScalarType error(Eigen::MatrixXf const& A, Eigen::MatrixXf const& B)
{
    const auto a_size = A.rows();
    const auto b_size = B.rows();
    assert(a_size == b_size);
    scalar_type somme = 0;
    //#pragma omp parallel reduction(+ : somme)
    for (int i = 0; i < a_size; i++)
    {
        auto const diff_x = (A(i, 0)) - (B(i, 0));
        auto const diff_y = (A(i, 1)) - (B(i, 1));
        auto const diff_z = (A(i, 2)) - (B(i, 2));
        somme += sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    }
    return somme;
}

template <class ElementType, class KnnMap>
ElementType nearestNeighbor(KnnMap knn_map, ElementType point)
{
    return knn_map(point);
}

template <class ScalarType>
Eigen::Matrix4f best_fit_Transform(Eigen::MatrixXf const& A, Eigen::MatrixXf const& B)
{
    Eigen::Vector3f center_a, center_b;
    const auto a_size = A.rows();
    const auto b_size = B.rows();
    assert(a_size == b_size);

    for (int i = 0; i < a_size; i++)
    {
        center_a(0) += A(i, 0);
        center_a(1) += A(i, 1);
        center_a(2) += A(i, 2);
        center_b(0) += B(i, 0);
        center_b(1) += B(i, 1);
        center_b(2) += B(i, 2);
    }
    center_a /= a_size;
    center_b /= b_size;

    Eigen::MatrixXf aa, bb;
    for (int i = 0; i < a_size; i++)
    {
        aa(i, 0) = a(i, 0) - center_a(0);
        aa(i, 1) = a(i, 1) - center_a(1);
        aa(i, 2) = a(i, 2) - center_a(2);

        bb(i, 0) = b(i, 0) - center_b(0);
        bb(i, 1) = b(i, 1) - center_b(1);
        bb(i, 2) = b(i, 2) - center_b(2);
    }
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int i = 0, i < a_size; i++)
    {
        covariance +=
            Eigen::Vector3f(aa(i,0), aa(i,1), aa(i,2) * Eigen::Vector3f(bb(i,0), bb(i,1), bb(i,2)).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    Eigen::Matrix3f R_ = U * (V.transpose());

    Eigen::Vector3f t_ = Eigen::Vector3f(center_a(0), center_a(1), center_a(2)) -
                         R_ * Eigen::Vector3f(center_b(0), center_b(1), center_b(2));

    Eigen::Matrix4f transformation_matrix = Eigen::MatrixXf::Identity(4, 4);
    // convert to cv::Mat
    transformation_matrix.block<3, 3>(0, 0) =
        (Mat_<double>(3, 3) << R_(0, 0),
         R_(0, 1),
         R_(0, 2),
         R_(1, 0),
         R_(1, 1),
         R_(1, 2),
         R_(2, 0),
         R_(2, 1),
         R_(2, 2));

    transformation_matrix.block<3, 1>(0, 3) = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));

    return transformation_matrix;
}
} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_AVERAGE_DISTANCE_TO_NEIGHBORS_HPP
