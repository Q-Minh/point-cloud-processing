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

template <class ForwardIterator, class ScalarType, class PointMap, class CoordinateMap>
ScalarType error(
    ForwardIterator begin_A,
    ForwardIterator end_A,
    ForwardIterator begin_B,
    ForwardIterator end_B,
    PointMap point_map,
    CoordinateMap coordinate_map)
{
    using scalar_type  = ScalarType;
    using element_type = typename std::iterator_traits<ForwardIterator>::value_type;
    using point_type   = std::invoke_result_t<PointMap, element_type>;

    auto const length_a = end_A - begin_A;
    auto const length_b = end_B - begin_A;
    assert(length_a == length_b);
    scalar_type somme = 0;
    //#pragma omp parallel reduction(+ : somme)
    for (; begin_A != end_A; begin_A++, begin_B++)
    {
        auto const pt_a             = point_map(begin_A);
        auto const pt_b             = point_map(begin_B);
        auto const pt_a_coordinates = coordinate_map(pt_a);
        auto const pt_b_coordinates = coordinate_map(pt_b);

        assert(pt_a_coordinates.size() == pt_b_coordinates.size());
        auto const diff_x = (pt_a_coordinates[0]) - (pt_b_coordinates[0]);
        auto const diff_y = (pt_a_coordinates[1]) - (pt_b_coordinates[1]);
        auto const diff_z = (pt_a_coordinates[2]) - (pt_b_coordinates[2]);
        somme += sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    }
    return somme;
}

template <class ElementType, class PointMap, class KnnMap>
ElementType nearestNeighbor(KnnMap knn_map, ElementType point)
{
    return knn_map(point);
}

template <class ForwardIterator, class ScalarType, class PointMap, class CoordinateMap>
Eigen::Matrix4d best_fit_Transform(
    ForwardIterator begin_A,
    ForwardIterator end_A,
    ForwardIterator begin_B,
    ForwardIterator end_B,
    PointMap point_map,
    CoordinateMap coordinate_map)
{
    using element_type = typename std::iterator_traits<ForwardIterator>::value_type;
    using point_type   = pcp::point_t; // std::invoke_result_t<PointMap, element_type>;

    point_type center_a, center_b;
    int a_size = std::distance(begin_A, end_A);
    int b_size = std::distance(begin_B, begin_B);
    assert(a_size == b_size);
    for (auto a = begin_A, auto b = begin_B; a != end_A; a++, b++)
    {
        center_a += point_map(a);
        center_b += point_map(b);
    }
    center_a /= a_size;
    center_b /= b_size;

    std::vector<point_type> aa(a_size), bb(a_size);
    for (int i = 0, auto a = begin_A, auto b = begin_B; a != end_A; a++, b++, i++)
    {
        aa[i] = point_map(a) - center_a;
        bb[i] = point_map(b) - center_b;
    }
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int i = 0, i < a_size; i++)
    {
        auto const a = coordinate_map(aa[i]);
        auto const b = coordinate_map(bb[i]);
        covariance +=
            Eigen::Vector3d(a[0], a[1], a[2]) * Eigen::Vector3d(b[0], b[1], b[2]).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    Eigen::Matrix3d R_ = U * (V.transpose());

    auto const coordinates_center_a = coordinate_map(center_a);
    auto const coordinates_center_b = coordinate_map(center_b);
    Eigen::Vector3d t_ =
        Eigen::Vector3d(coordinates_center_a[0], coordinates_center_a[1], coordinates_center_a[2]) -
        R_ * Eigen::Vector3d(
                 coordinates_center_b[0],
                 coordinates_center_b[1],
                 coordinates_center_b[2]);

    Eigen::Matrix4d transformation_matrix = Eigen::MatrixXd::Identity(4, 4);
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
