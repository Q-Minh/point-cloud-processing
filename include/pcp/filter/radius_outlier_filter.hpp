#ifndef PCP_FILTER_RADIUS_OUTLIER_FILTER_HPP
#define PCP_FILTER_RADIUS_OUTLIER_FILTER_HPP

/**
 * @file
 * @ingroup filter
 */

#include <algorithm>
#include <iterator>
#include <map>
#include <pcp/kdtree/linked_kdtree.hpp>
#include <vector>
namespace pcp {

namespace radius_outlier_filter {
/**
 * @brief parameters for the filter
 * @tparam ScalarType Scalar type for the parameter variables (float,double,etc)
 */
template <class ScalarType>
struct construction_params_t
{
    using scalar_type                    = ScalarType;
    scalar_type radius_                  = static_cast<float>(1.0);
    std::size_t min_neighbors_in_radius_ = 1u;
};
} // namespace radius_outlier_filter

/**
 * @brief Iterates through the entire input once, and for each point, retrieves the number of
 * neighbors within a certain radius. The point will be considered an outlier if it has too few
 * neighbors, as determined by min_neighbors_in_radius(). The radius can be changed with radius()
 *
 * @tparam Element Element type
 * @tparam ParametersType type of the parameters (double,float)
 * @tparam CoordinateMap coordinate map for a point
 */
template <class Element, class ParametersType, class CoordinateMap>
class basic_radius_outlier_filter_t
{
  public:
    using element_type      = Element;
    using kdtree_type       = pcp::basic_linked_kdtree_t<element_type, 3u, CoordinateMap>;
    using parameters_type   = ParametersType;
    using parameters_object = radius_outlier_filter::construction_params_t<parameters_type>;
    using coordinate_type   = traits::coordinate_type<CoordinateMap, Element>;
    template <class ForwardIterator>
    basic_radius_outlier_filter_t(
        ForwardIterator begin,
        ForwardIterator end,
        CoordinateMap coordinate_map,
        kdtree_type& kdtree,
        parameters_object params)
        : coordinate_map_(coordinate_map), kdtree_(kdtree), params_(params)
    {
    }
    bool operator()(element_type const& p)
    {
        pcp::sphere_a<coordinate_type> ball;
        ball.position[0] = coordinate_map_(p)[0];
        ball.position[1] = coordinate_map_(p)[1];
        ball.position[2] = coordinate_map_(p)[2];
        ball.radius      = params_.radius_;

        auto points_in_ball = kdtree_.range_search(ball);
        auto density        = points_in_ball.size();
        return density < params_.min_neighbors_in_radius_ + 1;
    }

  public:
    /**
     * @brief
     * @return radius of the range search
     */
    parameters_type radius() const { return params_.radius_; }
    /**
     * @brief
     * @return returns minimum amount of neighbors in radius required for a point to be considered
     * inlier
     */
    size_t min_meighbors_in_radius() const { return params_.min_neighbors_in_radius_; }
    /**
     * @brief sets radius of the range search
     * @param radius
     */
    void radius(parameters_type radius) { params_.radius_ = radius; }
    /**
     * @brief sets minimum amount of neighbors in radius required for a point to be considered
     * inlier
     * @param min
     */
    void min_neighbors_in_radius(std::size_t min) { params_.min_neighbors_in_radius_ = min; }

  private:
    kdtree_type& kdtree_;
    CoordinateMap coordinate_map_;
    parameters_object params_;
};

} // namespace pcp

#endif // PCP_FILTER_RADIUS_OUTLIER_FILTER_HPP
