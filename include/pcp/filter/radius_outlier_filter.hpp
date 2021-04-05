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
 * @brief Iterates through the entire input once, calculates the density of the points for every
 * point and removes points that do not pass the density threshold passed in the parameters.
 *
 * @tparam Element Element Type
 * @tparam ParametersType type of the parameters (double,float)
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam RangeSearchMap Type satisfying Range concept
 */

template <class Element, class ParametersType, class PointMap, class RangeSearchMap>
class basic_radius_outlier_filter_t
{
  public:
    using element_type      = Element;
    using parameters_type   = ParametersType;
    using parameters_object = radius_outlier_filter::construction_params_t<parameters_type>;
    template <class ForwardIterator>
    basic_radius_outlier_filter_t(
        ForwardIterator begin,
        ForwardIterator end,
        PointMap point_map,
        RangeSearchMap range_search_map,
        parameters_object params)
        : point_map_(point_map), range_search_map_(range_search_map), params_(params)
    {
    }
    bool operator()(element_type const& p)
    {
        auto points_in_ball = range_search_map_(point_map_(p), params_.radius_);
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
    PointMap point_map_;
    RangeSearchMap range_search_map_;
    parameters_object params_;
};

} // namespace pcp

#endif // PCP_FILTER_RADIUS_OUTLIER_FILTER_HPP
