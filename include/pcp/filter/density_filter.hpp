#ifndef PCP_FILTER_DENSITY_FILTER_HPP
#define PCP_FILTER_DENSITY_FILTER_HPP

/**
 * @file
 * @ingroup filter
 */

#include <algorithm>
#include <iterator>
#include <pcp/algorithm/average_distance_to_neighbors.hpp>
#include <vector>

namespace pcp {
namespace density_filter {

/**
 * @brief parameters for the filter, density threshold is
 * @tparam ScalarType Scalar type for the parameter variables (float,double,etc)
 */
template <class ScalarType>
struct construction_params_t
{
    using scalar_type              = ScalarType;
    scalar_type density_threshold_ = static_cast<float>(1.0f);
    scalar_type radius_multiplier_ = static_cast<float>(1.0f);
};
} // namespace density_filter

/**
 * @brief Iterates through the entire input once, calculates the density of the points for every
 * point and removes points that do not pass the density threshold passed in the parameters.
 * 
 * @tparam Element Element Type
 * @tparam ParametersType type of the parameters (double,float)
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam KnnMap Type satisfying KnnMap concept
 * @tparam RangeSearchMap Type satisfying Range concept
 */
template <class Element, class ParametersType, class PointMap, class KnnMap, class RangeSearchMap>
class basic_density_filter_t
{
  public:
    using self_type =
        basic_density_filter_t<Element, ParametersType, PointMap, KnnMap, RangeSearchMap>;
    using self_type_ptr     = std::unique_ptr<self_type>;
    using element_type      = Element;
    using parameters_type   = ParametersType;
    using parameters_object = density_filter::construction_params_t<parameters_type>;

    template <class ForwardIterator>
    basic_density_filter_t(
        ForwardIterator begin,
        ForwardIterator end,
        PointMap point_map,
        KnnMap knn_map,
        RangeSearchMap range_search_map,
        parameters_object params)
        : point_map_(point_map),
          knn_map_(knn_map),
          range_search_map_(range_search_map),
          params_(params)

    {
        radius_ = 0.0f;

        std::vector<float> mean_distances =
            pcp::algorithm::average_distances_to_neighbors(begin, end, point_map, knn_map);

        radius_ =
            std::reduce(std::execution::par, mean_distances.cbegin(), mean_distances.cend(), 0.f) /
            static_cast<float>(mean_distances.size());
    }

    /**
     * @brief sets density treshold
     * @param density_treshold
     */
    void density_threshold(parameters_type density_treshold)
    {
        params_.density_threshold_ = density_treshold;
    }
    parameters_type density_treshold() const { return params_.density_treshold_; }
    /**
     * @brief sets radius multiplier
     * @param radius_multiplier
     */
    void radius_multiplier(parameters_type radius_multiplier)
    {
        params_.radius_multiplier_ = radius_multiplier;
    }
    /**
     * @brief
     * @return radius multiplier
     */
    parameters_type radius_multiplier() { return params_.radius_multiplier_; }

    bool operator()(element_type const& p)
    {
        auto points_in_ball = range_search_map_(p, radius_ * params_.radius_multiplier_);
        auto density        = points_in_ball.size();
        return density < params_.density_threshold_;
    }

  private:
    PointMap point_map_;
    KnnMap knn_map_;
    RangeSearchMap range_search_map_;
    float radius_;
    parameters_object params_;
};

} // namespace pcp

#endif // PCP_FILTER_DENSITY_FILTER_HPP
