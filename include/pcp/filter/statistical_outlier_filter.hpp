#ifndef PCP_FILTER_STATISTICAL_OUTLIER_FILTER_HPP
#define PCP_FILTER_STATISTICAL_OUTLIER_FILTER_HPP
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

namespace statistical_outlier_filter {
/**
 * @brief parameters for the filter
 * @tparam ScalarType Scalar type for the parameter variables (float,double,etc)
 */
template <class ScalarType>
struct construction_params_t
{
    using scalar_type                         = ScalarType;
    scalar_type std_dev_multiplier_threshold_ = static_cast<float>(1);
    std::size_t mean_k_                       = 1;
};
} // namespace statistical_outlier_filter

/**
 * @brief The algorithm iterates through the entire input twice:
 * During the first iteration it will compute the average distance that each point has to its
 * nearest k neighbors. The value of k can be set using mean_k_ Next, the mean and standard
 * deviation of all these distances are computed in order to determine a distance threshold. The
 * distance threshold will be equal to: mean + stddev_mult * stddev. ' The multiplier for the
 * standard deviation can be set using the std_dev_multiplier_treshold in the params.
 *
 * @tparam Element Element type
 * @tparam ParametersType type of the parameters (double,float)
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam KnnMap Type satisfying KnnMap concept
 */
template <class Element, class ParametersType, class PointMap, class KnnMap>

class basic_statistical_outlier_filter_t
{
  public:
    using element_type      = Element;
    using parameters_type   = ParametersType;
    using parameters_object = statistical_outlier_filter::construction_params_t<parameters_type>;
    using point_type        = std::invoke_result_t<PointMap, element_type>;
    template <class ForwardIterator>
    basic_statistical_outlier_filter_t(
        ForwardIterator begin,
        ForwardIterator end,
        PointMap point_map,
        KnnMap knn_map,
        parameters_object params)
        : point_map_(point_map), knn_map_(knn_map), params_(params)
    {
        // size is the number of points in the point cloud
        auto const& size = std::distance(begin, end);
        std::vector<element_type> nn_index(params_.mean_k_);
        std::vector<float> nn_dists(params_.mean_k_);
        std::transform(
            begin,
            end,
            std::inserter(mean_distances_, mean_distances_.end()),
            [&](element_type& p) {
                point_type const pi    = point_map_(p);
                auto const& neighbours = knn_map_(p);
                auto const sum         = std::accumulate(
                    neighbours.cbegin(),
                    neighbours.cend(),
                    0.f,
                    [&](float val, element_type const& neighbour) {
                        point_type pj         = point_map_(neighbour);
                        float const& distance = pcp::common::norm(pi - pj);
                        return val + distance;
                    });

                float const& mean = sum / static_cast<float>(neighbours.size());
                return std::make_pair(&p, mean);
            });
        float sum = 0.0f, sq_sum = 0.0f;
        for (auto const& i : mean_distances_)
        {
            sum += i.second;
            sq_sum += i.second * i.second;
        }

        mean_ = sum / static_cast<float>(size);

        float variance =
            (sq_sum - sum * sum / static_cast<float>(size)) / (static_cast<float>(size) - 1);
        float std_dev = sqrt(variance);

        distance_threshold_ = mean_ + params_.std_dev_multiplier_threshold_ * std_dev;
    }

  public:
    /**
     * @brief sets standard deviation multiplier threshold
     * @param std_dev_multiplier_threshold
     */
    void std_dev_multiplier_threshold(parameters_type std_dev_multiplier_threshold)
    {
        params_.std_dev_multiplier_threshold_ = std_dev_multiplier_threshold;
    }
    /**
     * @brief
     * @return standard deviation multiplier threshold
     */
    parameters_type std_dev_multiplier_threshold() const
    {
        return params_.std_dev_multiplier_threshold_;
    }
    /**
     * @brief sets the mean k for the nearest neighbours search
     * @param mean_k
     */
    void mean_k_(std::size_t mean_k) { params_.mean_k_ = mean_k; }
    /**
     * @brief
     * @return  the mean k for the nearest neighbours search
     */
    std::size_t mean_k_() { return params_.mean_k_; }

    bool operator()(element_type& p) { return (mean_distances_[&p] > distance_threshold_); }

  private:
    PointMap point_map_;
    KnnMap knn_map_;
    parameters_object params_;
    float mean_;
    float distance_threshold_ = 0;
    std::unordered_map<element_type*, float> mean_distances_;
};

} // namespace pcp

#endif // PCP_FILTER_PASS_THROUGH_FILTER_HPP
