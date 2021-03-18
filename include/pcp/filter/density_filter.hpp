#ifndef PCP_FILTER_DENSITY_FILTER_HPP
#define PCP_FILTER_DENSITY_FILTER_HPP

/**
 * @file
 * @ingroup filter
 */

#include <algorithm>
#include <pcp/common/sphere.hpp>
#include <pcp/kdtree/linked_kdtree.hpp>

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
    std::size_t k_                 = 3u;
};
} // namespace density_filter
/**
 * @brief  Iterates through the entire input once, calculates the density of the points for every
 * point and removes points that do not pass the density threshold passed in the parameters.
 *
 * @tparam Element Element type
 * @tparam ParametersType type of the parameters (double,float)
 * @tparam CoordinateMap coordinate map for a point
 */
template <class Element, class ParametersType, class CoordinateMap>
class basic_density_filter_t
{
  public:
    using self_type         = basic_density_filter_t<Element, ParametersType, CoordinateMap>;
    using self_type_ptr     = std::unique_ptr<self_type>;
    using coordinate_type   = traits::coordinate_type<CoordinateMap, Element>;
    using element_type      = Element;
    using kdtree_type       = pcp::basic_linked_kdtree_t<element_type, 3u, CoordinateMap>;
    using parameters_type   = ParametersType;
    using parameters_object = density_filter::construction_params_t<parameters_type>;

    template <class ForwardIterator>
    basic_density_filter_t(
        ForwardIterator begin,
        ForwardIterator end,
        CoordinateMap coordinate_map,
        kdtree_type& kdtree,
        parameters_object params)
        : coordinate_map_{coordinate_map}, kdtree_(kdtree), params_(params)

    {
        radius_ = 0.0f;
        std::vector<float> mean_distances(std::distance(begin, end), 0.f);

        std::transform(
            std::execution::par,
            begin,
            end,
            mean_distances.begin(),
            [&](element_type const& p) {
                auto const& neighbours = kdtree_.nearest_neighbours(p, params_.k_);
                auto const sum         = std::accumulate(
                    neighbours.cbegin(),
                    neighbours.cend(),
                    0.f,
                    [&p](float val, element_type const& neighbour) {
                        auto const distance = pcp::common::norm(element_type(neighbour) - p);
                        return val + distance;
                    });

                return sum / static_cast<float>(neighbours.size());
            });

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
    /**
     * @brief sets k for the nearest neighbours search
     * @param
     */
    void k(std::size_t) { params_.k = k; }
    /**
     * @brief
     * @return k for the nearest neighbours search
     */
    std::size_t k() { return params_.k_; }

    bool operator()(element_type const& p)
    {
        pcp::sphere_a<coordinate_type> ball;
        ball.position[0] = coordinate_map_(p)[0];
        ball.position[1] = coordinate_map_(p)[1];
        ball.position[2] = coordinate_map_(p)[2];
        ball.radius      = radius_ * params_.radius_multiplier_;

        auto points_in_ball = kdtree_.range_search(ball);
        auto density        = points_in_ball.size();
        return density < params_.density_threshold_;
    }

  private:
    kdtree_type& kdtree_;
    CoordinateMap coordinate_map_;
    float radius_;
    parameters_object params_;
};

} // namespace pcp

#endif // PCP_FILTER_DENSITY_FILTER_HPP
