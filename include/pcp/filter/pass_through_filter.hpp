#ifndef PCP_FILTER_PASS_THROUGH_FILTER_HPP
#define PCP_FILTER_PASS_THROUGH_FILTER_HPP

/**
 * @file
 * @ingroup filter
 */

#include <algorithm>
#include <pcp/kdtree/linked_kdtree.hpp>

namespace pcp {

namespace pass_through_filter {

template <class ScalarType>
struct construction_params_t
{
    using scalar_type             = ScalarType;
    scalar_type filter_limit_min_ = std::numeric_limits<scalar_type>::lowest();
    scalar_type filter_limit_max_ = std::numeric_limits<scalar_type>::max();
    std::size_t index_to_filter_  = 0;
};
} // namespace pass_through_filter
/**
 * @brief Iterates through the entire input once, and for each point, will check
 * if it satisfies the maximum and minimum value for the filter for a certain coordinate
 * index_to_filter() is the index of the coordinate which we filter on
 *  index : 0 for x; 1 for y; 2 for z
 * filter_limit_min() and filter_limit_max() is the range
 *
 * @tparam Element Element type
 * @tparam ParametersType type of the parameters (double,float)
 * @tparam CoordinateMap coordinate map for a point
 */
template <class Element, class ParametersType, class CoordinateMap>

class basic_pass_through_filter_t
{
  public:
    using element_type      = Element;
    using coordinate_map    = CoordinateMap;
    using kdtree_type       = pcp::basic_linked_kdtree_t<element_type, 3u, coordinate_map>;
    using parameters_type   = ParametersType;
    using parameters_object = pass_through_filter::construction_params_t<parameters_type>;
    template <class ForwardIterator>
    basic_pass_through_filter_t(
        ForwardIterator begin,
        ForwardIterator end,
        CoordinateMap coordinate_map,
        kdtree_type& kdtree,
        parameters_object params)
        : coordinate_map_(coordinate_map), kdtree_(kdtree), params_(params)
    {
    }
    /**
     * @brief
     * @param min
     */
    void filter_limit_min(parameters_type min) { params_.filter_limit_min_ = min; }
    /**
     * @brief
     * @param max
     */
    void filter_limit_max(parameters_type max) { params_.filter_limit_max_ = max; }
    /**
     * @brief sets index that the filter be applied on (0 for x, 1 for y, 2 for z)
     * @param index
     */
    void index_to_filter(std::size_t index) { params_.index_to_filter = index; }
    /**
     * @brief
     * @return filter_limit_min
     */
    parameters_type filter_limit_min() const { return params_.filter_limit_min_; }
    /**
     * @brief
     * @return filter_limit_max
     */
    parameters_type filter_limit_max() const { return params_.filter_limit_max_; }
    /**
     * @brief
     * @return index that the filter be applied on (0 for x, 1 for y, 2 for z)
     */
    std::size_t index_to_filter() const { return index_to_filter; }

    bool operator()(element_type const& p)
    {
        auto const p_one_coordinate = coordinate_map_(p)[params_.index_to_filter_];
        return (
            p_one_coordinate < params_.filter_limit_min_ ||
            p_one_coordinate > params_.filter_limit_max_);
    }

  private:
    kdtree_type& kdtree_;
    coordinate_map coordinate_map_;
    parameters_object params_;
};

} // namespace pcp

#endif // PCP_FILTER_PASS_THROUGH_FILTER_HPP
