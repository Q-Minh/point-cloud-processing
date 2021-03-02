#ifndef PCP_COMMON_SPHERE_HPP
#define PCP_COMMON_SPHERE_HPP

/**
 * @file
 * @ingroup common
 */

#include "norm.hpp"

namespace pcp {

/**
 * @ingroup geometric-primitives
 * @brief
 * Simple sphere with containment predicate.
 * @tparam Point Type satisfying Point concept
 */
template <class Point>
struct sphere_t
{
    Point position{0.f, 0.f, 0.f};
    typename Point::coordinate_type radius = static_cast<typename Point::coordinate_type>(0.);

    Point center() const { return position; }

    bool contains(Point const& p) const
    {
        /*
         * d = sqrt(a^2 + b^2 + c^2)
         * equivalent to
         * d^2 = a^2 + b^2 + c^2
         */
        return common::squared_distance(position, p) <= radius * radius;
    }
};
/**
 * @ingroup geometric-primitives
 * @brief
 * Simple sphere with containment predicate.
 * @tparam Type Type of the data
 */
template <class CoordinateType>
struct sphere_a
{
    using point_type = std::array<CoordinateType, 3>;
    point_type position;
    CoordinateType radius;

    point_type center() const { return position; }

    bool contains(point_type const& p) const
    {
        CoordinateType distance = common::squared_distance(position, p);
        return distance <= radius * radius;
    }
};

} // namespace pcp

#endif // PCP_COMMON_SPHERE_HPP
