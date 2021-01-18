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
    float radius = 0.f;

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

} // namespace pcp

#endif // PCP_COMMON_SPHERE_HPP
