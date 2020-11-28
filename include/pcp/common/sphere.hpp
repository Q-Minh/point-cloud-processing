#pragma once

#include "norm.hpp"

namespace pcp {

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
