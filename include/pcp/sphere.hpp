#pragma once

#include "point.hpp"

namespace pcp {

template <class Point>
struct sphere_t
{
    Point position{0.f, 0.f, 0.f};
    float radius = 0.f;

    Point center() const { return position; }

    bool contains(Point const& p) const
    {
        auto const distance = [](Point const& p1, Point const& p2) -> float {
            auto const dx = p2.x() - p1.x();
            auto const dy = p2.y() - p1.y();
            auto const dz = p2.z() - p1.z();

            return dx * dx + dy * dy + dz * dz;
        };

        /*
         * d = sqrt(a^2 + b^2 + c^2)
         * equivalent to
         * d^2 = a^2 + b^2 + c^2
         */
        return distance(position, p) <= radius * radius;
    }
};

} // namespace pcp
