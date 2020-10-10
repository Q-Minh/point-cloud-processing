#pragma once

#include "point.hpp"

#include <array>

namespace pcp {

struct axis_aligned_bounding_box_t
{
    point_t min{}, max{};

    bool contains(point_t const& p) const
    {
        auto const greater_than_or_equal = [](point_t const& p1, point_t const& p2) -> bool {
            return p1.x >= p2.x && p1.y >= p2.y && p1.z >= p2.z;
        };

        auto const less_than_or_equal = [](point_t const& p1, point_t const& p2) -> bool {
            return p1.x <= p2.x && p1.y <= p2.y && p1.z <= p2.z;
        };

        return greater_than_or_equal(p, min) && less_than_or_equal(p, max);
    }

    point_t center() const { return (min + max) / 2.f; }

    point_t nearest_point_from(point_t const& p) const
    {
        point_t nearest_point = p;
        nearest_point.x       = std::clamp(nearest_point.x, min.x, max.x);
        nearest_point.y       = std::clamp(nearest_point.y, min.y, max.y);
        nearest_point.z       = std::clamp(nearest_point.z, min.z, max.z);

        return nearest_point;
    }
};

} // namespace pcp
