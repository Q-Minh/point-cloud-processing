#pragma once

#include "point.hpp"

#include <array>

namespace pcp {

struct axis_aligned_bounding_box_t
{
    point_t min{}, max{};

    bool contains(point_t const& p) const
    {
        auto const greater_than_or_equal = [](point_t const& p1, point_t const& p2) -> bool
        {
            return p1.x >= p2.x && p1.y >= p2.y && p1.z >= p2.z;
        };

        auto const less_than_or_equal = [](point_t const& p1, point_t const& p2) -> bool
        {
            return p1.x <= p2.x && p1.y <= p2.y && p1.z <= p2.z;
        };

        return greater_than_or_equal(p, min) && less_than_or_equal(p, max);
    }

    point_t center() const
    {
        return (min + max) / 2.f;
    }

    point_t nearest_point_from(point_t const& p) const
    {
        /*
        * The faces are indexed in this fashion:
        * 0: front face of cube
        * 1: right face of cube
        * 2: back face of cube
        * 3: left face of cube
        * 4: bottom face of cube
        * 5: top face of cube
        */
        std::array<float, 6> distances_to_box_faces{ 0.f, };

        /*
        * Here, we calculate shortest distances squared from point p
        * to each face of this axis-aligned bounding box.
        */
        distances_to_box_faces[0] = (min.y - p.y) * (min.y - p.y);
        distances_to_box_faces[1] = (max.x - p.x) * (max.x - p.x);
        distances_to_box_faces[2] = (max.y - p.y) * (max.y - p.y);
        distances_to_box_faces[3] = (min.x - p.x) * (min.x - p.x);
        distances_to_box_faces[4] = (min.z - p.z) * (min.z - p.z);
        distances_to_box_faces[5] = (max.z - p.z) * (max.z - p.z);

        /*
        * Find the index of the closest face of this axis-aligned
        * bounding box to the point p.
        */
        auto const it = std::min_element(distances_to_box_faces.cbegin(), distances_to_box_faces.cend());
        auto const index = std::distance(distances_to_box_faces.cbegin(), it);

        point_t nearest_point = p;
        nearest_point.x = std::clamp(nearest_point.x, min.x, max.x);
        nearest_point.y = std::clamp(nearest_point.y, min.y, max.y);
        nearest_point.z = std::clamp(nearest_point.z, min.z, max.z);

        return nearest_point;
    }
};

} // pcp
