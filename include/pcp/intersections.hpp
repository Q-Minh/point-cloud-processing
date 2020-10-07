#pragma once

#include "axis_aligned_bounding_box.hpp"
#include "sphere.hpp"

namespace pcp {
namespace intersections {

static inline float squared_distance(point_t const& p1, point_t const& p2)
{
	auto const dx = p2.x - p1.x;
	auto const dy = p2.y - p1.y;
	auto const dz = p2.z - p1.z;

	return dx * dx + dy * dy + dz * dz;
}

inline bool intersects(axis_aligned_bounding_box_t const& b1, axis_aligned_bounding_box_t const& b2)
{
	return
		(b1.max.x >= b2.min.x && b1.max.y >= b2.min.y && b1.max.z >= b2.min.z) &&
		(b1.min.x <= b2.max.x && b1.min.y <= b2.max.y && b1.min.z <= b2.max.z);
}

inline bool intersects(sphere_t const& s1, sphere_t const& s2)
{
	auto const c1 = s1.center();
	auto const c2 = s2.center();

	auto const maximum_distance_between_spheres = (s1.radius + s2.radius);
	auto const maximum_squared_distance_between_spheres = 
		maximum_distance_between_spheres * maximum_distance_between_spheres;

	return squared_distance(c1, c2) <= maximum_squared_distance_between_spheres;
}

inline bool intersects(axis_aligned_bounding_box_t const& b, sphere_t const& s)
{
	point_t const center = s.center();

	bool const is_center_in_box =
		center.x >= b.min.x &&
		center.y >= b.min.y &&
		center.z >= b.min.z &&
		center.x <= b.max.x &&
		center.y <= b.max.y &&
		center.z <= b.max.z;

	if (is_center_in_box)
		return true;

	point_t const nearest_point_on_box_from_sphere = b.nearest_point_from(center);
	return squared_distance(nearest_point_on_box_from_sphere, center) <= s.radius;
}

inline bool intersects(sphere_t const& s, axis_aligned_bounding_box_t const& b)
{
	return intersects(b, s);
}

} // intersections
} // pcp