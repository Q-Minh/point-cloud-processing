#pragma once

#include "traits/point_traits.hpp"

#include <iterator>

namespace pcp {

template <class Point>
class point_view_t
{
    using point_type = Point;
    static_assert(traits::is_point_view_v<point_type>, "Point must satisfy PointView concept");

  public:
    using self_type       = point_view_t<point_type>;
    using component_type  = typename point_type::component_type;
    using coordinate_type = typename point_type::coordinate_type;
    using T               = component_type;

    point_view_t()                                = default;
    point_view_t(self_type const& other) = default;
    point_view_t(self_type&& other)      = default;
    self_type& operator=(self_type const& other) = default;

    self_type& operator=(point_type const& other) { point_ = &other; }
    point_view_t(point_type point) : point_(&point) {}
    explicit point_view_t(point_type* point) : point_(point) {}

    T const& x() const { return (*point_).x(); }
    T const& y() const { return (*point_).y(); }
    T const& z() const { return (*point_).z(); }

    T& x() { return (*point_).x(); }
    T& y() { return (*point_).y(); }
    T& z() { return (*point_).z(); }

    bool operator!=(self_type const& other) const { return *point_ != *(other.point_); }
    bool operator==(self_type const& other) const { return *point_ == *(other.point_); }

  private:
    point_type* point_;
};

} // namespace pcp
