#pragma once

#include "traits/point_traits.hpp"

#include <iterator>

namespace pcp {

template <class PointView>
class point_view_t
{
    using point_type = PointView;
    static_assert(traits::is_point_view_v<point_type>, "PointView must satisfy PointView concept");

  public:
    using self_type       = point_view_t<point_type>;
    using component_type  = typename point_type::component_type;
    using coordinate_type = typename point_type::coordinate_type;
    using T               = component_type;

    explicit point_view_t(point_type* point) : point_(point) {}
    point_view_t()                       = default;
    point_view_t(self_type const& other) = default;
    point_view_t(self_type&& other)      = default;
    self_type& operator=(self_type const& other) = default;
    self_type& operator=(self_type&& other) = default;

    template <class TPointView>
    self_type& operator=(TPointView const& other)
    {
        static_assert(
            traits::is_point_view_v<TPointView>,
            "TPointView must satisfy PointView concept");
        x(other.x());
        y(other.y());
        z(other.z());
    }

    T const& x() const { return point_->x(); }
    T const& y() const { return point_->y(); }
    T const& z() const { return point_->z(); }

    void x(T value) { point_->x(value); }
    void y(T value) { point_->y(value); }
    void z(T value) { point_->z(value); }

    template <class TPointView>
    bool operator==(TPointView const& other) const
    {
        static_assert(
            traits::is_point_view_v<TPointView>,
            "PointView must satisfy PointView concept");
        T constexpr e     = static_cast<T>(1e-5);
        T const dx        = std::abs(x() - other.x());
        T const dy        = std::abs(y() - other.y());
        T const dz        = std::abs(z() - other.z());
        bool const equals = (dx < e) && (dy < e) && (dz < e);
        return equals;
    }
    template <class TPointView>
    bool operator!=(TPointView const& other) const
    {
        return !(*this == other);
    }

    void point(point_type* point) { point_ = point; }

  private:
    point_type* point_;
};

} // namespace pcp
