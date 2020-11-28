#pragma once

#include <cmath>
#include <cstdint>
#include <pcp/traits/point_traits.hpp>

namespace pcp {
namespace test {

struct custom_point_t
{
    using component_type  = float;
    using coordinate_type = float;
    using self_type       = custom_point_t;

    component_type const& x() const { return x_; }
    component_type const& y() const { return y_; }
    component_type const& z() const { return z_; }

    void x(component_type const& value) { x_ = value; }
    void y(component_type const& value) { y_ = value; }
    void z(component_type const& value) { z_ = value; }

    std::uint32_t dummy() const { return dummy_; }

    custom_point_t()                           = default;
    custom_point_t(self_type const& other)     = default;
    custom_point_t(self_type&& other) noexcept = default;
    self_type& operator=(self_type const& other) = default;
    self_type& operator=(self_type&& other) noexcept = default;
    custom_point_t(component_type x, component_type y, component_type z) : x_(x), y_(y), z_(z) {}

    template <class PointView>
    custom_point_t(PointView const& other) : x_(other.x()), y_(other.y()), z_(other.z())
    {
        static_assert(
            pcp::traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
    }

    friend self_type operator*(coordinate_type k, self_type const& p)
    {
        return self_type{
            k * p.x_,
            k * p.y_,
            k * p.z_,
        };
    }

    friend self_type operator/(self_type const& p, coordinate_type k)
    {
        return self_type{p.x_ / k, p.y_ / k, p.z_ / k};
    }

    template <class PointView>
    bool operator==(PointView const& p) const
    {
        static_assert(
            pcp::traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");

        coordinate_type constexpr e = static_cast<coordinate_type>(1e-5);
        coordinate_type const dx    = std::abs(x() - p.x());
        coordinate_type const dy    = std::abs(y() - p.y());
        coordinate_type const dz    = std::abs(z() - p.z());
        bool const equals           = (dx < e) && (dy < e) && (dz < e);
        return equals;
    }
    template <class PointView>
    bool operator!=(PointView const& p) const
    {
        return !(*this == p);
    }

    template <class PointView>
    self_type operator+(PointView const& other) const
    {
        static_assert(
            pcp::traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return self_type{x() + other.x(), y() + other.y(), z() + other.z()};
    }

    template <class PointView>
    self_type operator-(PointView const& other) const
    {
        static_assert(
            pcp::traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return self_type{x() - other.x(), y() - other.y(), z() - other.z()};
    }

  private:
    coordinate_type x_ = 0., y_ = 0., z_ = 0.;
    std::uint32_t dummy_ = 0u;
};

} // namespace test
} // namespace pcp
