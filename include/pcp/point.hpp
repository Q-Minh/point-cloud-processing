#pragma once

#include "traits/point_traits.hpp"

#include <cmath>

namespace pcp {

template <class T /* point coordinates' type */>
struct basic_point_t
{
    using component_type  = T;
    using coordinate_type = T;
    using self_type       = basic_point_t<T>;

    T const& x() const { return x_; }
    T const& y() const { return y_; }
    T const& z() const { return z_; }

    void x(T const& value) { x_ = value; }
    void y(T const& value) { y_ = value; }
    void z(T const& value) { z_ = value; }

    basic_point_t()                           = default;
    basic_point_t(self_type const& other)     = default;
    basic_point_t(self_type&& other) noexcept = default;
    self_type& operator=(self_type const& other) = default;
    self_type& operator=(self_type&& other) noexcept = default;
    basic_point_t(T x, T y, T z) : x_(x), y_(y), z_(z) {}

    template <class PointView>
    basic_point_t(PointView const& other) : x_(other.x()), y_(other.y()), z_(other.z())
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
    }

    friend self_type operator*(coordinate_type k, self_type const& p)
    {
        return basic_point_t{
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
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        if constexpr (std::is_integral_v<coordinate_type>)
        {
            return p.x_ == x_ && p.y_ == y_ && p.z_ == z_;
        }
        else
        {
            coordinate_type constexpr e = static_cast<coordinate_type>(1e-5);
            coordinate_type const dx    = std::abs(x() - p.x());
            coordinate_type const dy    = std::abs(y() - p.y());
            coordinate_type const dz    = std::abs(z() - p.z());
            bool const equals           = (dx < e) && (dy < e) && (dz < e);
            return equals;
        }
    };
    template <class PointView>
    bool operator!=(PointView const& p) const
    {
        return !(*this == p);
    }

    template <class PointView>
    self_type operator+(PointView const& other) const
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return self_type{x() + other.x(), y() + other.y(), z() + other.z()};
    }

    template <class PointView>
    self_type operator-(PointView const& other) const
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return self_type{x() - other.x(), y() - other.y(), z() - other.z()};
    }

  private:
    coordinate_type x_ = 0., y_ = 0., z_ = 0.;
};

using point_t = basic_point_t<float>;

} // namespace pcp
