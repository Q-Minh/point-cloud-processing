#pragma once

#include <cmath>
#include <pcp/traits/point_traits.hpp>

namespace pcp {

/**
 * @brief 3-dimensional point
 * @tparam T Type of the point's (x,y,z) coordinates
 */
template <class T>
class basic_point_t
{
  public:
    using component_type  = T;
    using coordinate_type = T;
    using self_type       = basic_point_t<T>;

    T const& x() const { return x_; }
    T const& y() const { return y_; }
    T const& z() const { return z_; }

    void x(T const& value) { x_ = value; }
    void y(T const& value) { y_ = value; }
    void z(T const& value) { z_ = value; }

    basic_point_t() noexcept                       = default;
    basic_point_t(self_type const& other) noexcept = default;
    basic_point_t(self_type&& other) noexcept      = default;
    self_type& operator=(self_type const& other) noexcept = default;
    self_type& operator=(self_type&& other) noexcept = default;
    basic_point_t(T x, T y, T z) noexcept : x_(x), y_(y), z_(z) {}

    template <class PointView>
    basic_point_t(PointView const& other) noexcept : x_(other.x()), y_(other.y()), z_(other.z())
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
    }

    friend self_type operator*(coordinate_type k, self_type const& p) noexcept
    {
        return basic_point_t{
            k * p.x_,
            k * p.y_,
            k * p.z_,
        };
    }

    friend self_type operator/(self_type const& p, coordinate_type k) noexcept
    {
        return self_type{p.x_ / k, p.y_ / k, p.z_ / k};
    }

    template <class PointView>
    self_type operator+(PointView const& other) const noexcept
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return self_type{x() + other.x(), y() + other.y(), z() + other.z()};
    }

    template <class PointView>
    self_type operator-(PointView const& other) const noexcept
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return self_type{x() - other.x(), y() - other.y(), z() - other.z()};
    }

    self_type operator-() const noexcept { return self_type{-x(), -y(), -z()}; }

  private:
    coordinate_type x_ = 0., y_ = 0., z_ = 0.;
};

using point_t = basic_point_t<float>;

} // namespace pcp
