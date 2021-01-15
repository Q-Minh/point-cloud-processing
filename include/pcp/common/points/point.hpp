#ifndef PCP_COMMON_POINTS_POINT_HPP
#define PCP_COMMON_POINTS_POINT_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/common/vector3d.hpp"
#include "pcp/traits/point_traits.hpp"

#include <cmath>

namespace pcp {

/**
 * @ingroup geometric-primitives
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

    template <class Vector3d>
    self_type operator+(Vector3d const& v) const noexcept
    {
        static_assert(traits::is_vector3d_v<Vector3d>, "Vector3d must satisfy Vector3d concept");
        return self_type{x() + v.x(), y() + v.y(), z() + v.z()};
    }

    template <class PointView, class Vector3d = common::basic_vector3d_t<coordinate_type>>
    Vector3d operator-(PointView const& other) const noexcept
    {
        static_assert(
            traits::is_point_view_v<PointView>,
            "PointView must satisfy PointView concept");
        return Vector3d{x() - other.x(), y() - other.y(), z() - other.z()};
    }

    self_type operator-() const noexcept { return self_type{-x(), -y(), -z()}; }

  private:
    coordinate_type x_ = 0., y_ = 0., z_ = 0.;
};

/**
 * @ingroup geometric-primitives
 * @brief
 * Default point type
 */
using point_t = basic_point_t<float>;

} // namespace pcp

#endif // PCP_COMMON_POINTS_POINT_HPP