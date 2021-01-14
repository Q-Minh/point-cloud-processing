#ifndef PCP_COMMON_VECTOR3D_HPP
#define PCP_COMMON_VECTOR3D_HPP

#include <pcp/traits/vector3d_traits.hpp>

namespace pcp {
namespace common {

/**
 * @brief General 3-dimensional vector
 * @tparam T Type of this vector's components
 */
template <class T>
class basic_vector3d_t
{
  public:
    using component_type = T;
    using self_type      = basic_vector3d_t<T>;

    basic_vector3d_t() noexcept                 = default;
    basic_vector3d_t(self_type const&) noexcept = default;
    basic_vector3d_t(self_type&&) noexcept      = default;
    self_type& operator=(self_type const&) noexcept = default;
    self_type& operator=(self_type&&) noexcept = default;

    basic_vector3d_t(component_type x, component_type y, component_type z) noexcept : x_(x), y_(y), z_(z)
    {
    }

    template <class Vector3d>
    basic_vector3d_t(Vector3d const& other)
    {
        static_assert(traits::is_vector3d_v<Vector3d>, "other must satisfy Vector3d concept");
        x(other.x());
        y(other.y());
        z(other.z());
    }

    template <class Vector3d>
    basic_vector3d_t(Vector3d&& other)
    {
        static_assert(traits::is_vector3d_v<Vector3d>, "other must satisfy Vector3d concept");
        x(other.x());
        y(other.y());
        z(other.z());
    }

    component_type x() const { return x_; }
    component_type y() const { return y_; }
    component_type z() const { return z_; }
    void x(component_type value) { x_ = value; }
    void y(component_type value) { y_ = value; }
    void z(component_type value) { z_ = value; }

    friend self_type operator*(component_type k, self_type const& v)
    {
        return self_type{k * v.x(), k * v.y(), k * v.z()};
    }

    friend self_type operator/(self_type const& v, component_type k)
    {
        return self_type{v.x() / k, v.y() / k, v.z() / k};
    }

    self_type operator+(self_type const& v)
    {
        return self_type{x() + v.x(), y() + v.y(), z() + v.z()};
    }

    self_type operator-(self_type const& v)
    {
        return self_type{x() - v.x(), y() - v.y(), z() - v.z()};
    }

  private:
    component_type x_, y_, z_;
};

using vector3d_t = basic_vector3d_t<float>;

} // namespace common
} // namespace pcp

#endif // PCP_COMMON_VECTOR3D_HPP