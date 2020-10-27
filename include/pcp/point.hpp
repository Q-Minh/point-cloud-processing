#pragma once

#include <cmath>

namespace pcp {

template <class T /* point coordinates' type */>
struct basic_point_t
{
    using component_type = T;
    using coordinate_type = T;
    using self_type = basic_point_t<T>;

    T const& x() const { return x_; }
    T const& y() const { return y_; }
    T const& z() const { return z_; }

    void x(T const& value) { x_ = value; }
    void y(T const& value) { y_ = value; }
    void z(T const& value) { z_ = value; }

    basic_point_t() = default;
    basic_point_t(self_type const& other) = default;
    basic_point_t(self_type&& other) = default;
    self_type& operator=(self_type const& other) = default;
    basic_point_t(T x, T y, T z) : x_(x), y_(y), z_(z) {}

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

    bool operator==(self_type const& p) const
    {
        if constexpr (std::is_integral_v<coordinate_type>)
        {
            return p.x_ == x_ && p.y_ == y_ && p.z_ == z_;
        }
        else
        {
            coordinate_type constexpr e = static_cast<coordinate_type>(1e-5);
            coordinate_type const dx    = std::abs(x_ - p.x_);
            coordinate_type const dy    = std::abs(y_ - p.y_);
            coordinate_type const dz    = std::abs(z_ - p.z_);
            bool const equals           = (dx < e) && (dy < e) && (dz < e);
            return equals;
        }
    };

    bool operator!=(self_type const& p) const { return !(*this == p); }

    self_type operator+(self_type const& other) const
    {
        return self_type{x_ + other.x_, y_ + other.y_, z_ + other.z_};
    }

    self_type operator-(self_type const& other) const
    {
        return self_type{x_ - other.x_, y_ - other.y_, z_ - other.z_};
    }

  private:
    coordinate_type x_ = 0., y_ = 0., z_ = 0.;
};

using point_t = basic_point_t<float>;

} // namespace pcp
