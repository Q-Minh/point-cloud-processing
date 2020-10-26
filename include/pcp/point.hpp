#pragma once

#include <cmath>

namespace pcp {

template <class T /* point coordinates' ty_pe */>
struct basic_point_t
{
    using coordinate_type = T;

    T const& x() const { return x_; }
    T const& y() const { return y_; }
    T const& z() const { return z_; }

    T& x() { return x_; }
    T& y() { return y_; }
    T& z() { return z_; }

    basic_point_t() = default;
    basic_point_t(T x, T y, T z) : x_(x), y_(y), z_(z) {}

    friend basic_point_t operator*(coordinate_type k, basic_point_t p)
    {
        return basic_point_t{
            k * p.x_,
            k * p.y_,
            k * p.z_,
        };
    }

    friend basic_point_t operator/(basic_point_t p, coordinate_type k)
    {
        return basic_point_t{p.x_ / k, p.y_ / k, p.z_ / k};
    }

    bool operator==(basic_point_t const& p) const
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

    bool operator!=(basic_point_t const& p) const { return !(*this == p); }

    basic_point_t operator+(basic_point_t const& other) const
    {
        return basic_point_t{x_ + other.x_, y_ + other.y_, z_ + other.z_};
    }

    basic_point_t operator-(basic_point_t const& other) const
    {
        return basic_point_t{x_ - other.x_, y_ - other.y_, z_ - other.z_};
    }

  private:
    coordinate_type x_ = 0., y_ = 0., z_ = 0.;
};

using point_t = basic_point_t<float>;

} // namespace pcp
