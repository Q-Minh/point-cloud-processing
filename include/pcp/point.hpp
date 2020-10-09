#pragma once

namespace pcp {

template <class T /* point coordinates' type */>
struct basic_point_t
{
    T x = 0., y = 0., z = 0.;

    friend basic_point_t operator*(T k, basic_point_t p)
    {
        return basic_point_t{
            k * p.x,
            k * p.y,
            k * p.z,
        };
    }

    friend basic_point_t operator/(basic_point_t p, T k)
    {
        return basic_point_t{p.x / k, p.y / k, p.z / k};
    }

    bool operator==(basic_point_t const& p) const
    {
        if constexpr (std::is_integral_v<T>)
        {
            return p.x == x && p.y == y && p.z == z;
        }
        else
        {
            T constexpr e     = static_cast<T>(1e-5);
            T const dx        = std::abs(x - p.x);
            T const dy        = std::abs(y - p.y);
            T const dz        = std::abs(z - p.z);
            bool const equals = (dx < e) && (dy < e) && (dz < e);
            return equals;
        }
    };

    bool operator!=(basic_point_t const& p) const { return !(*this == p); }

    basic_point_t operator+(basic_point_t const& other) const
    {
        return basic_point_t{x + other.x, y + other.y, z + other.z};
    }

    basic_point_t operator-(basic_point_t const& other) const
    {
        return basic_point_t{x - other.x, y - other.y, z - other.z};
    }
};

using point_t = basic_point_t<float>;

} // namespace pcp