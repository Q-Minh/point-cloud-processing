#pragma once

namespace pcp {

template <class T>
struct basic_normal_t
{
    using component_type = T;

    basic_normal_t() = default;
    basic_normal_t(T x, T y, T z) : x_(x), y_(y), z_(z) {}

    T const& x() const { return x_; }
    T const& y() const { return y_; }
    T const& z() const { return z_; }
    void x(T value) { x_ = value; }
    void y(T value) { y_ = value; }
    void z(T value) { z_ = value; }

    T const& nx() const { return x_; }
    T const& ny() const { return y_; }
    T const& nz() const { return z_; }
    void nx(T value) { x_ = value; }
    void ny(T value) { y_ = value; }
    void nz(T value) { z_ = value; }

    friend basic_normal_t operator*(T k, basic_normal_t p)
    {
        return basic_normal_t{
            k * p.x_,
            k * p.y_,
            k * p.z_,
        };
    }

    friend basic_normal_t operator/(basic_normal_t n, T k)
    {
        return basic_normal_t{n.x_ / k, n.y_ / k, n.z_ / k};
    }

    bool operator==(basic_normal_t const& n) const
    {
        if constexpr (std::is_integral_v<T>)
        {
            return n.x_ == x_ && n.y_ == y_ && n.z_ == z_;
        }
        else
        {
            T constexpr e     = static_cast<T>(1e-5);
            T const dx        = std::abs(x_ - n.x_);
            T const dy        = std::abs(y_ - n.y_);
            T const dz        = std::abs(z_ - n.z_);
            bool const equals = (dx < e) && (dy < e) && (dz < e);
            return equals;
        }
    };

    bool operator!=(basic_normal_t const& n) const { return !(*this == n); }

    basic_normal_t operator+(basic_normal_t const& other) const
    {
        return basic_normal_t{x_ + other.x_, y_ + other.y_, z_ + other.z_};
    }

    basic_normal_t operator-(basic_normal_t const& other) const
    {
        return basic_normal_t{x_ - other.x_, y_ - other.y_, z_ - other.z_};
    }

    basic_normal_t operator-() const { return basic_normal_t{-x_, -y_, -z_}; }

  private:
    T x_ = 0., y_ = 0., z_ = 0.;
};

using normal_t = basic_normal_t<float>;

} // namespace pcp
