#ifndef PCP_ALGORITHM_COMMON_HPP
#define PCP_ALGORITHM_COMMON_HPP

namespace pcp {

template <class T>
T const& self(T const& any)
{
    return any;
}

template <class T>
T self(T const& any)
{
    return any;
}

template <class T>
T& self(T& any)
{
    return any;
}

namespace algorithm {

template <class Input, class Normal>
inline auto const default_normal_transform = [](Input const&, Normal const& n) {
    return n;
};

template <class Input, class Plane>
inline auto const default_plane_transform = [](Input const&, Plane const& plane) {
    return plane;
};


} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_COMMON_HPP