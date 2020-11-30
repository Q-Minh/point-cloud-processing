#pragma once

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

} // namespace pcp