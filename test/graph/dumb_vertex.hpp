#pragma once

#include <cstdint>

namespace pcp {
namespace test {

struct dumb_vertex_t
{
    using id_type = std::uint32_t;

    dumb_vertex_t(id_type id) : id_(id) {}
    void id(id_type id) { id_ = id; }
    id_type id() const { return id_; }

    // STL algorithms need equality comparisons
    bool operator==(dumb_vertex_t const& other) const { return id_ == other.id_; }
    bool operator==(id_type id) const { return id_ == id; }

    id_type id_   = 0u;
};

} // namespace test
} // namespace pcp