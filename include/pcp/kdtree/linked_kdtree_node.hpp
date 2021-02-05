#ifndef PCP_KDTREE_KDTREE_NODE_HPP
#define PCP_KDTREE_KDTREE_NODE_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include "pcp/common/points/point.hpp"

#include <algorithm>
#include <array>
#include <vector>

namespace pcp {

class basic_linked_kdtree_node_t
{
  public:
    using self_type     = basic_linked_kdtree_node_t;
    using self_type_ptr = std::unique_ptr<self_type>;
    using element_type  = pcp::point_t;
    using points_type   = std::vector<element_type*>;

    enum class construction_t { approximate_median, exact_median };

    self_type_ptr& left() { return left_; }
    self_type_ptr& right() { return right_; }

    self_type_ptr const& left() const { return left_; }
    self_type_ptr const& right() const { return right_; }

    void set_left(self_type_ptr&& l) { left_ = std::move(l); }
    void set_right(self_type_ptr&& r) { right_ = std::move(r); }

    points_type& points() { return points_; }
    points_type const& points() const { return points_; }

  private:
    std::unique_ptr<self_type> left_;
    std::unique_ptr<self_type> right_;
    std::vector<pcp::point_t*> points_;
};

} // namespace pcp

#endif // PCP_KDTREE_KDTREE_NODE_HPP