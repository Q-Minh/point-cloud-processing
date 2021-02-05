#ifndef PCP_KDTREE_KDTREE_NODE_HPP
#define PCP_KDTREE_KDTREE_NODE_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include <algorithm>
#include <array>
#include <vector>

namespace pcp {

template <class Element>
class basic_linked_kdtree_node_t
{
  public:
    using self_type     = basic_linked_kdtree_node_t;
    using self_type_ptr = std::unique_ptr<self_type>;
    using element_type  = Element;
    using points_type   = std::vector<element_type*>;

    self_type_ptr& left() { return left_; }
    self_type_ptr& right() { return right_; }

    self_type_ptr const& left() const { return left_; }
    self_type_ptr const& right() const { return right_; }

    void set_left(self_type_ptr&& l) { left_ = std::move(l); }
    void set_right(self_type_ptr&& r) { right_ = std::move(r); }

    points_type& points() { return points_; }
    points_type const& points() const { return points_; }

    bool is_leaf() const { return left_ == nullptr && right_ == nullptr; }
    bool is_internal() const { return !is_leaf(); }

  private:
    std::unique_ptr<self_type> left_;
    std::unique_ptr<self_type> right_;
    points_type points_;
};

} // namespace pcp

#endif // PCP_KDTREE_KDTREE_NODE_HPP