#ifndef PCP_KDTREE_KDTREE_NODE_HPP
#define PCP_KDTREE_KDTREE_NODE_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include <memory>
#include <vector>

namespace pcp {

/**
 *  @ingroup linked-kd-tree
 * @brief
 * A kdtree node at internal should only contain an element,
 * while a leaf node may contain more than one element
 * @tparam Element The element type
 */
template <class Element>
class basic_linked_kdtree_node_t
{
  public:
    using self_type     = basic_linked_kdtree_node_t;
    using self_type_ptr = std::unique_ptr<self_type>;
    using element_type  = Element;
    using points_type   = std::vector<element_type*>;

    /**
     * @brief Get the right child of the node
     * @return Right child of the node
     */
    self_type_ptr& left() { return left_; }

    /**
     * @brief Get the right child of the node
     * @return Right child of the node
     */
    self_type_ptr& right() { return right_; }

    /**
     * @brief Get the left child of the node (readonly)
     * @return Left child of the node (readonly)
     */
    self_type_ptr const& left() const { return left_; }

    /**
     * @brief Get the right child of the node (readonly)
     * @return Right child of the node (readonly)
     */
    self_type_ptr const& right() const { return right_; }

    /**
     * @brief Set the left child
     * @param l the left child
     */
    void set_left(self_type_ptr&& l) { left_ = std::move(l); }

    /**
     * @brief Set the right chikd
     * @param r the right child
     */
    void set_right(self_type_ptr&& r) { right_ = std::move(r); }

    /**
     * @brief Get the list of elements in the node
     * internal node only contains 1 element
     * Leaf node may contain more than 1 element
     * @return List of elements in the node
     */
    points_type& points() { return points_; }

    /**
     * @brief Get the list of elements in the node (readonly)
     * internal node only contains 1 element
     * Leaf node may contain more than 1 element
     * @return List of elements in the node (readonly)
     */
    points_type const& points() const { return points_; }

    /**
     * @brief Check if node is leaf
     * @return True if node is leaf
     */
    bool is_leaf() const { return left_ == nullptr && right_ == nullptr; }

    /**
     * @brief Check if leaf is internal
     * @return True if leaf is internal
     */
    bool is_internal() const { return !is_leaf(); }

  private:
    std::unique_ptr<self_type> left_;
    std::unique_ptr<self_type> right_;
    points_type points_;
};

} // namespace pcp

#endif // PCP_KDTREE_KDTREE_NODE_HPP