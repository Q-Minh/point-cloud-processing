#pragma once

#include "octree_node.hpp"

namespace pcp {

/*
 * Read-only forward iterator for points in the octree
 */
class octree_iterator_t
{
  public:
    using value_type        = typename octree_node_t::value_type;
    using difference_type   = std::size_t;
    using reference         = typename octree_node_t::reference;
    using pointer           = typename octree_node_t::pointer;
    using iterator_category = std::forward_iterator_tag;

    using const_reference = typename octree_node_t::const_reference;

  private:
    using point_iterator        = typename octree_node_t::points_type::iterator;
    using const_point_iterator  = typename octree_node_t::points_type::const_iterator;
    using octant_iterator       = typename octree_node_t::octants_type::iterator;
    using const_octant_iterator = typename octree_node_t::octants_type::const_iterator;

  public:
    friend class octree_node_t;

    octree_iterator_t() : octree_node_(nullptr), it_() {}

    octree_iterator_t(octree_node_t const* octree_node) : octree_node_(nullptr), it_()
    {
        octree_node_ = get_next_node(octree_node, octree_node->octants_.cbegin());
        it_          = octree_node_->points_.cbegin();
    }

    octree_iterator_t(octree_iterator_t const& other)     = default;
    octree_iterator_t(octree_iterator_t&& other) noexcept = default;

    octree_iterator_t& operator=(octree_iterator_t const& other) = default;

    octree_node_t const* root() const
    {
        using stack_type = decltype(ancestor_octree_nodes_);

        decltype(ancestor_octree_nodes_) copy;
        while (!ancestor_octree_nodes_.empty())
        {
            copy.push(ancestor_octree_nodes_.top());
            const_cast<stack_type&>(ancestor_octree_nodes_).pop();
        }

        auto const* root = copy.top();

        while (!copy.empty())
        {
            const_cast<stack_type&>(ancestor_octree_nodes_).push(copy.top());
            copy.pop();
        }

        return root;
    }

    const_reference operator*() const { return *it_; }

    octree_iterator_t& operator++()
    {
        /*
         * There are still points in this octree node, just
         * return the next point in line.
         */
        if (++it_ != octree_node_->points_.cend())
            return *this;

        /*
         * If there are no ancestors, the current octree node
         * is the root node. If we have exhausted all points
         * of the root node, then we have reached the end
         * iterator.
         */
        if (ancestor_octree_nodes_.empty())
        {
            octree_node_ = nullptr;
            it_          = decltype(it_){};
            return *this;
        }

        /*
         * If there are no more points in this octree node,
         * but there are still points remaining in the tree,
         * we move to the next node in the sequence.
         */
        move_to_next_node();
        return *this;
    }

    octree_iterator_t const& operator++() const { return ++(*this); }

    octree_iterator_t& operator++(int)
    {
        octree_iterator_t previous{*this};
        ++(*this);
        return previous;
    }

    octree_iterator_t const& operator++(int) const { return (*this)++; }

    bool operator==(octree_iterator_t const& other) const
    {
        return (octree_node_ == other.octree_node_) && (it_ == other.it_);
    }

    bool operator!=(octree_iterator_t const& other) const { return !(*this == other); }

    point_t const* operator->() const { return &(*it_); }

  private:
    /*
     * Returns the next node in post-order sequence.
     */
    octree_node_t const* get_next_node(octree_node_t const* octree, const_octant_iterator begin)
    {
        auto const end = octree->octants_.cend();

        for (auto octree_child_node_it = begin; octree_child_node_it != end; ++octree_child_node_it)
        {
            auto const& octree_child_node = *octree_child_node_it;
            if (!octree_child_node)
                continue;

            ancestor_octree_nodes_.push(octree);
            return get_next_node(octree_child_node.get(), octree_child_node->octants_.begin());
        }

        /*
         * If there were no more children to visit from this octree,
         * it means this octree node is the next node to visit in
         * post-order sequence.
         */
        return octree;
    }

    void move_to_next_node()
    {
        /*
         * If we've exhausted all points of this octree node,
         * then it's time to go back to our parent and look
         * for the next point from there.
         */
        auto const* parent = ancestor_octree_nodes_.top();
        ancestor_octree_nodes_.pop();

        auto const is_same_octant =
            [target = octree_node_](
                std::iterator_traits<const_octant_iterator>::reference octant) -> bool {
            return octant.get() == target;
        };

        /*
         * Find our next non-null sibling
         */
        const_octant_iterator next_octant =
            std::find_if(parent->octants_.cbegin(), parent->octants_.cend(), is_same_octant);

        octree_node_ = get_next_node(parent, ++next_octant);

        /*
         * Once we've found the next octree node in the
         * sequence, we initialize our internal iterator
         * to the first point of that octree node.
         */
        it_ = octree_node_->points_.cbegin();
    }

    octree_node_t const* octree_node_;
    std::stack<octree_node_t const*> ancestor_octree_nodes_;
    const_point_iterator it_;
};

} // namespace pcp