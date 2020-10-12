#pragma once

#include "intersections.hpp"

#include <algorithm>
#include <cassert>
#include <memory>
#include <numeric>
#include <queue>
#include <stack>
#include <vector>

namespace pcp {

struct octree_parameters_t
{
    std::uint32_t node_capacity = 1u;
    std::uint8_t max_depth      = 21;
    axis_aligned_bounding_box_t voxel_grid{};
};

class octree_iterator_t;

class octree_node_t
{
  public:
    friend class octree_iterator_t;

    using iterator        = octree_iterator_t;
    using const_iterator  = octree_iterator_t const;
    using value_type      = point_t;
    using reference       = value_type&;
    using const_reference = value_type const&;
    using pointer         = value_type*;
    using const_pointer   = value_type const*;

    explicit octree_node_t(octree_parameters_t const& params)
        : capacity_(params.node_capacity),
          max_depth_(params.max_depth),
          voxel_grid_(params.voxel_grid),
          octants_(),
          points_()
    {
        assert(capacity_ > 0u);
        assert(max_depth_ > 0u);
        assert(
            (voxel_grid_.min.x < voxel_grid_.max.x) && (voxel_grid_.min.y < voxel_grid_.max.y) &&
            (voxel_grid_.min.z < voxel_grid_.max.z));
        points_.reserve(params.node_capacity);
    }

    template <class ForwardIter>
    explicit octree_node_t(ForwardIter begin, ForwardIter end, octree_parameters_t const& params)
        : octree_node_t(params)
    {
        insert(begin, end);
    }

    void clear()
    {
        points_.clear();
        for (auto& octant : octants_)
            octant.release();
    }

    template <class ForwardIter>
    std::size_t insert(ForwardIter begin, ForwardIter end)
    {
        return std::accumulate(
            begin,
            end,
            static_cast<std::size_t>(0u),
            [this](std::size_t const count, point_t const& p) {
                return this->insert(p) ? count + 1 : count;
            });
    }

    bool insert(point_t const& p)
    {
        /*
         * If the point does not reside in the voxel grid
         * that this octree deals with, we don't insert it.
         * An octree holds points in a regular grid in 3d
         * space and spatially subdivides the grid further.
         */
        if (!voxel_grid_.contains(p))
            return false;

        /*
         * If this octree node has reached the maximum depth
         * of the octree defined by the root node, we know
         * we can't create any more child nodes, so we just
         * append new points to our list of points, having
         * no regard for this node's capacity.
         */
        if (max_depth_ == 1u)
        {
            points_.push_back(p);
            return true;
        }

        /*
         * If this octree node still has space to hold more
         * points, just append it to our list of points.
         */
        auto const num_points = points_.size();
        if (num_points < capacity_)
        {
            points_.push_back(p);
            return true;
        }

        /*
         * If this octree node's list of points can't grow
         * further, just insert it in this octree's child
         * node that contains the point to be inserted.
         * Since this octree node's regular grid is spatially
         * subdivided in 8 separate octants, there can be
         * only 1 octant that contains this point.
         */
        point_t const center = voxel_grid_.center();

        /*
         * Since we know that an octree may only have 8
         * child nodes, we need only 3 bits to uniquely
         * identify which octree child node corresponds
         * to which octant.
         *
         * We label the octants:
         *
         *           o----------o----------o
         *          /|         /|         /|
         *         / |   011  / |  111   / |
         *        o--|-------o--|-------o  |
         *       /|  o------/|--o------/|--o
         *      / | /|001  / | /| 101 / | /|
         *     o--|/-|----o--|/-|----o  |/ |
         *     |  o--|----|--o--|----|--o  |
         *     | /|  o----|-/|--o----|-/|--o
         *     |/ | /  010|/ | /  110|/ | /
         *     o--|/------o--|/------o  |/
         *     |  o-------|--o-------|--o
         *     | /        | /        | /
         *     |/   000   |/   100   |/
         *     o----------o----------o
         *
         * So the bottom octants will be: 000, 100, 110, 010
         *   and the top octants will be: 001, 101, 111, 011
         *
         * Basically, we choose the order of the bits to correspond to xyz directions.
         * For example, the child octant o of this octree node's center c,
         * having index 100, contains the set of points p where:
         *
         * - x coordinate of p > x coordinate of c => true
         * - y coordinate of p > y coordinate of c => false
         * - z coordinate of p > z coordinate of c => false
         *
         * Additionally, the child octant o of this octree node's center c,
         * having index 101, contains the set of points p where:
         *
         * - x coordinate of p > x coordinate of c => true
         * - y coordinate of p > y coordinate of c => false
         * - z coordinate of p > z coordinate of c => true
         *
         * Going from 000 to 111 in binary, and looking at their
         * corresponding octants shows how the 8 octants are
         * uniquely identified by this indexing scheme.
         *
         */
        std::uint64_t octants_bitmask = 0b000;

        if (p.x > center.x)
            octants_bitmask |= 0b100;
        if (p.y > center.y)
            octants_bitmask |= 0b010;
        if (p.z > center.z)
            octants_bitmask |= 0b001;

        /*
         * Since the octants bitmask can hold values 000 to 111 in binary,
         * or 0 to 7 in decimal, we can simply hold the child octree nodes
         * in a fixed array of 8 contiguous octree nodes which naturally
         * supports 0-based indexing.
         */
        auto& octant = octants_[octants_bitmask];

        /*
         * If the child octree node for this octant already exists,
         * then the child is also an octree, and we can delegate
         * the work of inserting point p to the child octree. This
         * is the beauty of recursion and divide-and-conquer.
         */
        if (octant)
            return octant->insert(p);

        /*
         * If the child octree node for this octant does not exist
         * yet, we create it and delegate the work of inserting
         * point p to the newly created child octree.
         */
        octree_parameters_t params;

        /*
         * This octree node must propagate the node capacity
         * parameter to its children from top to bottom.
         */
        params.node_capacity = capacity_;

        /*
         * Creating a child octree node implies having moved
         * down a level in the tree. To propagate this information,
         * we simply assign a value for the max depth of the
         * created child to be 1 less than this octree node's
         * max depth. By doing so, if we wanted a max depth of
         * 2 for this octree node, for example, then the created
         * child node will have a max depth of 1. Looking at the
         * "if (max_depth_ == 1u)" check at the start of the insertion,
         * we see that at that moment, we will not create any
         * child octree nodes anymore, but instead append points to
         * the octree's leaf nodes' list of points.
         *
         * If we wanted a max depth of 3, then the root octree node's
         * children will have a max depth of 3 -1 = 2. Then, each
         * child's children will have a max depth of 2 -1 = 1, at
         * which point we will have reached the desired 3-level
         * octree form that was initially specified.
         *
         * This works for any initial max depth > 0.
         */
        params.max_depth = max_depth_ - std::uint8_t{1u};

        params.voxel_grid.min.x = octants_bitmask & 0b100 ? center.x : voxel_grid_.min.x;
        params.voxel_grid.max.x = octants_bitmask & 0b100 ? voxel_grid_.max.x : center.x;

        params.voxel_grid.min.y = octants_bitmask & 0b010 ? center.y : voxel_grid_.min.y;
        params.voxel_grid.max.y = octants_bitmask & 0b010 ? voxel_grid_.max.y : center.y;

        params.voxel_grid.min.z = octants_bitmask & 0b001 ? center.z : voxel_grid_.min.z;
        params.voxel_grid.max.z = octants_bitmask & 0b001 ? voxel_grid_.max.z : center.z;

        octant = std::make_unique<octree_node_t>(params);
        return octant->insert(p);
    }

    const_iterator find(point_t const& p) const;

    const_iterator erase(const_iterator it);

    std::vector<point_t> nearest_neighbours(point_t const& target, std::size_t k) const
    {
        if (k <= 0u)
            return {};

        /*
         * The l2-norm is defined as sqrt(x*x + y*y + z*z), but we don't actually need
         * the sqrt computation for distance comparisons:
         *
         * Having points p1 and p2, we have that:
         * sqrt((p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2) < sqrt((p2.x-p.x)^2 + (p2.y-p.y)^2 +
         * (p2.z-p.z)^2) is equivalent to (p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2 < (p2.x-p.x)^2
         * + (p2.y-p.y)^2 + (p2.z-p.z)^2
         *
         * since we are squaring both sides of the equation.
         */
        auto const distance = [](point_t const& p1, point_t const& p2) -> float {
            auto const x = (p2.x - p1.x);
            auto const y = (p2.y - p1.y);
            auto const z = (p2.z - p1.z);
            return x * x + y * y + z * z;
        };

        struct min_heap_node_t
        {
            point_t const* p       = nullptr;
            octree_node_t const* o = nullptr;
            bool is_point          = false;
        };

        /*
         * This predicate defines our priority queue to be a min-heap, that
         * is to say that the priority queue is a heap in which the top element
         * is the point/octant of this octree nearest to the reference point p.
         */
        auto const greater =
            [&distance, &target](min_heap_node_t const& h1, min_heap_node_t const& h2) -> bool {
            point_t const& p1 = h1.is_point ? *h1.p : h1.o->voxel_grid_.nearest_point_from(target);
            point_t const& p2 = h2.is_point ? *h2.p : h2.o->voxel_grid_.nearest_point_from(target);

            auto const d1 = distance(target, p1);
            auto const d2 = distance(target, p2);

            return d1 > d2;
        };

        /*
         * This min heap holds points or octants. The trick is that when the
         * top element of the min heap is a point, we know that this point
         * is the closest one to the reference point, because no other point
         * in all of the other octants could be closer. In other words, if
         * there was a point in any of the octants in the min heap that was
         * the closest to the reference point, the top element of the min
         * heap would be an octant, not a point.
         *
         * Since we start adding elements to the min heap starting from the
         * root of the octree, we know that the octants fill up all the
         * space in the octree. Hence, there is no subset of the voxel grid
         * that will not have been considered in this algorithm.
         */
        using min_heap_t =
            std::priority_queue<min_heap_node_t, std::vector<min_heap_node_t>, decltype(greater)>;
        min_heap_t min_heap(greater);

        /*
         * Add the root octree node to the heap
         */
        min_heap.push(min_heap_node_t{nullptr, this, false});

        std::vector<point_t> knearest_points{};
        // we only need up to k elements, so we can reserve the memory upfront
        knearest_points.reserve(k);

        /*
         * We continue searching for nearest neighbours until we have
         * k nearest neighbors, or until we have visited every single
         * point contained in this octree. Basically, continue searching
         * if we have less than k nearest neighbours and the min heap
         * still hasn't been exhausted.
         */
        while (knearest_points.size() < k && !min_heap.empty())
        {
            min_heap_node_t heap_node = min_heap.top();
            min_heap.pop();

            /*
             * If the heap node is a point, then we know for sure that
             * this point is closer to the reference point than any other
             * point contained in any octant of the octree. In this case,
             * we have effectively found a new nearest neighbour.
             */
            if (heap_node.is_point)
            {
                knearest_points.push_back(*heap_node.p);
                continue;
            }

            /*
             * If the heap node is an octant, then we know that there are
             * points in this octant that are closer to the reference point
             * than any other point in any other octant of this octree.
             * We add points of this octree node to the priority queue.
             */
            for (auto const& p : heap_node.o->points_)
            {
                min_heap.push(min_heap_node_t{&p, nullptr, true});
            }

            /*
             * We also add the child octree nodes of this octree
             * node.
             */
            for (auto const& octree_child_node : heap_node.o->octants_)
            {
                if (!octree_child_node)
                    continue;

                min_heap.push(min_heap_node_t{nullptr, octree_child_node.get(), false});
            }
        }

        return knearest_points;
    }

    template <class Range>
    void range_search(Range const& range, std::vector<point_t>& points_in_range) const
    {
        for (auto const& p : points_)
            if (range.contains(p))
                points_in_range.push_back(p);

        for (auto const& octree_child_node : octants_)
        {
            if (!octree_child_node)
                continue;

            /*
             * If the queried range does not even intersect this
             * octant, then no point in that octant can be contained
             * in the queried range. In that case, we can discard
             * searching in this whole octant.
             */
            if (!intersections::intersects(octree_child_node->voxel_grid_, range))
                continue;

            /*
             * If the queried range does intersect this octant,
             * then any point of this octant could be inside or
             * outside of the queried range, but we don't know
             * which. In this case, we simply delegate the job
             * of searching to this octant's octree node.
             */
            octree_child_node->range_search(range, points_in_range);
        }
    }

  protected:
    const_iterator do_find(point_t const& p, octree_iterator_t& it) const;

  private:
    using points_type  = std::vector<point_t>;
    using octants_type = std::array<std::unique_ptr<octree_node_t>, 8>;

    typename octants_type::const_iterator take_point_from_first_nonempty_octant()
    {
        auto const exists = [](std::unique_ptr<octree_node_t> const& o) {
            return static_cast<bool>(o);
        };

        /*
         * Look for first non-null octant of this octree.
         */
        auto octree_child_node_it = std::find_if(octants_.begin(), octants_.end(), exists);

        /*
         * If this octree has no octant, then we can't
         * take points from any of its children, so
         * just return the octants' end() iterator.
         */
        if (octree_child_node_it == octants_.cend())
            return octants_.cend();

        auto& octree_child_node = *octree_child_node_it;

        /*
         * Steal a point from this child octant.
         */
        points_.push_back(octree_child_node->points_.back());
        octree_child_node->points_.pop_back();

        /*
         * If the octree's child octant still has points left,
         * we don't need to touch the octree anymore.
         */
        if (!octree_child_node->points_.empty())
            return octree_child_node_it;

        /*
         * If we've stolen the octree's child octant's last point,
         * then we tell it to steal a point from its own children
         * recursively.
         *
         * If it can't manage to steal any, though, it means this
         * octree's child octant can't fill itself up with a point
         * anymore. In that case, we have to destroy this child.
         */
        if (octree_child_node->take_point_from_first_nonempty_octant() ==
            octree_child_node->octants_.cend())
        {
            octree_child_node.release();
        }

        /*
         * Return the octree's child octant that we have stolen
         * a point from.
         */
        return octree_child_node_it;
    }

    std::uint32_t capacity_;
    std::uint8_t max_depth_;
    axis_aligned_bounding_box_t voxel_grid_;
    octants_type octants_;
    points_type points_;
};

} // namespace pcp

#include "octree_iterator.hpp"

namespace pcp {

inline octree_node_t::const_iterator octree_node_t::find(point_t const& p) const
{
    /*
     * Pretty much the same implementation as an insert, except we return the found point
     * if it exists, instead of inserting.
     */
    if (!voxel_grid_.contains(p))
        return octree_iterator_t{};

    octree_iterator_t it{};
    return this->do_find(p, it);
}

inline octree_node_t::const_iterator
octree_node_t::do_find(point_t const& p, octree_iterator_t& it) const
{
    it.octree_node_ = this;
    it.it_          = std::find(points_.cbegin(), points_.cend(), p);
    if (it.it_ != points_.cend())
        return it;

    point_t const center          = voxel_grid_.center();
    std::uint64_t octants_bitmask = 0b000;

    if (p.x > center.x)
        octants_bitmask |= 0b100;
    if (p.y > center.y)
        octants_bitmask |= 0b010;
    if (p.z > center.z)
        octants_bitmask |= 0b001;

    auto& octant = octants_[octants_bitmask];

    if (!octant)
        return octree_iterator_t{};

    it.ancestor_octree_nodes_.push(this);
    return octant->do_find(p, it);
}

inline octree_node_t::const_iterator octree_node_t::erase(const_iterator it)
{
    iterator next = it;

    /*
     * Get the octree node that the iterator currently
     * resides in.
     */
    octree_node_t* octree_node = const_cast<octree_node_t*>(next.octree_node_);

    /*
     * If the point to erase actually exists, then
     * we erase it and set the iterator's point to
     * the next point in the sequence.
     */
    if (next.it_ != octree_node->points_.cend())
        const_cast<decltype(next.it_)&>(next.it_) = octree_node->points_.erase(it.it_);

    /*
     * If after erasing the point, we still have other
     * points in this node, then simply return the next
     * iterator which already resides in the right octree
     * node and which already points to the next point in
     * the sequence.
     */
    if (!octree_node->points_.empty())
        return next;

    /*
     * If this is an internal node, then it will succeed in
     * taking a point from one of its children.
     */
    if (octree_node->take_point_from_first_nonempty_octant() != octree_node->octants_.cend())
    {
        next.it_ = octree_node->points_.cbegin();
        return next;
    }

    /*
     * Return end() iterator if this is the root node
     * (which happens to be a leaf node) and there
     * are no points left.
     */
    if (next.ancestor_octree_nodes_.empty())
    {
        return octree_iterator_t{};
    }

    /*
     * If this is a leaf node, then we simply move the iterator
     * to the next node, and we remove this leaf from its parent
     * since the leaf is empty.
     */
    auto* parent = const_cast<octree_node_t*>(next.ancestor_octree_nodes_.top());

    /*
     * Move the iterator to the next point before we change the structure
     * of the tree by releasing the leaf. The next iterator used to reside
     * in the leaf node that we are about to delete, so we must move the
     * iterator before the deletion.
     */
    next.move_to_next_node();

    auto const is_same_octant =
        [octree_node](std::unique_ptr<octree_node_t> const& octant) -> bool {
        return octant.get() == octree_node;
    };

    /*
     * Find the leaf and release/delete it.
     */
    auto octant_it = std::find_if(parent->octants_.begin(), parent->octants_.end(), is_same_octant);
    octant_it->release();

    return next;
}

} // namespace pcp
