#ifndef PCP_OCTREE_FLAT_OCTREE_HPP
#define PCP_OCTREE_FLAT_OCTREE_HPP


/**
 * @file
 * @ingroup octree
 */

#include "pcp/common/points/point.hpp"
#include "pcp/traits/range_traits.hpp"

#include <cassert>
#include <unordered_map>
#include <vector>

namespace pcp {

/**
 * @ingroup flat_octree
 * @brief Default type used to parameterize flat reprentation of octrees.
*/
struct flat_octree_parameters_t
{
    std::uint8_t depth = 12u;
};

/**
 * @ingroup flat-octree
 * @brief
 * An octree is a tree data structure for 3-dimensional quantities which
 * recursively subdivides a regular grid into its 8 octants by having
 * 8 child octrees. Octrees, much like binary trees, offer efficient
 * searching and insertion of those 3-d quantities (logarithmic time).
 * Interesting applications of octrees include efficient collision detection,
 * k-nearest-neighbor searches and range queries, among others.
 *
 * @tparam PointView Type satisfying PointView concept
 * @tparam ParamsType Type containing the parameters for this octree
 */
template <class PointView, class ParamsType = flat_octree_parameters_t>
class basic_flat_octree_t
{
  public:
    using point_view_type = PointView;
    using params_type = ParamsType;
    using value_type = point_view_type;
    using reference = value_type&;
    using const_reference = value_type const&;
    using pointer = value_type*;
    using const_pointer = value_type const*;
    using self_type = basic_flat_octree_t<PointView, ParamsType>;

    explicit basic_flat_octree_t(params_type const& params) 
        : depth_(params.depth), size_(0u)
    { 
        assert((depth_ > 0u) && (depth_ <= 21u));
    }

    template <class ForwardIter>
    explicit basic_flat_octree_t(
        ForwardIter begin, 
        ForwardIter end, 
        params_type const& params)
        : basic_flat_octree_t(params)
    {
        insert(begin, end);
    }

    template <class ForwardIter>
    explicit basic_flat_octree_t(ForwardIter begin, ForwardIter end) 
        : depth_(12u), size_(0u)
    {
        insert(begin, end);
    }

    template <class ForwardIter>
    std::size_t insert(ForwardIter begin, ForwardIter end)
    {
        // static assert here
        auto const inserted = std::accumulate(
            begin,
            end,
            static_cast<std::size_t(0u)>,
            [this](std::size_t const count, point_view_type const& p) {
                return this->insert(p) ? count + 1 : count;
            });
        size_ += inserted;
        return inserted;
    }

    bool insert(point_view_type const& p) 
    { 
        std::uint64_t encoding = encode(p);
        std::vector<point_view_type> points = map_[encoding];
        points.push_back(p);
        ++size_;
    }

    /*
     * Returns all points that reside in the same leaf.
     *
     * @param p A point in the leaf
     * @return A list of all points that reside in the same leaf
     */
    std::vector<point_view_type> find(point_view_type const& p) 
    {
        std::uint64_t encoding = encode(p);

        /* Might need to replace this to make a proper copy v v v*/
        std::vector<point_view_type> points = map_[encoding];

        return points;
    }

    /**
     * Returns the k-nearest-neighbours
     *
     * @param target The reference point for which we want the k nearest neighbours.
     * @param k The number of neighbours to return that are neareat to the specific point.
     * @return A list of nearest points ordered from nearest to furthest of size s where 0 <= s <= k.
     */
    /*
    template <class TPointView>
    std::vector<point_view_type> nearest_neighbours(TPointView const& target, std::size_t k, )
    {
    }
    */

    /**
    * Returns all points that reside in the given range.
    * 
    * @param range A range satisfying the Range type requirements.
    * @return A list of all the points that reside in the given range.
    */
    /*
    template <class Range> 
    std::vector<point_view_type> range_search(Range const& range) const
    {
    }
    */

    void clear() 
    { 
        for (auto const& [key, val] : map_)
        {
            val.clear();
        }

        map_.clear();

        size_ = 0u;
    }

    std::size_t size() const { return size_; }
    bool empty() const { return size() == 0u; }

  private:
    std::uint64_t encode(point_view_type const& p) {}

    std::size_t size_;
    std::uint8_t depth_;
    std::unordered_map<std::uint64_t, std::vector<point_view_type>> map_;
   
};

using flat_octree_t = pcp::basic_flat_octree_t<pcp::point_t, flat_octree_parameters_t>;