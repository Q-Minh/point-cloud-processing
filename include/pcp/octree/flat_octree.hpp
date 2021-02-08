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

#include <range/v3/view/subrange.hpp>
#include <range/v3/view/transform.hpp>

namespace pcp {

/**
 * @ingroup flat_octree
 * @brief Default type used to parameterize flat reprentation of octrees.
*/
template <class Point>
struct flat_octree_parameters_t
{
    using aabb_type = axis_aligned_bounding_box_t<Point>;
    using point_type = Point;

    std::uint8_t depth = 9u;
    aabb_type voxel_grid{};
};

template <class Element, class ParamsType>
class flat_octree_iterator_t;

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
template <class Element, class ParamsType = flat_octree_parameters_t>
class basic_flat_octree_t
{
  public:
    friend class flat_octree_iterator_t<Element, ParamsType>;

    using element_type = Element;
    using params_type = ParamsType;
    using value_type = element_type;
    using reference = value_type&;
    using const_reference = value_type const&;
    using pointer = value_type*;
    using const_pointer = value_type const*;
    using aabb_type = typename params_type::aabb_type;
    using aabb_point_type = typename aabb_type::point_type;
    using self_type = basic_flat_octree_t<element_type, params_type>;

  private:
    using container_type = std::vector<element_type>;
    using key_type = std::uint64_t;
    using map_type = std::unordered_map<key_type, container_type>;


  public:
    basic_flat_octree_t(self_type&& other) = default;

    explicit basic_flat_octree_t(params_type const& params) 
        : depth_(params.depth), size_(0u), voxel_grid_(params.voxel_grid)
    { 
        assert((depth_ > 0u) && (depth_ <= 21u));
        assert(
            (voxel_grid_.min.x() < voxel_grid_.max.x()) &&
            (voxel_grid_.min.y() < voxel_grid_.max.y()) &&
            (voxel_grid_.min.z() < voxel_grid_.max.z()));
    }

    template <class ForwardIter, class PointViewMap>
    explicit basic_flat_octree_t(
        ForwardIter begin, 
        ForwardIter end, 
        PointViewMap const& point_view,
        params_type const& params)
        : basic_flat_octree_t(params)
    {
        insert(begin, end, point_view);
    }

    template <class ForwardIter, class PointViewMap>
    explicit basic_flat_octree_t(
        ForwardIter begin,
        ForwardIter end, PointViewMap const& point_view)
        : depth_{}, size_{}, voxel_grid_{}
    {
        params_type params;
        auto const projection = [&](element_type const& e) {
            return point_view(e);
        };
        auto rng = ranges::make_subrange(begin, end) | ranges::views::transform(projection);
        using rng_iter_type = decltype(rng.begin());
        auto const bbox =
            pcp::bounding_box<rng_iter_type, aabb_point_type, aabb_type>(rng.begin(), rng.end());
        depth_      = params.depth;
        voxel_grid_ = bbox;
        size_       = insert(begin, end, point_view);
    }
    

    template <class ForwardIter, class PointViewMap>
    std::size_t insert(ForwardIter begin, ForwardIter end, PointViewMap const& point_view)
    {
        // static assert here
        auto const inserted = std::accumulate(
            begin,
            end,
            static_cast<std::size_t(0u)>,
            [this, &point_view](std::size_t const count, point_view_type const& p) {
                return this->insert(p, point_view) ? count + 1 : count;
            });
        size_ += inserted;
        return inserted;
    }

    template <class PointViewMap>
    bool insert(element_type const& element, PointViewMap const& point_view) 
    { 
        auto p = point_view(element);

        if (!voxel_grid_.contains(p))
            return false;

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
    std::vector<element_type> find(point_view_type const& p) 
    {
        std::uint64_t encoding = encode(p);

        /* Might need to replace this to make a proper copy v v v*/
        std::vector<element_type> points = map_[encoding];

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
    std::uint64_t encode(element_type const& element) 
    {
        std::uint32_t const m = std::pow(2, depth_);
        //auto const scale_x    = get_next_power_of_2(voxel_grid_.max.x() -)


        //auto const scale     = [](element_type const& e) -> float {
               
        //}
    }

    // std::float_t 

    std::uint32_t get_next_power_of_2(std::uint32_t n)
    {
        std::uint32_t r = 1, i = 0;
        while (r < n)
        {
            r = r << 1;
            ++i;
        }
        return r;
    }

    aabb_type voxel_grid_;
    std::size_t size_;
    std::uint8_t depth_;
    map_type map_;
   
};

using flat_octree_t = pcp::basic_flat_octree_t<pcp::point_t, flat_octree_parameters_t<pcp::point_t>;