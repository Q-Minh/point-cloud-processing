#ifndef PCP_OCTREE_FLAT_OCTREE_HPP
#define PCP_OCTREE_FLAT_OCTREE_HPP

/**
 * @file
 * @ingroup octree
 */

#include "pcp/common/points/point.hpp"
#include "pcp/common/vector3d_queries.hpp"
#include "pcp/octree/flat_octree_iterator.hpp"
#include "pcp/traits/point_traits.hpp"
#include "pcp/traits/range_traits.hpp"

#include <cassert>
#include <queue>
#include <range/v3/view/subrange.hpp>
#include <range/v3/view/transform.hpp>
#include <unordered_map>
#include <vector>

namespace pcp {

/**
 * @ingroup flat_octree
 * @brief Default type used to parameterize flat reprentation of octrees.
 */
template <class Point>
struct flat_octree_parameters_t
{
    using aabb_type  = axis_aligned_bounding_box_t<Point>;
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
template <class Element, class ParamsType = flat_octree_parameters_t<pcp::point_t>>
class basic_flat_octree_t
{
    static_assert(traits::is_point_view_v<Element>, "Element must satisfy PointView concept");

  public:
    friend class flat_octree_iterator_t<Element, ParamsType>;

    using element_type    = Element;
    using params_type     = ParamsType;
    using value_type      = element_type;
    using reference       = value_type&;
    using const_reference = value_type const&;
    using pointer         = value_type*;
    using const_pointer   = value_type const*;
    using iterator        = flat_octree_iterator_t<Element, ParamsType>;
    using const_iterator  = iterator const;
    using aabb_type       = typename params_type::aabb_type;
    using aabb_point_type = typename aabb_type::point_type;
    using coordinate_type = typename element_type::coordinate_type;
    using self_type       = basic_flat_octree_t<element_type, params_type>;

  private:
    using container_type = std::vector<element_type>;
    using key_type       = std::uint_fast64_t;
    using int_type       = std::uint_fast32_t;
    using map_type       = std::unordered_map<key_type, container_type>;

  public:

    /**
     * @brief Default move constructor
     * @param other Copied-from octree
     */
    basic_flat_octree_t(self_type&& other) = default;

     /**
     * @brief
     * Constructs this octree with configuration specified by params
     * @param params The configuration for this octree
     */
    explicit basic_flat_octree_t(params_type const& params)
        : depth_(params.depth), size_(0u), voxel_grid_(params.voxel_grid)
    {
        assert((depth_ > 0u) && (depth_ <= 21u));
        assert(
            (voxel_grid_.min.x() < voxel_grid_.max.x()) &&
            (voxel_grid_.min.y() < voxel_grid_.max.y()) &&
            (voxel_grid_.min.z() < voxel_grid_.max.z()));
        calc_factor();
    }

    /**
     * @brief
     * Constructs this octree from a range of elements
     * @tparam ForwardIter Type of iterator to the elements
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param begin Begin iterator to the elements
     * @param end End iterator to the elements
     * @param point_view The point view property map
     * @param params The configuration for this octree
     */
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

    /**
     * @brief
     * Constructs this octree from a range of elements.
     * Computes the octree's configuration automatically.
     * @tparam ForwardIter Type of iterator to the elements
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param begin Begin iterator to the elements
     * @param end The point view property map
     * @param point_view The point view property map
     */
    template <class ForwardIter, class PointViewMap>
    explicit basic_flat_octree_t(ForwardIter begin, ForwardIter end, PointViewMap const& point_view)
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
        calc_factor();
        size_ = insert(begin, end, point_view);
    }

    /**
     * @brief Iterator to the first element of this octree
     * @return Iterator to the first element of this octree
     */
    iterator begin() { return iterator(&map_); }

    /**
     * @brief End iterator to this octree's elements
     * @return End iterator to this octree's elements
     */
    iterator end()
    {
        auto it = iterator(&map_);
        it.make_end_iterator();

        return it;
    }

    /**
     * @brief Const iterator to the first element of this octree
     * @return Const iterator to the first element of this octree
     */
    const_iterator begin() const { return const_iterator(const_cast<map_type*>(&map_)); }

    /**
     * @brief End const iterator to this octree's elements
     * @return End const iterator to this octree's elements
     */
    const_iterator end() const
    {
        map_type* map = const_cast<map_type*>(&map_);
        auto it       = iterator(map);
        it.make_end_iterator();

        return it;
    }

    /**
     * @brief Const iterator to the first element of this octree
     * @return Const iterator to the first element of this octree
     */
    const_iterator cbegin() const { return const_iterator(const_cast<map_type*>(&map_)); }

    /**
     * @brief End const iterator to this octree's elements
     * @return End const iterator to this octree's elements
     */
    const_iterator cend() const { return end(); }

    /**
     * @brief Insert range of elements in the octree
     * @tparam ForwardIter Type of the range's iterators
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param begin Iterator to the first element of the range
     * @param end End iterator of the range
     * @param point_view The point view property map
     * @return The number of inserted elements
     */
    template <class ForwardIter, class PointViewMap>
    std::size_t insert(ForwardIter begin, ForwardIter end, PointViewMap const& point_view)
    {
        // static assert here
        auto const inserted = std::accumulate(
            begin,
            end,
            static_cast<std::size_t>(0u),
            [this, &point_view](std::size_t const count, auto const& p) {
                return this->insert(p, point_view) ? count + 1 : count;
            });
        return inserted;
    }

    /**
     * @brief Insert one element in the octree
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param e The element to insert
     * @param point_view The point view property map
     * @return true if insert was successful
     */
    template <class PointViewMap>
    bool insert(element_type const& element, PointViewMap const& point_view)
    {
        auto p = point_view(element);

        if (!voxel_grid_.contains(p))
            return false;

        key_type key           = compute_key(p);
        container_type& points = map_[key];
        points.push_back(element);
        ++size_;
        return true;
    }

    /*
     * @brief Returns an iterator to the element in the octree if it exists
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param element Element to find
     * @param point_view The point view property map
     * @return Iterator to the find element
     */
    template <class PointViewMap>
    const_iterator find(element_type const& element, PointViewMap const& point_view) const
    {
        auto p = point_view(element);

        if (!voxel_grid_.contains(p))
            return end();

        auto k = compute_key(p);

        map_type& map = const_cast<map_type&>(map_);

        auto map_it = map.find(k);

        // If there's no key associated with the point, then the point is surely not there
        if (map_it == map.end())
            return end();

        auto& current_octant = map_it->second;
        auto current_it      = std::find_if(
            current_octant.begin(),
            current_octant.end(),
            [&p, &point_view](auto const& e) {
                auto p2 = point_view(e);
                return common::are_vectors_equal(p, p2);
            });

        // If we somehow didn't find the point in the current octant
        if (current_it == current_octant.end())
            return end();

        iterator it(map_it, map.end(), current_it);

        return it;
    }

    /*
     * @brief Removes the element pointed-to by iterator pos.
     * @param pos Iterator to the element to remove. Must be in the range [cbegin(), cend()).
     * @return Iterator to the next element or cend()
     */
    const_iterator erase(const_iterator it)
    {
        iterator next = it;

        container_type* octant = const_cast<container_type*>(&next.map_it_->second);

        if (next.it_ != octant->cend())
        {
            const_cast<decltype(next.it_)&>(next.it_) = octant->erase(it.it_);
            --size_;
        }

        if (!octant->empty())
            return next;
        else
        {
            const_cast<decltype(next.map_it_)&>(next.map_it_) = map_.erase(it.map_it_);
            next.refresh();
        }
        return next;
    }

    /**
     * @brief KNN search
     * @tparam TPointView Type satisfying PointView concept
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param target Position around which we want to find the k nearest neighbours
     * @param k Number of nearest neighbours to query
     * @param point_view The point view property map
     * @param eps The error tolerance for floating point equality
     * @return The k nearest neighbours
     */
    template <class TPointView, class PointViewMap>
    std::vector<element_type> nearest_neighbours(
        TPointView const& target,
        std::size_t k,
        PointViewMap const& point_view,
        double const eps = 1e-5) const
    {
        if (k <= 0u)
            return {};

        // Calculer la réprensation en float du point dans morton
        TPointView rep = get_float_rep_morton(target);

        struct min_heap_node_t
        {
            container_type const* elements = nullptr;
            coordinate_type distance       = 0.f;
        };

        auto const greater = [rep](min_heap_node_t const& h1, min_heap_node_t const& h2) -> bool {
            auto const d1 = h1.distance;
            auto const d2 = h2.distance;

            return d1 > d2;
        };

        auto const lesser_coordinates = [rep, this](auto const& c1, auto const& c2) -> bool {
            auto const r1 = get_float_rep_morton(c1);
            auto const r2 = get_float_rep_morton(c2);

            auto const d1 = common::squared_distance(rep, r1);
            auto const d2 = common::squared_distance(rep, r2);

            return d1 < d2;
        };

        auto const lesser_elements = [lesser_coordinates, point_view, this](
                                         element_type const& e1,
                                         element_type const& e2) -> bool {
            auto const& c1 = point_view(e1);
            auto const& c2 = point_view(e2);

            return lesser_coordinates(c1, c2);
        };

        using min_heap_t =
            std::priority_queue<min_heap_node_t, std::vector<min_heap_node_t>, decltype(greater)>;

        using max_heap_t =
            std::priority_queue<element_type, std::vector<element_type>, decltype(lesser_elements)>;

        // Créer les structures de données
        max_heap_t max_heap(lesser_elements);

        // If k is bigger than the number of points in the octree
        if (size_ <= k)
        {
            auto it = begin();
            for (; it != end(); ++it)
            {
                auto p = point_view(*it);
                if (!common::are_vectors_equal(p, target, static_cast<coordinate_type>(eps)))
                    max_heap.push(*it);
                else
                {
                    break; // We can stop searching and add all the rest to the heap
                }
            }
            while (++it != end())
            {
                max_heap.push(*it);
            }
        }
        else
        {
            min_heap_t min_heap(greater);

            map_type& map = const_cast<map_type&>(map_);

            for (auto it = map.begin(); it != map.end(); ++it)
            {
                auto index               = it->first;
                auto points              = &it->second;
                auto voxel               = get_voxel_from_morton(index);
                auto const closest_point = voxel.nearest_point_from(rep);
                auto distance            = common::squared_distance(rep, closest_point);

                auto node = min_heap_node_t{points, distance};
                min_heap.push(node);
            }

            while (!min_heap.empty())
            {
                min_heap_node_t node = min_heap.top();
                min_heap.pop();

                for (auto it = node.elements->begin(); it != node.elements->end(); ++it)
                {
                    auto p = point_view(*it);
                    if (!common::are_vectors_equal(p, target, static_cast<coordinate_type>(eps)))
                        max_heap.push(*it);
                }

                while (max_heap.size() > k)
                {
                    max_heap.pop();
                }

                if (max_heap.size() == k)
                {
                    auto p  = point_view(max_heap.top());
                    auto dp = common::squared_distance(get_float_rep_morton(p), rep);

                    if (!min_heap.empty())
                    {
                        min_heap_node_t next_node = min_heap.top();

                        if (dp < next_node.distance)
                        {
                            break;
                        }
                    }
                }
            }
        }

        std::vector<element_type> knearest_neighbours{};
        knearest_neighbours.reserve(k);
        while (!max_heap.empty())
        {
            knearest_neighbours.push_back(max_heap.top());
            max_heap.pop();
        }
        std::reverse(knearest_neighbours.begin(), knearest_neighbours.end());
        return knearest_neighbours;
    }

    /**
     * @brief Returns all points that reside in the given range.
     * @tparam Range Type satisfying Range concept
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param range The range in which we went to find points
     * @param elements_in_range The elements found to be in the range
     * @param point_view The point view property map
     */
    template <class Range, class PointViewMap>
    std::vector<element_type> range_search(Range const& range, PointViewMap const& point_view) const
    {
        std::vector<element_type> elements_in_range;

        for (auto it_m = map_.begin(); it_m != map_.end(); ++it_m)
        {
            aabb_type voxel = get_real_voxel_from_morton(it_m->first);
            if (intersections::intersects(range, voxel))
            {
                auto octant = it_m->second;

                for (auto const& e : octant)
                {
                    if (range.contains(point_view(e)))
                        elements_in_range.push_back(e);
                }
            }
        } // Worse case O(n), but still more efficient that the other one

        return elements_in_range;
    }

    /**
     * @brief Remove all elements from this octree
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

    /**
     * @brief Gets the top-level voxel from this octree (the bounding box)
     * @return This octree's voxel
     */
    aabb_type const& voxel_grid() const { return voxel_grid_; }

    /**
     * @brief Number of elements in the octree
     * @return Number of elements in the octree
     */
    std::size_t size() const { return size_; }
    
    /**
     * @brief Checks if octree is empty
     * @return True if octree is empty
     */
    bool empty() const { return size() == 0u; }

  private:

    /**
     * @brief Compute the key of the given point as morton index
     * @tparam TPointView Type satisfying PointView concept
     * @param p The point compute key
     * @returns The morton index
     */
    template <class TPointView>
    key_type compute_key(TPointView const& p) const
    {
        auto a = get_float_rep_morton(p);

        return encode_morton(
            static_cast<int_type>(a.x()),
            static_cast<int_type>(a.y()),
            static_cast<int_type>(a.z()));
    }

    /**
     * @brief Calculate the float reprentation of the point in morton scale
     * @tparam TPointView Type satisfying PointView concept
     * @param p The point to scale
     * @returns The point scaled to morton 
     */
    template <class TPointView>
    TPointView get_float_rep_morton(TPointView const& p) const
    {
        auto a            = p - voxel_grid_.min;
        coordinate_type x = a.x() * factor_.x();
        coordinate_type y = a.y() * factor_.y();
        coordinate_type z = a.z() * factor_.z();

        return TPointView{x, y, z};
    }

    /**
     * @brief Gives the morton index of a given point
     * @param x The x value of the point
     * @param y The y value of the point
     * @param z The z value of the point
     * @return Morton index of the given point
     */
    inline key_type encode_morton(int_type x, int_type y, int_type z) const
    {
        auto xx = split_by_3(x);
        auto yy = split_by_3(y);
        auto zz = split_by_3(z);

        key_type ret = 0;
        ret |= xx | yy << 1 | zz << 2;
        return ret;
    }

    /**
     * @brief Split a morton index of an axis with 0s
     * @param a The integer representation of the morton index of an axis to split
     * @return Integer representation of a splitted morton index of an axis splitted
     */
    inline key_type split_by_3(int_type a) const
    {
        key_type x = a & 0x1fffff;
        x          = (x | x << 32) & 0x1f00000000ffff;
        x          = (x | x << 16) & 0x1f0000ff0000ff;
        x          = (x | x << 8) & 0x100f00f00f00f00f;
        x          = (x | x << 4) & 0x10c30c30c30c30c3;
        x          = (x | x << 2) & 0x1249249249249249;
        return x;
    }

    /**
     * @brief Calculate the real voxel of the morton key
     * @param key The morton key
     * @return Voxel in real scale of the morton key
     */
    inline aabb_type get_real_voxel_from_morton(key_type key) const
    {
        auto morton_voxel = get_voxel_from_morton(key);

        auto min     = morton_voxel.min;
        auto max     = morton_voxel.max;
        auto new_min = aabb_point_type{min.x() / factor_.x(), min.y() / factor_.y(), min.z() / factor_.z()} + voxel_grid_.min;
        auto new_max = aabb_point_type{max.x() / factor_.x(), max.y() / factor_.y(), max.z() / factor_.z()} + voxel_grid_.min;

        return aabb_type{new_min, new_max};
    }

    /**
     * @brief Calculate the voxel of the morton key
     * @param key The morton key
     * @return Voxel in morton scale of the morton key
     */
    inline aabb_type get_voxel_from_morton(key_type key) const
    {
        auto x = static_cast<coordinate_type>(group_from_3(key));
        auto y = static_cast<coordinate_type>(group_from_3(key >> 1));
        auto z = static_cast<coordinate_type>(group_from_3(key >> 2));

        return aabb_type{{x, y, z}, {x + 1, y + 1, z + 1}};
    }

    /**
     * @brief Regroup 3d morton indexes
     * @param a The integer representation of the morton index of an axis splitted
     * @return Integer reprensation of the morton index on an axis grouped
     */
    inline int_type group_from_3(key_type a) const
    {
        key_type x = a & 0x1249249249249249;
        x          = (x ^ (x >> 2)) & 0x10c30c30c30c30c3;
        x          = (x ^ (x >> 4)) & 0x100f00f00f00f00f;
        x          = (x ^ (x >> 8)) & 0x1f0000ff0000ff;
        x          = (x ^ (x >> 16)) & 0x1f00000000ffff;
        x          = (x ^ (static_cast<key_type>(x) >> 32)) & 0x1fffff;

        return static_cast<int_type>(x);
    }


    /**
     * @brief Calculate the factor to scale points to morton indexes
     */
    void calc_factor()
    {
        coordinate_type base = 2;
        coordinate_type pow  = std::pow(base, static_cast<coordinate_type>(depth_));
        auto factor          = aabb_point_type{
            pow / (voxel_grid_.max.x() - voxel_grid_.min.x()),
            pow / (voxel_grid_.max.y() - voxel_grid_.min.y()),
            pow / (voxel_grid_.max.z() - voxel_grid_.min.z())};
        factor_ = factor;
    }

    aabb_point_type factor_;
    aabb_type voxel_grid_;
    std::size_t size_;
    std::uint8_t depth_;
    map_type map_;
};

using flat_octree_t = pcp::basic_flat_octree_t<pcp::point_t>;

} // namespace pcp

#endif // PCP_OCTREE_FLAT_OCTREE_HPP