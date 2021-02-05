#ifndef PCP_basic_kdtree_t_HPP
#define PCP_basic_kdtree_t_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include "pcp/common/points/point.hpp"
#include "pcp/kdtree/linked_kdtree_node.hpp"
#include "pcp/traits/range_traits.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

namespace pcp {

class basic_linked_kdtree_t
{
  public:
    using self_type        = basic_linked_kdtree_t;
    using element_type     = pcp::point_t;
    using node_type        = basic_linked_kdtree_node_t;
    using node_type_ptr    = std::unique_ptr<node_type>;
    using coordinates_type = std::array<float, 3u>;

    enum class construction_t { approximate_median, exact_median };

    basic_linked_kdtree_t(
        std::vector<element_type> const& storage,
        std::size_t max_depth = 12u,
        construction_t c      = construction_t::exact_median)
        : max_depth_{max_depth}, storage_{storage}, root_{}
    {
        if (c == construction_t::approximate_median)
        {
        }
        if (c == construction_t::exact_median)
        {
            construct_exact_median();
        }
    }

    bool empty() const { return storage_.empty(); }
    std::size_t size() const { return storage_.size(); }

  public:
    struct less_than_t
    {
        bool operator()(element_type const& e1, element_type const& e2) const
        {
            coordinates_type coordinates1{e1.x(), e1.y(), e1.z()};
            coordinates_type coordinates2{e2.x(), e2.y(), e2.z()};

            for (auto i = 0u; i < coordinates1.size(); ++i)
            {
                if (coordinates1[i] < coordinates2[i])
                    return true;
                if (coordinates1[i] > coordinates2[i])
                    return false;
            }
            return false;
        }
    };

  private:
    void construct_exact_median()
    {
        std::sort(storage_.begin(), storage_.end(), less_than_t{});
        std::size_t first         = 0u;
        std::size_t last          = storage_.size() - 1u;
        std::size_t current_depth = 0u;
        root_                     = construct_exact_median_recursive(first, last, current_depth);
    }

    std::unique_ptr<node_type>
    construct_exact_median_recursive(std::size_t first, std::size_t last, std::size_t current_depth)
    {
        /**
         * No left sub-tree for parent node
         */
        if (last < first)
        {
            return nullptr;
        }

        auto leaf    = std::make_unique<node_type>();
        auto& points = leaf->points();

        /**
         * Leaf node
         */
        if (current_depth == max_depth_ - 1u)
        {
            points.reserve((last + 1u) - first);
            auto begin     = storage_.begin() + first;
            auto end       = storage_.begin() + last + 1u;
            auto out_begin = points.begin();

            std::transform(begin, end, out_begin, [](element_type const& e) {
                return std::addressof(e);
            });
            return leaf;
        }

        /**
         * Leaf node
         */
        if (first == last)
        {
            points.push_back(std::addressof(storage_[first]));
            return leaf;
        }

        // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

        // root: (0 + 9) / 2 = 4
        //       left=(0, 3) --- right=(5,9)

        // left recurse: (0 + 3) / 2 = 1
        //               left=(0,0) --- right=(2,3)

        // right recurse: (5 + 9) / 2 = 7
        //                left=(5, 6) ---- right=(8,9)

        // left-left recurse: end condition

        // left-right recurse: (2 + 3) / 2 = 2
        //                     left=null --- right=(3,3)

        // right-left recurse: (5 + 6) / 2 = 5
        //                     left=null --- right=(6,6)

        // right-right recurse: (8 + 9) / 2 = 8
        //                      left=null --- right=(9,9)

        auto median = (first + last) / 2u;
        auto node   = std::make_unique<node_type>();
        points.push_back(std::addressof(storage_[median]));

        auto left_child  = construct_exact_median_recursive(first, median - 1u, current_depth + 1u);
        auto right_child = construct_exact_median_recursive(median + 1u, last, current_depth + 1u);
        node->set_left(std::move(left_child));
        node->set_right(std::move(right_child));
    }

    void construct_approximate_median(std::vector<pcp::point_t>& storage) {}

  private:
    std::size_t max_depth_;
    std::vector<element_type> storage_;
    node_type_ptr root_;
};

using linked_kdtree_t = basic_linked_kdtree_t;

} // namespace pcp

#endif // PCP_basic_kdtree_t_HPP
