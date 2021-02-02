#ifndef PCP_basic_kdtree_t_HPP
#define PCP_basic_kdtree_t_HPP

#include "pcp/traits/range_traits.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <random>
#include <tuple>
#include <vector>
#include "pcp/common/points/point.hpp"
//#include <iostream>

namespace pcp {

// k-d tree implementation
//

template <typename type>
class basic_kdtree_t
{

  public:
    using point_type = pcp::basic_point_t<type>;

  private:
    struct node
    {
        std::vector<point_type> points_;
        node* left_;
        node* right_;

        node(const point_type& pt)
        {
            insert_point(pt);
            left_  = nullptr;
            right_ = nullptr;
        }
        node()
        {
            left_  = nullptr;
            right_ = nullptr;
        }
        size_t get_coordinate(size_t index_coordinate, size_t index_point)
        {
            // intermediate node or leaf node with only a single point => index_coordinate = 0
            if (index_coordinate == 0)
                return points_[index_point].x;
            else if (index_coordinate == 2)
                return points_[index_point].y;
            else
                return points_[index_point].z;
        }
        
        void insert_point(const point_type& p) { points_.push_back(p);}
    };


    node* root_;
    size_t size_;
    std::vector<point_type> points_;
    struct comparator
    {
        size_t index;
        comparator(size_t index) : index(index) {}
        bool operator()(const point_type& p1, const point_type& p2) const
        {
            if (index== 0)
                return (p2.x() > p1.x());
            else if (index == 2)
                return (p2.y() > p1.y());
            else
                return (p2.z() > p1.z());


        }
    };

  struct node* new_node(const point_type pt)
    {
        struct node* temp = new node;
        temp->insert_point(pt);
        temp->left_ = temp->right_ = NULL;
        return temp;
    } 
 

    node* create_tree(node* root, size_t begin, size_t end, size_t index, size_t fixed_depth, size_t depth)
    {
        if (end <= begin)
            return nullptr;
        //we will add all the remaining points to a node,
        else if (depth == fixed_depth-1)
        {
            size_++;
            if (root == nullptr)
                root = new_node(points_[begin]);
            else
            {
           
                root->insert_point(points_[begin]);
            }
            for (size_t i = begin+1; i < end; i++)
            {   
                root->insert_point(points_[i]);
            }
            return root;
        } 
        else
        {
            size_++;
            size_t n = begin + (end - begin) / 2;
            std::nth_element(&points_[begin], &points_[n], &points_[end], comparator(index));
            index   = (index + 1) % 3;
            if (root == nullptr)
                root = new_node(points_[n]);
            else
            {
                root->insert_point(points_[n]);
            }
           root->left_ =  create_tree(root->left_,begin, n, index, fixed_depth, depth+1);
           root->right_ = create_tree(root->right_,n + 1, end, index,fixed_depth,depth+1);
            return root;
        }

    }
 


  

 
  public:

    basic_kdtree_t(const basic_kdtree_t&) = delete;
    basic_kdtree_t& operator=(const basic_kdtree_t&) = delete;


    /**
     * Constructor taking a pair of iterators. Adds each
     * point in the range [begin, end) to the tree.
     *
     * @param begin start of range
     * @param end end of range
     */
    template <typename iterator>
    basic_kdtree_t(iterator begin, iterator end,size_t fixed_depth)
    {
        size_ = 0;
        points_.reserve(std::distance(begin, end));
        for (auto i = begin; i != end; ++i)
            points_.emplace_back(*i);
        root_ = new node();
        create_tree(root_,0, points_.size()-1, 0, fixed_depth, 0);
    }
    /* Given a binary tree, print its nodes according to the
   "bottom-up" postorder traversal. */
    void print_in_order(struct node* node)
    {
        if (node == NULL)
            return;

        // first recur on left subtree
        print_in_order(node->left_);

        for (int i = 0; i < node->points_.size(); i++)
        {
            std::cout << " x : " << node->points_[i].x() << "  y: " << node->points_[i].y() << " z "
                      << node->points_[i].z() << "\n";
            ;
        }

        // then recur on right subtree
        print_in_order(node->right_);
    }
    void print_in_order() { print_in_order(root_); }

    void nearest_n(node* root, const point_type& point, size_t index)
    {
       

    }


    /**
     * Returns true if the tree is empty, false otherwise.
     */
    bool empty() const { return points_.empty(); }


    size_t const& size() { return size_; }



};


} // namespace pcp
using kdtree_t = pcp::basic_kdtree_t<float> ;

#endif // PCP_basic_kdtree_t_HPP
