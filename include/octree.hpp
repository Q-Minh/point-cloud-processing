#pragma once

#include "octree_node.hpp"

namespace detail {

template <class T>
struct has_contains
{
private:
	using yes = std::true_type;
	using no = std::false_type;

	template <class U> static auto test(int) -> decltype(std::declval<U>().contains(std::declval<point_t>()), yes());
	template <class> static no test(...);
public:
	static constexpr bool value = std::is_same_v<decltype(test<T>(0)), yes>;
};

template <class Range1, class Range2>
struct can_intersect
{
private:
	using yes = std::true_type;
	using no = std::false_type;

	template <class R1, class R2> static auto test(int) -> decltype(
		intersections::intersects(std::declval<R1>(), std::declval<R2>()), yes()
		);

	template <class> static no test(...);
public:
	static constexpr bool value = std::is_same_v<decltype(test<Range1, Range2>(0)), yes>;
};

} // detail

/*
* An octree is a tree data structure for 3-dimensional quantities which
* recursively subdivides a regular grid into its 8 octants by having
* 8 child octrees. Octrees, much like binary trees, offer efficient
* searching and insertion of those 3-d quantities (logarithmic time).
* Interesting applications of octrees include efficient collision detection,
* k-nearest-neighbor searches and range queries.
*/
class octree_t
{
public:
	explicit octree_t(octree_parameters_t const& params)
		:
		root_(params),
		size_(0u)
	{}

	template <class ForwardIter>
	explicit octree_t(
		ForwardIter begin, ForwardIter end,
		octree_parameters_t const& params
	)	:
		root_(params),
		size_(root_.insert(begin, end))
	{}

	template <class ForwardIter>
	std::size_t insert(ForwardIter begin, ForwardIter end)
	{
		static_assert(
			std::is_same_v<
				std::iterator_traits<ForwardIter>::value_type,
				point_t
			>,
			"Iterators must have value_type of point_t"
			);

		auto const inserted = root_.insert(begin, end);
		size_ += inserted;
		return inserted; 
	}

	bool insert(point_t const& p) 
	{ 
		bool const inserted = root_.insert(p);
		if (inserted)
			++size_;

		return inserted; 
	}

	/*
	* Returns the k-nearest-neighbours in 3d Euclidean space
	* using the l2-norm as the notion of distance.
	*
	* @param k         The number of neighbors to return that are nearest to the specified point for all points of the octree
	* @param reference The reference point for which we want the k nearest neighbors
	* @return A list of nearest points ordered from nearest to furthest of size s where 0 <= s <= k
	*/
	std::vector<point_t> nearest_neighbours(point_t const& reference, std::size_t k) const
	{
		return root_.nearest_neighbours(reference, k);
	}

	/*
	* Returns all points that reside in the given range.
	* 
	* @param range A range satisfying the Range type requirements
	* @return A list of all points that reside in the given range
	*/
	template <class Range>
	std::vector<point_t> range_search(Range const& range) const
	{
		static_assert(
			detail::has_contains<Range>::value && detail::can_intersect<Range, axis_aligned_bounding_box_t>::value,
			"template type argument to parameter Range must have member function "
			"bool contains(point_t const& p) and be overloaded in function "
			"bool intersects(Range const& range, axis_aligned_bounding_box_t const& aabb)"
			);

		std::vector<point_t> points_in_range;
		root_.range_search(range, points_in_range);
		return points_in_range;
	}

	std::size_t size() const { return size_; }

private:
	octree_node_t root_;
	std::size_t   size_;
};
