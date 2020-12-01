#pragma once

#include <array>
#include <pcp/traits/point_traits.hpp>

namespace pcp {
namespace common {

/**
 * This class is a discretization of continuous 3d space into voxels, which are the smallest volume
 * in the regular grid, in the form of cubes. The grid is represented as:
 *
 * - a point (x,y,z)
 * - voxel dimensions in every direction (dx,dy,dz)
 * - the number of voxels in every direction (sx,sy,sz)
 *
 * We consider that the grid's dimensions in 3d space form a rectangular prism with
 * lower-left-frontmost corner (x,y,z)
 * upper-right-deepest corner  (x + sx*dx, y + sy*dy, z + sz*dz)
 *
 * As such, the grid is only a subset of 3d space and corresponds to the axis-aligned bounding box
 * delimited by:
 * lower-left-frontmost corner (x,y,z)
 * upper-right-deepest corner  (x + sx*dx, y + sy*dy, z + sz*dz)
 *
 * The grid considers that space is divided into small cubes of dimensions (dx,dy,dz) called
 * voxels, all stacked next to each other in the x,y,z directions.
 *
 * Many scalar functions can be approximated continuously everywhere in the voxel grid
 * using trilinear interpolation by having the values of the scalar function at every
 * voxel's corners.
 */
template <class Scalar>
struct regular_grid3d_t
{
    using scalar_type = Scalar;

    // origin of the grid
    scalar_type x = static_cast<scalar_type>(0), y = static_cast<scalar_type>(0),
                z = static_cast<scalar_type>(0);
    // voxel size in x,y,z directions
    scalar_type dx = static_cast<scalar_type>(0), dy = static_cast<scalar_type>(0),
                dz = static_cast<scalar_type>(0);
    // number of voxels in x,y,z directions starting from the point (x,y,z)
    std::size_t sx = 0, sy = 0, sz = 0;
};

/**
 * @brief
 * Computes a regular 3d grid containing min and max.
 * The grid is discretized by the specified dimensions.
 * @tparam Scalar Arithmetic type of the x,y or z coordinates of the grid
 * @tparam PointView Type satisfying PointView
 * @param min The minimum point to be contained
 * @param max The maximum point to be contained
 * @param dimensions The number of cells along the x,y,z dimensions. dimensions[0] is the number of
 * cells along x axis, dimensions[1] is the number of cells along y axis, dimensions[2] is the
 * number of cells along z axis
 * @return A regular grid padded to assuredly contain min and max
 */
template <class PointView, class Scalar = float>
regular_grid3d_t<Scalar> regular_grid_containing(
    PointView const& min,
    PointView const& max,
    std::array<std::size_t, 3> dimensions)
{
    regular_grid3d_t<Scalar> grid;

    grid.x  = min.x();
    grid.y  = min.y();
    grid.z  = min.z();
    grid.sx = dimensions[0];
    grid.sy = dimensions[1];
    grid.sz = dimensions[2];
    grid.dx = (max.x() - min.x()) / static_cast<Scalar>(grid.sx);
    grid.dy = (max.y() - min.y()) / static_cast<Scalar>(grid.sy);
    grid.dz = (max.z() - min.z()) / static_cast<Scalar>(grid.sz);

    grid.x -= grid.dx;
    grid.y -= grid.dx;
    grid.z -= grid.dx;
    grid.sx += 2;
    grid.sy += 2;
    grid.sz += 2;

    return grid;
}

} // namespace common
} // namespace pcp