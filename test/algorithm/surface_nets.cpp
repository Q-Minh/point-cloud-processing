#include <catch2/catch.hpp>
#include <pcp/algorithm/surface_nets.hpp>
#include <pcp/common/axis_aligned_bounding_box.hpp>

SCENARIO("isosurface extraction with surface nets", "[isosurface]")
{
    GIVEN("a 3d implicit function")
    {
        auto const unit_circle = [](float x, float y, float z) {
            return std::sqrtf(x * x + y * y + z * z) - 1.f;
        };

        GIVEN("a 3d regular grid domain")
        {
            auto const grid = pcp::common::regular_grid_containing(
                pcp::point_t{-1.f, -1.f, -1.f},
                pcp::point_t{1.f, 1.f, 1.f},
                {5, 5, 5});

            // TODO: 
            // Add a test scenario where we really look at the generated triangles and 
            // vertices and see if they are where they are supposed to be. It should not 
            // take too long to do by hand with a small grid (3x3 or 5x5).
            auto const check_mesh_validity = [=](auto const& vertices, auto const& triangles) {
                bool const are_triangles_valid =
                    std::none_of(triangles.cbegin(), triangles.cend(), [=](auto const& triangle) {
                        auto const n        = vertices.size();
                        auto const& indices = triangle.indices();
                        return indices[0] < 0 || indices[0] >= n || indices[1] < 0 ||
                               indices[1] >= n || indices[2] < 0 || indices[2] >= n;
                    });
                REQUIRE(are_triangles_valid);
                bool const are_vertices_in_domain =
                    std::all_of(vertices.cbegin(), vertices.cend(), [=](auto const& v) {
                        pcp::point_t const min{grid.x, grid.y, grid.z};
                        pcp::point_t const max{
                            grid.x + grid.sx * grid.dx,
                            grid.x + grid.sx * grid.dx,
                            grid.x + grid.sx * grid.dx};
                        pcp::axis_aligned_bounding_box_t<pcp::point_t> aabb{min, max};
                        return aabb.contains(v);
                    });
                REQUIRE(are_vertices_in_domain);
            };

            WHEN("extracting the level set f(x,y,z)=0 as a 3d triangle mesh sequentially")
            {
                auto const [vertices, triangles] = pcp::algorithm::isosurface::surface_nets(
                    std::execution::seq,
                    unit_circle,
                    grid);

                check_mesh_validity(vertices, triangles);
            }
            WHEN("extracting the level set f(x,y,z)=0 as a 3d triangle mesh in parallel")
            {
                auto const [vertices, triangles] = pcp::algorithm::isosurface::surface_nets(
                    std::execution::par,
                    unit_circle,
                    grid);

                check_mesh_validity(vertices, triangles);
            }
        }
    }
}
