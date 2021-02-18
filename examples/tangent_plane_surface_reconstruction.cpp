#include <atomic>
#include <future>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/writePLY.h>
#include <iostream>
#include <pcp/pcp.hpp>
#include <range/v3/view/transform.hpp>
#include <sstream>

using point_type  = pcp::point_t;
using normal_type = pcp::normal_t;
using vertex_type = pcp::vertex_t;
using plane_type  = pcp::common::plane3d_t;

Eigen::MatrixXd from_point_cloud(std::vector<pcp::point_t> const& points);

std::pair<Eigen::MatrixXd, Eigen::MatrixXi> to_mesh(
    std::vector<pcp::point_t> const& points,
    std::vector<pcp::common::shared_vertex_mesh_triangle<std::uint32_t>> const& triangles);

auto reconstruct_surface_from_point_cloud(
    std::vector<pcp::point_t>& points,
    std::size_t k,
    std::array<std::size_t, 3> dims,
    int variant,
    std::atomic<float>& progress)
    -> std::tuple<
        std::vector<pcp::point_t>,
        std::vector<pcp::common::shared_vertex_mesh_triangle<std::uint32_t>>,
        pcp::common::basic_timer_t>;

int main(int argc, char** argv)
{
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_window = [&]() {
        ImGui::Begin("Point Cloud Processing");
        if (ImGui::CollapsingHeader(
                "Tangent Plane Surface Reconstruction",
                ImGuiTreeNodeFlags_DefaultOpen))
        {
            static std::vector<pcp::point_t> points;
            static std::vector<pcp::point_t> vertices;
            static std::vector<pcp::common::shared_vertex_mesh_triangle<std::uint32_t>> triangles;
            static std::atomic<float> recon_progress = 0.f;
            using future_type                        = std::future<std::tuple<
                std::vector<pcp::point_t>,
                std::vector<pcp::common::shared_vertex_mesh_triangle<std::uint32_t>>,
                pcp::common::basic_timer_t>>;
            static future_type recon_handle;
            static std::string execution_report;

            ImGui::TreePush();
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;
            if (ImGui::Button("Load##PointCloud", ImVec2((w - p) / 2.f, 0)) &&
                !recon_handle.valid())
            {
                std::string const filename = igl::file_dialog_open();
                std::filesystem::path ply_point_cloud{filename};
                auto [p, _]  = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
                points       = std::move(p);
                auto const V = from_point_cloud(points);
                viewer.data().clear();
                viewer.data().add_points(V, Eigen::RowVector3d(1.0, 1.0, 0.0));
                viewer.data().point_size = 1.f;
                viewer.core().align_camera_center(V);
            }
            ImGui::SameLine();
            if (ImGui::Button("Save##PointCloud", ImVec2((w - p) / 2.f, 0.f)) &&
                !recon_handle.valid())
            {
                std::filesystem::path ply_mesh = igl::file_dialog_save();
                pcp::io::write_ply(
                    ply_mesh,
                    vertices,
                    triangles,
                    pcp::io::ply_format_t::binary_little_endian);
            }

            static std::uint64_t k       = 10u;
            static int algorithm_variant = 0;
            static std::size_t dimx      = 20u;
            static std::size_t dimy      = 20u;
            static std::size_t dimz      = 20u;
            static ImU64 const step      = 1;

            if (ImGui::CollapsingHeader("Display##PointCloud", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Button("Point Cloud", ImVec2((w - p) / 2.f, 0)) && !recon_handle.valid())
                {
                    auto const V = from_point_cloud(points);
                    viewer.data().clear();
                    viewer.data().add_points(V, Eigen::RowVector3d(1.0, 1.0, 0.0));
                    viewer.data().point_size = 1.f;
                    viewer.core().align_camera_center(V);
                }
                ImGui::SameLine();
                if (ImGui::Button("Mesh", ImVec2((w - p) / 2.f, 0)) && !recon_handle.valid())
                {
                    viewer.data().clear();
                    auto [V, F] = to_mesh(vertices, triangles);
                    viewer.data().set_mesh(V, F);
                    viewer.core().align_camera_center(V);
                }
                std::string const points_str =
                    std::string("Points: ") + std::to_string(points.size());
                ImGui::Text(points_str.c_str());
                std::string const vertices_str =
                    std::string("Vertices: ") + std::to_string(vertices.size());
                ImGui::Text(vertices_str.c_str());
                std::string const triangles_str =
                    std::string("Triangles: ") + std::to_string(triangles.size());
                ImGui::Text(triangles_str.c_str());
            }

            if (ImGui::CollapsingHeader("k neighborhood", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::InputScalar("size", ImGuiDataType_U64, &k);
            }
            if (ImGui::CollapsingHeader("Regular Grid Dimensions", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::InputScalar("X", ImGuiDataType_U64, &dimx, &step);
                ImGui::InputScalar("Y", ImGuiDataType_U64, &dimy, &step);
                ImGui::InputScalar("Z", ImGuiDataType_U64, &dimz, &step);
            }
            if (ImGui::CollapsingHeader("Execution Variant", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::RadioButton("sequential", &algorithm_variant, 0);
                ImGui::RadioButton("parallel", &algorithm_variant, 1);
                ImGui::RadioButton("graph", &algorithm_variant, 2);
            }

            if (ImGui::Button("Reset", ImVec2((w - p) / 2.f, 0)) && !recon_handle.valid())
            {
                points.clear();
                vertices.clear();
                triangles.clear();
                viewer.data().clear();
            }
            if (ImGui::Button("Execute", ImVec2((w - p) / 2.f, 0)) && !recon_handle.valid())
            {
                recon_handle = std::async(std::launch::async, [&]() {
                    return reconstruct_surface_from_point_cloud(
                        points,
                        std::size_t{k},
                        {dimx, dimy, dimz},
                        int{algorithm_variant},
                        recon_progress);
                });
            }

            ImGui::ProgressBar(recon_progress, ImVec2(0.f, 0.f));

            if (recon_handle.valid() &&
                recon_handle.wait_for(std::chrono::microseconds(0u)) == std::future_status::ready)
            {
                auto triple = recon_handle.get();
                vertices    = std::get<0>(triple);
                triangles   = std::get<1>(triple);
                auto timer  = std::get<2>(triple);

                std::ostringstream oss{};
                oss << "Execution report:\n";
                for (auto const [operation, duration] : timer.ops)
                {
                    oss << operation << ": "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
                        << " ms"
                        << "\n";
                }

                execution_report = oss.str();

                viewer.data().clear();
                auto [V, F] = to_mesh(vertices, triangles);
                viewer.data().set_mesh(V, F);
                viewer.core().align_camera_center(V);
            }

            if (!execution_report.empty())
            {
                ImGui::Text(execution_report.c_str());
            }

            ImGui::TreePop();
        }
        ImGui::End();
    };

    viewer.core().set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);
    viewer.data().show_lines = false;
    viewer.launch();

    return 0;
}

Eigen::MatrixXd from_point_cloud(std::vector<pcp::point_t> const& points)
{
    Eigen::MatrixXd P;
    P.resize(points.size(), 3u);
    for (std::size_t i = 0u; i < points.size(); ++i)
    {
        P(i, 0) = points[i].x();
        P(i, 1) = points[i].y();
        P(i, 2) = points[i].z();
    }
    return P;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXi> to_mesh(
    std::vector<pcp::point_t> const& points,
    std::vector<pcp::common::shared_vertex_mesh_triangle<std::uint32_t>> const& triangles)
{
    Eigen::MatrixXi F;
    F.resize(triangles.size(), 3u);
    for (std::size_t i = 0u; i < triangles.size(); ++i)
    {
        auto const& indices = triangles[i].indices();
        F(i, 0)             = indices[0];
        F(i, 1)             = indices[1];
        F(i, 2)             = indices[2];
    }

    Eigen::MatrixXd V = from_point_cloud(points);
    return {V, F};
}

auto reconstruct_surface_from_point_cloud(
    std::vector<pcp::point_t>& points,
    std::size_t k,
    std::array<std::size_t, 3> dims,
    int variant,
    std::atomic<float>& progress)
    -> std::tuple<
        std::vector<pcp::point_t>,
        std::vector<pcp::common::shared_vertex_mesh_triangle<std::uint32_t>>,
        pcp::common::basic_timer_t>
{
    enum algorithm { sequential = 0, parallel = 1, graph = 2 };

    if (points.empty())
        return {};

    progress                           = 0.f;
    float constexpr num_ops            = 4.f;
    float constexpr progress_increment = 1.f / 4.f;
    auto const progress_forward        = [&]() {
        progress = progress + progress_increment;
    };

    pcp::common::basic_timer_t timer;
    timer.register_op("setup kdtree");
    timer.start();
    using vertex_type = std::size_t;
    std::vector<vertex_type> vertices;
    vertices.resize(points.size());
    std::iota(vertices.begin(), vertices.end(), 0u);

    auto const point_map = [&](vertex_type const& v) {
        return points[v];
    };

    auto const coordinate_map = [&](vertex_type const& v) {
        return std::array<float, 3u>{points[v].x(), points[v].y(), points[v].z()};
    };

    pcp::kdtree::construction_params_t params;
    params.compute_max_depth = true;

    pcp::basic_linked_kdtree_t<vertex_type, 3u, decltype(coordinate_map)> kdtree{
        vertices.begin(),
        vertices.end(),
        coordinate_map,
        params};
    // pcp::basic_linked_octree_t<vertex_type> octree{
    //    std::cbegin(vertices),
    //    std::cend(vertices),
    //    point_map};
    progress_forward();
    timer.stop();

    auto const knn_map = [=, &kdtree, &point_map](vertex_type const& v) {
        auto target = point_map(v);
        // auto const& neighbours = octree.nearest_neighbours(target, k, point_map);
        auto const& neighbours = kdtree.nearest_neighbours(v, k);
        return neighbours;
    };

    std::vector<plane_type> tangent_planes;
    tangent_planes.resize(points.size());

    timer.register_op("estimate tangent planes");
    timer.start();
    if (variant != sequential)
    {
        pcp::algorithm::estimate_tangent_planes(
            std::execution::par,
            vertices.cbegin(),
            vertices.cend(),
            tangent_planes.begin(),
            point_map,
            knn_map,
            pcp::algorithm::default_plane_transform<vertex_type, plane_type>);
    }
    else
    {
        pcp::algorithm::estimate_tangent_planes(
            std::execution::seq,
            vertices.cbegin(),
            vertices.cend(),
            tangent_planes.begin(),
            point_map,
            knn_map,
            pcp::algorithm::default_plane_transform<vertex_type, plane_type>);
    }
    progress_forward();
    timer.stop();

    auto const vertex_knn = [=, &kdtree, &point_map](vertex_type const& v) {
        auto target = point_map(v);
        // return octree.nearest_neighbours(target, k, point_map);
        return kdtree.nearest_neighbours(v, k);
    };
    auto const normal_map = [&tangent_planes](vertex_type const& v) {
        return tangent_planes[v].normal();
    };
    auto const transform_op = [&tangent_planes](vertex_type const& v, pcp::normal_t const& n) {
        tangent_planes[v].normal(n);
    };

    timer.register_op("propagate normal orientations");
    timer.start();
    auto const index_map = [](vertex_type const& v) {
        return v;
    };

    pcp::algorithm::propagate_normal_orientations(
        vertices.begin(),
        vertices.end(),
        index_map,
        vertex_knn,
        point_map,
        normal_map,
        transform_op);
    progress_forward();
    timer.stop();

    auto const signed_distance_function =
        [=, &kdtree, &tangent_planes, &point_map](float x, float y, float z) {
            point_type const p{x, y, z};
            // auto const nearest_neighbours = octree.nearest_neighbours(p, 1u, point_map);
            auto const nearest_neighbours = kdtree.nearest_neighbours({p.x(), p.y(), p.z()}, 1u);
            auto const& nearest_vertex    = nearest_neighbours.front();
            auto const& tangent_plane     = tangent_planes[nearest_vertex];
            auto const o                  = tangent_plane.point();
            auto const n                  = tangent_plane.normal();
            auto const op                 = p - o;
            auto const f                  = pcp::common::inner_product(op, n);
            return f;
        };

    timer.register_op("surface nets");
    timer.start();
    auto const dimx = dims[0];
    auto const dimy = dims[1];
    auto const dimz = dims[2];
    // auto const grid = pcp::common::regular_grid_containing(
    //    octree.voxel_grid().min,
    //    octree.voxel_grid().max,
    //    {dimx, dimy, dimz});
    auto const& aabb = kdtree.aabb();
    auto const grid  = pcp::common::regular_grid_containing(
        pcp::point_t{aabb.min[0], aabb.min[1], aabb.min[2]},
        pcp::point_t{aabb.max[0], aabb.max[1], aabb.max[2]},
        {dimx, dimy, dimz});

    auto const mesh = [=, &kdtree, &point_map](auto const& sdf, auto const& grid, algorithm algo) {
        if (algo == parallel)
        {
            return pcp::algorithm::isosurface::surface_nets(std::execution::par, sdf, grid);
        }
        else if (algo == sequential)
        {
            return pcp::algorithm::isosurface::surface_nets(std::execution::seq, sdf, grid);
        }
        else /* if (algo == graph) */
        {
            auto max_density_point = std::min_element(
                std::execution::par,
                points.cbegin(),
                points.cend(),
                [=, &kdtree, &point_map](pcp::point_t const& p1, pcp::point_t const& p2) {
                    // auto const& neighbours1 = octree.nearest_neighbours(p1, k, point_map);
                    auto const& neighbours1 =
                        kdtree.nearest_neighbours({p1.x(), p1.y(), p1.z()}, k);
                    float const sum1 = std::accumulate(
                        neighbours1.cbegin(),
                        neighbours1.cend(),
                        0.f,
                        [&](float val, vertex_type const& neighbour) {
                            auto const distance = pcp::common::norm(point_map(neighbour) - p1);
                            return val + distance;
                        });

                    auto const mean_distance1 = sum1 / static_cast<float>(neighbours1.size());

                    // auto const& neighbours2 = octree.nearest_neighbours(p2, k, point_map);
                    auto const& neighbours2 =
                        kdtree.nearest_neighbours({p2.x(), p2.y(), p2.z()}, k);
                    float const sum2 = std::accumulate(
                        neighbours2.cbegin(),
                        neighbours2.cend(),
                        0.f,
                        [&](float val, vertex_type const& neighbour) {
                            auto const distance = pcp::common::norm(point_map(neighbour) - p2);
                            return val + distance;
                        });

                    auto const mean_distance2 = sum2 / static_cast<float>(neighbours2.size());

                    return mean_distance1 < mean_distance2;
                });

            // auto const kneighbours = octree.nearest_neighbours(*max_density_point, k, point_map);
            auto const kneighbours = kdtree.nearest_neighbours(
                {max_density_point->x(), max_density_point->y(), max_density_point->z()},
                k);
            auto const hint = std::accumulate(
                                  kneighbours.cbegin(),
                                  kneighbours.cend(),
                                  pcp::point_t{0.f, 0.f, 0.f},
                                  [&](pcp::point_t const& val, vertex_type const& v) {
                                      auto p = point_map(v);
                                      return val + p;
                                  }) /
                              static_cast<float>(kneighbours.size());

            return pcp::algorithm::isosurface::surface_nets(std::execution::par, sdf, grid, hint);
        }
    };
    auto const [mesh_vertices, mesh_triangles] =
        mesh(signed_distance_function, grid, static_cast<algorithm>(variant));
    progress_forward();
    timer.stop();

    return {mesh_vertices, mesh_triangles, timer};
}
