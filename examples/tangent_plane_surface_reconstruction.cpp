#include <atomic>
#include <future>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/writePLY.h>
#include <iostream>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/algorithm/estimate_tangent_planes.hpp>
#include <pcp/algorithm/surface_nets.hpp>
#include <pcp/common/points/vertex.hpp>
#include <pcp/common/timer.hpp>
#include <pcp/common/vector3d.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>
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

    menu.callback_draw_custom_window = [&]() {
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
            static ImU64 const step = 1;

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

                oss << "vertices: " << vertices.size() << "\n"
                    << "triangles: " << triangles.size() << "\n";

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

    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();
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
    timer.register_op("setup octree");
    timer.start();
    std::vector<vertex_type> vertices;
    vertices.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
        vertices.push_back(vertex_type{&points[i], i});

    pcp::basic_octree_t<vertex_type> octree{std::cbegin(vertices), std::cend(vertices)};
    progress_forward();
    timer.stop();

    auto const point_knn = [=, &octree](vertex_type const& v) {
        auto const& neighbours = octree.nearest_neighbours(v, k);
        return std::vector<point_type>(neighbours.cbegin(), neighbours.cend());
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
            point_knn,
            pcp::algorithm::default_plane_transform<point_type, plane_type>);
    }
    else
    {
        pcp::algorithm::estimate_tangent_planes(
            std::execution::seq,
            vertices.cbegin(),
            vertices.cend(),
            tangent_planes.begin(),
            point_knn,
            pcp::algorithm::default_plane_transform<point_type, plane_type>);
    }
    progress_forward();
    timer.stop();

    auto const vertex_knn = [=, &octree](vertex_type const& v) {
        return octree.nearest_neighbours(v, k);
    };
    auto const get_point = [&vertices](vertex_type const& v) {
        return pcp::point_t{vertices[v.id()]};
    };
    auto const get_normal = [&tangent_planes](vertex_type const& v) {
        return tangent_planes[v.id()].normal();
    };
    auto const transform_op = [&tangent_planes](vertex_type const& v, pcp::normal_t const& n) {
        tangent_planes[v.id()].normal(n);
    };

    timer.register_op("propagate normal orientations");
    timer.start();
    pcp::algorithm::propagate_normal_orientations(
        vertices.begin(),
        vertices.end(),
        vertex_knn,
        get_point,
        get_normal,
        transform_op);
    progress_forward();
    timer.stop();

    auto const signed_distance_function = [=, &octree, &tangent_planes](float x, float y, float z) {
        point_type const p{x, y, z};
        auto const nearest_neighbours = octree.nearest_neighbours(p, 1u);
        auto const& nearest_vertex    = nearest_neighbours.front();
        auto const idx                = nearest_vertex.id();
        auto const& tangent_plane     = tangent_planes[idx];
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
    auto const grid = pcp::common::regular_grid_containing(
        octree.voxel_grid().min,
        octree.voxel_grid().max,
        {dimx, dimy, dimz});

    auto const mesh = [&points, &octree, &k](auto const& sdf, auto const& grid, algorithm algo) {
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
                [&octree, &k](pcp::point_t const& p1, pcp::point_t const& p2) {
                    auto const& neighbours1 = octree.nearest_neighbours(p1, k);
                    float const sum1        = std::accumulate(
                        neighbours1.cbegin(),
                        neighbours1.cend(),
                        0.f,
                        [&p1](float val, pcp::point_view_t const& neighbour) {
                            auto const distance = pcp::common::norm(pcp::point_t(neighbour) - p1);
                            return val + distance;
                        });

                    auto const mean_distance1 = sum1 / static_cast<float>(neighbours1.size());

                    auto const& neighbours2 = octree.nearest_neighbours(p2, k);
                    float const sum2        = std::accumulate(
                        neighbours2.cbegin(),
                        neighbours2.cend(),
                        0.f,
                        [&p2](float val, pcp::point_view_t const& neighbour) {
                            auto const distance = pcp::common::norm(pcp::point_t(neighbour) - p2);
                            return val + distance;
                        });

                    auto const mean_distance2 = sum2 / static_cast<float>(neighbours2.size());

                    return mean_distance1 < mean_distance2;
                });

            auto const kneighbours = octree.nearest_neighbours(*max_density_point, k);
            auto const hint        = std::accumulate(
                                  kneighbours.cbegin(),
                                  kneighbours.cend(),
                                  pcp::point_t{0.f, 0.f, 0.f},
                                  [](pcp::point_t const& val, pcp::point_view_t const& p) {
                                      return val + pcp::point_t{p.x(), p.y(), p.z()};
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
