#include <array>
#include <future>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <pcp/algorithm/algorithm.hpp>
#include <pcp/common/timer.hpp>
#include <pcp/io/io.hpp>
#include <pcp/kdtree/kdtree.hpp>

int main(int argc, char** argv)
{
    igl::opengl::glfw::Viewer viewer{};
    igl::opengl::glfw::imgui::ImGuiMenu menu{};
    viewer.plugins.push_back(&menu);

    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;

    std::vector<point_type> input_point_cloud;
    std::vector<normal_type> input_normals;
    std::vector<std::size_t> indices;
    std::vector<point_type> output_point_cloud;
    std::vector<normal_type> output_normals;

    auto const input_point_map = [&](std::size_t const i) {
        return input_point_cloud[i];
    };
    auto const input_normal_map = [&](std::size_t const i) {
        return input_normals[i];
    };
    auto const input_coordinate_map = [&](std::size_t const i) {
        auto const& p = input_point_cloud[i];
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    Eigen::RowVector3d const yellow{1., 1., 0.};
    Eigen::RowVector3d const green{0., 1., 0.};

    auto const from_point_cloud = [](std::vector<point_type> const& points) -> Eigen::MatrixXd {
        Eigen::MatrixXd P;
        P.resize(points.size(), 3u);
        for (std::size_t i = 0u; i < points.size(); ++i)
        {
            P(i, 0) = points[i].x();
            P(i, 1) = points[i].y();
            P(i, 2) = points[i].z();
        }
        return P;
    };

    auto const draw_point_cloud = [&](std::vector<point_type> const& points) {
        Eigen::MatrixXd const V = from_point_cloud(points);
        viewer.data().clear_points();
        viewer.data().add_points(V, yellow);
        viewer.core().align_camera_center(V);
    };

    auto const draw_normals = [&](std::vector<point_type> const& points,
                                  std::vector<normal_type> const& normals,
                                  float const scale) {
        if (normals.empty())
            return;
        if (normals.size() != points.size())
            return;

        viewer.data().clear_edges();
        for (std::size_t i = 0u; i < points.size(); ++i)
        {
            auto const& p = points[i];
            auto const& n = normals[i];
            Eigen::RowVector3d const p1{p.x(), p.y(), p.z()};
            Eigen::RowVector3d const dir{n.nx(), n.ny(), n.nz()};
            Eigen::RowVector3d const p2 = p1 + static_cast<double>(scale) * dir;
            viewer.data().add_edges(p1, p2, green);
        }
    };

    auto const resize_output_point_cloud = [&]() {
        std::size_t const output_size = static_cast<std::size_t>(input_point_cloud.size());
        output_point_cloud.clear();
        output_point_cloud.resize(output_size);
        output_normals.clear();
        output_normals.resize(output_size);
    };
    auto const get_average_spacing = [&](std::size_t const k) {
        pcp::kdtree::construction_params_t kdtree_params{};
        kdtree_params.compute_max_depth                   = true;
        kdtree_params.min_element_count_for_parallel_exec = input_point_cloud.size();

        pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(input_coordinate_map)> kdtree{
            indices.begin(),
            indices.end(),
            input_coordinate_map,
            kdtree_params};

        auto const knn_map = [&kdtree, k](std::size_t const i) {
            return kdtree.nearest_neighbours(i, k);
        };

        float const avg_spacing = pcp::algorithm::average_distance_to_neighbors(
            indices.begin(),
            indices.end(),
            input_point_map,
            knn_map);

        return avg_spacing;
    };

    static std::future<void> execution_handle{};

    auto const is_downsampling_running = [&]() -> bool {
        return execution_handle.valid();
    };

    pcp::common::basic_timer_t timer;
    menu.callback_draw_viewer_window = [&]() {
        ImGui::Begin("Point Cloud Processing");

        static std::string progress_str{""};
        if (ImGui::CollapsingHeader("Info", ImGuiTreeNodeFlags_DefaultOpen))
        {
            std::string const input_num_points_str =
                "Input points: " + std::to_string(input_point_cloud.size());
            ImGui::BulletText(input_num_points_str.c_str());
            std::string const input_num_normals_str =
                "Input normals: " + std::to_string(input_normals.size());
            ImGui::BulletText(input_num_normals_str.c_str());
            std::string const output_num_points_str =
                "Output points: " + std::to_string(output_point_cloud.size());
            ImGui::BulletText(output_num_points_str.c_str());
            std::string const output_num_normals_str =
                "Output normals: " + std::to_string(output_normals.size());
            ImGui::BulletText(output_num_normals_str.c_str());

            ImGui::BulletText(progress_str.c_str());
        }

        static bool show_normals{false};
        static float normals_scale = .2f;
        static bool is_input_point_cloud{true};

        if (ImGui::CollapsingHeader("IO", ImGuiTreeNodeFlags_DefaultOpen) &&
            !is_downsampling_running())
        {
            float const w = ImGui::GetContentRegionAvailWidth();
            float const p = ImGui::GetStyle().FramePadding.x;

            if (ImGui::Button("Load##IO", ImVec2((w - p) / 2.f, 0.f)))
            {
                std::string const filename = igl::file_dialog_open();
                std::filesystem::path ply_point_cloud{filename};
                auto [p, n] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
                if (!p.empty())
                {
                    input_point_cloud.clear();
                    input_normals.clear();

                    input_point_cloud = std::move(p);
                    if (!n.empty())
                        input_normals = std::move(n);

                    indices.clear();
                    indices.resize(input_point_cloud.size());
                    std::iota(indices.begin(), indices.end(), 0u);

                    draw_point_cloud(input_point_cloud);
                    if (show_normals)
                        draw_normals(input_point_cloud, input_normals, normals_scale);

                    is_input_point_cloud = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Save##IO", ImVec2((w - p) / 2.f, 0.f)))
            {
                std::filesystem::path ply_mesh = igl::file_dialog_save();
                pcp::io::write_ply(
                    ply_mesh,
                    output_point_cloud,
                    output_normals,
                    pcp::io::ply_format_t::binary_little_endian);
            }
        }

        if (ImGui::CollapsingHeader("Normal Estimation", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float const w = ImGui::GetContentRegionAvailWidth();
            float const p = ImGui::GetStyle().FramePadding.x;

            static int k = 6u;
            ImGui::InputInt("K neighbours", &k);

            if (ImGui::Button("Estimate##NormalEstimation", ImVec2((w - p) / 2.f, 0.f)))
            {
                pcp::kdtree::construction_params_t kdtree_params{};
                kdtree_params.compute_max_depth                   = true;
                kdtree_params.min_element_count_for_parallel_exec = input_point_cloud.size();

                pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(input_coordinate_map)> kdtree{
                    indices.begin(),
                    indices.end(),
                    input_coordinate_map,
                    kdtree_params};

                auto const knn_map = [&kdtree, k = k](std::size_t const i) {
                    return kdtree.nearest_neighbours(i, k);
                };

                input_normals.resize(indices.size());
                pcp::algorithm::estimate_normals(
                    std::execution::par,
                    indices.begin(),
                    indices.end(),
                    input_normals.begin(),
                    input_point_map,
                    knn_map,
                    [](auto const, auto const n) { return n; });

                pcp::algorithm::propagate_normal_orientations(
                    indices.begin(),
                    indices.end(),
                    [](std::size_t const i) { return i; },
                    knn_map,
                    input_point_map,
                    input_normal_map,
                    [&](std::size_t const i, normal_type const& n) { input_normals[i] = n; });
            }
        }

        if (ImGui::CollapsingHeader("EAR", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float const w = ImGui::GetContentRegionAvailWidth();
            float const p = ImGui::GetStyle().FramePadding.x;

            static int K                                         = 3;
            static float normal_smoothing_radial_mean_multiplier = 1.f;
            static float sigman                                  = 0.261799f;

            static float wlop_radial_mean_multiplier = 1.f;
            static float mu                          = 0.45f;
            static bool uniform                      = true;

            static int kneighbours = 6;

            ImGui::InputInt("Resampling iterations", &K);
            ImGui::InputInt("K neighbours for average spacing computation", &kneighbours);
            if (ImGui::TreeNode("Normal Smoothing##EAR"))
            {
                ImGui::InputFloat(
                    "Normal smoothing radius multiplier",
                    &normal_smoothing_radial_mean_multiplier);
                ImGui::InputFloat("sigman", &sigman);
                static int num_normal_smoothing_iterations = 1;
                ImGui::InputInt("Iterations##NormalSmoothing", &num_normal_smoothing_iterations);

                if (ImGui::Button("Smooth normals##EAR", ImVec2((w - p) / 2.f, 0.f)) &&
                    !is_downsampling_running())
                {
                    progress_str     = "Executing ...";
                    execution_handle = std::async(std::launch::async, [&]() {
                        timer.register_op("normal smoothing");
                        timer.start();

                        resize_output_point_cloud();
                        float const avg_spacing = get_average_spacing(kneighbours);

                        pcp::algorithm::ear::normal_smoothing_params_t normal_smoothing_params{};
                        normal_smoothing_params.K =
                            static_cast<std::size_t>(num_normal_smoothing_iterations);
                        normal_smoothing_params.sigmap =
                            static_cast<double>(normal_smoothing_radial_mean_multiplier) *
                            avg_spacing;
                        normal_smoothing_params.sigman = sigman;

                        pcp::algorithm::ear::smooth_normals(
                            indices.begin(),
                            indices.end(),
                            output_normals.begin(),
                            input_point_map,
                            input_normal_map,
                            normal_smoothing_params);

                        output_point_cloud = input_point_cloud;

                        timer.stop();
                    });
                }

                ImGui::TreePop();
            }
            if (ImGui::TreeNode("WLOP##EAR"))
            {
                ImGui::InputFloat("WLOP radius multiplier", &wlop_radial_mean_multiplier);
                ImGui::InputFloat("Repulsion force coefficient", &mu);
                static int num_wlop_iterations = 1;
                ImGui::InputInt("Iterations##WLOP", &num_wlop_iterations);
                ImGui::Checkbox("Require uniform", &uniform);

                if (ImGui::Button("Project with WLOP##WLOP", ImVec2((w - p) / 2.f, 0.f)) &&
                    !is_downsampling_running())
                {
                    progress_str     = "Executing ...";
                    execution_handle = std::async(std::launch::async, [&]() {
                        timer.register_op("wlop");
                        timer.start();

                        resize_output_point_cloud();
                        float const avg_spacing = get_average_spacing(kneighbours);

                        pcp::algorithm::ear::wlop_params_t wlop_params{};
                        wlop_params.K = static_cast<std::size_t>(num_wlop_iterations);
                        wlop_params.h =
                            static_cast<double>(wlop_radial_mean_multiplier) * avg_spacing;
                        wlop_params.I       = output_point_cloud.size();
                        wlop_params.mu      = mu;
                        wlop_params.uniform = uniform;

                        pcp::algorithm::ear::wlop(
                            indices.begin(),
                            indices.end(),
                            output_point_cloud.begin(),
                            input_point_map,
                            input_normal_map,
                            wlop_params);

                        timer.stop();
                    });
                }

                ImGui::TreePop();
            }

            if (ImGui::Button("Resample##EAR", ImVec2((w - p) / 2.f, 0.f)) &&
                !is_downsampling_running())
            {
                progress_str     = "Executing ...";
                execution_handle = std::async(std::launch::async, [&]() {
                    timer.register_op("ear");
                    timer.start();

                    resize_output_point_cloud();
                    float const avg_spacing = get_average_spacing(kneighbours);

                    pcp::algorithm::ear::normal_smoothing_params_t normal_smoothing_params{};
                    normal_smoothing_params.K = K;
                    normal_smoothing_params.sigmap =
                        static_cast<double>(normal_smoothing_radial_mean_multiplier) * avg_spacing;
                    normal_smoothing_params.sigman = sigman;

                    pcp::algorithm::ear::wlop_params_t wlop_params{};
                    wlop_params.K  = K;
                    wlop_params.h  = static_cast<double>(wlop_radial_mean_multiplier) * avg_spacing;
                    wlop_params.I  = output_point_cloud.size();
                    wlop_params.mu = mu;
                    wlop_params.uniform = uniform;

                    pcp::algorithm::ear::resample_away_from_edges(
                        indices.begin(),
                        indices.end(),
                        output_point_cloud.begin(),
                        output_normals.begin(),
                        input_point_map,
                        input_normal_map,
                        normal_smoothing_params,
                        wlop_params);

                    timer.stop();
                });
            }
        }

        if (execution_handle.valid() &&
            execution_handle.wait_for(std::chrono::microseconds(0u)) == std::future_status::ready)
        {
            execution_handle.get();

            auto const duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(timer.ops.front().second);

            progress_str = "Execution time: " + std::to_string(duration.count()) + " ms";

            timer.ops.clear();
            draw_point_cloud(output_point_cloud);
            if (show_normals)
                draw_normals(output_point_cloud, output_normals, normals_scale);

            is_input_point_cloud = false;
        }

        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float const w = ImGui::GetContentRegionAvailWidth();
            float const p = ImGui::GetStyle().FramePadding.x;

            ImGui::SliderFloat("Normals scaling", &normals_scale, 0.01f, 1.f, "%.2f");

            if (ImGui::Button("Show input point cloud##Visualization", ImVec2((w - p) / 2.f, 0.f)))
            {
                is_input_point_cloud = true;
                draw_point_cloud(input_point_cloud);
            }
            if (ImGui::Button(
                    "Show output point cloud##Visualization",
                    ImVec2((w - p) / 2.f, 0.f)) &&
                !is_downsampling_running())
            {
                is_input_point_cloud = false;
                draw_point_cloud(output_point_cloud);
            }
            if (ImGui::Button("Show Input Normals##Visualization", ImVec2((w - p) / 2.f, 0.f)))
            {
                draw_normals(input_point_cloud, input_normals, normals_scale);
            }
            if (ImGui::Button("Show Output Normals##Visualization", ImVec2((w - p) / 2.f, 0.f)))
            {
                draw_normals(output_point_cloud, output_normals, normals_scale);
            }
            if (ImGui::Button("Hide Normals", ImVec2((w - p) / 2.f, 0.f)))
            {
                viewer.data().clear_edges();
            }
            ImGui::SliderFloat("Point size##Visualization", &viewer.data().point_size, 1.f, 50.f);
        }

        ImGui::End();
    };

    viewer.launch();
    return 0;
}