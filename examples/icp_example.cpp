#include <atomic>
#include <future>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/writePLY.h>
#include <iostream>
#include <pcp/filter/density_filter.hpp>
#include <pcp/filter/pass_through_filter.hpp>
#include <pcp/filter/radius_outlier_filter.hpp>
#include <pcp/filter/statistical_outlier_filter.hpp>
#include <pcp/pcp.hpp>
#include <range/v3/view/transform.hpp>
#include <sstream>

using point_type  = pcp::point_t;
using normal_type = pcp::normal_t;
using vertex_type = pcp::vertex_t;
using plane_type  = pcp::common::plane3d_t;

Eigen::MatrixXd from_point_cloud(std::vector<pcp::point_t> const& points);

int main(int argc, char** argv)
{
    auto const point_map = [](pcp::point_t const& p) {
        return p;
    };
    point_type shift                         = point_type{3.00, 0, 0};
    static std::atomic<float> recon_progress = 0.f;
    static std::vector<pcp::point_t> points;
    static std::vector<pcp::point_t> points_B;
    static std::future<void> execution_handle{};
    static std::string progress_str{""};
    pcp::common::basic_timer_t timer;
    auto const is_running = [&]() {
        return execution_handle.valid();
    };

    static std::string execution_report;
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    auto const reset = [&]() {
        points.clear();
        points_B.clear();
        progress_str = "";
        viewer.data().clear();
        recon_progress = 0.0f;
    };

    menu.callback_draw_viewer_window = [&]() {
        ImGui::Begin("Point Cloud Processing");
        if (ImGui::CollapsingHeader("Filters example", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TreePush();
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;
            if (ImGui::Button("Load##PointCloud", ImVec2((w - p) / 2.f, 0)) && !is_running())
            {
                reset();
                std::string const filename = igl::file_dialog_open();
                std::filesystem::path ply_point_cloud{filename};
                auto [p, _] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
                points      = std::move(p);
                points_B.resize(points.size());
                // translate original point_cloud by shift
                for (auto i = 0; i < points.size(); ++i)
                {
                    auto const p = point_map(points[i]);
                    points_B[i]  = p + shift;
                }

                auto const V = from_point_cloud(points);
                auto const U = from_point_cloud(points_B);
                viewer.data().clear();
                viewer.data().add_points(V, Eigen::RowVector3d(1.0, 1.0, 0.0));
                viewer.data().add_points(U, Eigen::RowVector3d(1.0, 1.0, 0.0));
                viewer.data().point_size = 1.f;
                viewer.core().align_camera_center(V);
            }

            // ImGui::SameLine();
            // if (ImGui::Button("Save##PointCloud", ImVec2((w - p) / 2.f, 0.f)))
            //{
            //    std::filesystem::path ply_mesh = igl::file_dialog_save();
            //    pcp::io::write_ply(
            //        ply_mesh,
            //        points,
            //        std::vector<normal_type>{},
            //        pcp::io::ply_format_t::binary_little_endian);
            //}

            //static ImU64 const step = 1;

            if (ImGui::CollapsingHeader("Display##PointCloud", ImGuiTreeNodeFlags_DefaultOpen))
            {
                std::string const points_str =
                    std::string("Points: ") + std::to_string(points.size());
                ImGui::Text(points_str.c_str());
            }

            if (ImGui::Button("Reset", ImVec2((w - p) / 2.f, 0)) && !is_running())
            {
                reset();
            }
            if (ImGui::Button("StepIcp", ImVec2((w - p) / 2.f, 0)) && !is_running())
            {
                if (points.size() > 0)
                {
                    progress_str     = "Executing ...";
                    execution_handle = std::async(std::launch::async, [&]() {
                        timer.register_op("icp");
                        timer.start();
                        step_icp();
                        timer.stop();
                    });
                }
            }

            if (!progress_str.empty())
                ImGui::BulletText(progress_str.c_str());

            ImGui::ProgressBar(recon_progress, ImVec2(0.f, 0.f));
            if (execution_handle.valid() && execution_handle.wait_for(std::chrono::microseconds(
                                                0u)) == std::future_status::ready)
            {
                execution_handle.get();
                viewer.data().clear();
                auto const V = from_point_cloud(points);
                viewer.data().add_points(V, Eigen::RowVector3d(1.0, 1.0, 0.0));
                viewer.data().point_size = 1.f;
                viewer.core().align_camera_center(V);
                auto const duration =
                    std::chrono::duration_cast<std::chrono::milliseconds>(timer.ops.front().second);
                timer.ops.clear();
                progress_str = "Execution time: " + std::to_string(duration.count()) + " ms";
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