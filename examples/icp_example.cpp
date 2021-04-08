#include <atomic>
#include <future>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/writePLY.h>
#include <iostream>
#include <pcp/algorithm/icp.hpp>
#include <pcp/pcp.hpp>
#include <sstream>

auto const coordinate_map = [](pcp::point_t const& p) {
    return std::array<float, 3u>{p.x(), p.y(), p.z()};
};

using normal_type   = pcp::normal_t;
using kdtree_type   = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
using matrix_3_type = Eigen::Matrix<float, 3, 3>;
using vector_3_type = Eigen::Matrix<float, 3, 1>;

Eigen::MatrixXd from_point_cloud(std::vector<pcp::point_t> const& points);

void step_icp(std::vector<pcp::point_t> const& points_ref, std::vector<pcp::point_t>& points_src);

void merge(std::vector<pcp::point_t>& ref, std::vector<pcp::point_t>& src);

int main(int argc, char** argv)
{
    pcp::point_t shift = pcp::point_t{0.10, 0, 0};

    std::vector<pcp::point_t> points;
    std::vector<pcp::point_t> points_B;

    std::future<void> execution_handle{};
    std::string progress_str{""};
    pcp::common::basic_timer_t timer;

    // Rotation matrix
    matrix_3_type R = matrix_3_type::Identity();
    // Translation vector
    vector_3_type t;

    auto const is_running = [&]() {
        return execution_handle.valid();
    };

    static std::string execution_report;
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    auto const reset = [&]() {
        R = matrix_3_type::Identity();
        t = vector_3_type::Zero();
        points.clear();
        points_B.clear();
        progress_str = "";
        viewer.data().clear();
    };
    auto const refresh = [&]() {
        auto m_ref = from_point_cloud(points);
        auto m_src = from_point_cloud(points_B);
        viewer.data().clear();
        viewer.data().add_points(m_ref, Eigen::RowVector3d(1.0, 0.5, 0.0));
        viewer.data().add_points(m_src, Eigen::RowVector3d(1.0, 1.0, 1.0));
        viewer.data().point_size = 1.f;
        viewer.core().align_camera_center(m_ref);
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
                auto [p, _] = pcp::io::read_ply<pcp::point_t, normal_type>(ply_point_cloud);
                points      = std::move(p);
                points_B.resize(points.size());

                // translate original point_cloud by shift, for now our point cloud to stitch
                for (auto i = 0; i < points.size(); ++i)
                {
                    auto const p = (points[i]);
                    points_B[i]  = p + shift;
                }

                refresh();
            }

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
                        if (points_B.size() > 0)
                        {
                            step_icp(points, points_B);
                            timer.stop();
                            refresh();
                        }
                    });
                }
            }

            if (ImGui::Button("Merge", ImVec2((w - p) / 2.f, 0)) && !is_running())
            {
                if (points.size() > 0)
                {
                    progress_str     = "Executing ...";
                    execution_handle = std::async(std::launch::async, [&]() {
                        timer.register_op("merge");
                        timer.start();
                        merge(points, points_B);
                        points_B.clear();
                        timer.stop();
                        refresh();
                    });
                }
            }

            if (!progress_str.empty())
                ImGui::BulletText(progress_str.c_str());

            if (execution_handle.valid() && execution_handle.wait_for(std::chrono::microseconds(
                                                0u)) == std::future_status::ready)
            {
                execution_handle.get();
                refresh();
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

void step_icp(std::vector<pcp::point_t> const& points_ref, std::vector<pcp::point_t>& points_src)
{
    // Smallest of the two
    auto const n = (points_ref.size() <= points_src.size()) ? points_ref.size() : points_src.size();

    std::vector<pcp::point_t> down_ref{}, down_src{}, ref(n);

    // downsample largest to smallest
    if (points_ref.size() > n)
    {
        pcp::algorithm::random_simplification(
            points_ref.begin(),
            points_ref.end(),
            std::back_inserter(down_ref),
            n);
    }
    else
    {
        down_ref = points_ref;
    }

    if (points_src.size() > n)
    {
        pcp::algorithm::random_simplification(
            points_src.begin(),
            points_src.end(),
            std::back_inserter(down_src),
            n);
    }
    else
    {
        down_src = points_src;
    }

    // Kdtree construction for knn (k = 1) searches
    pcp::kdtree::construction_params_t params{};
    params.compute_max_depth = true;
    params.construction      = pcp::kdtree::construction_t::nth_element;

    kdtree_type kdtree{down_ref.begin(), down_ref.end(), coordinate_map, params};

    auto const knn_map = [&](pcp::point_t const& p) {
        std::size_t num_neighbors = static_cast<std::size_t>(1);
        return kdtree.nearest_neighbours(p, num_neighbors);
    };
    auto const point_map = [](pcp::point_t const& p) {
        return p;
    };
    std::transform(down_src.begin(), down_src.end(), ref.begin(), [&](pcp::point_t const& p) {
        auto const neighbors = knn_map(p);
        return neighbors.front();
    });

    matrix_3_type R;
    vector_3_type t;

    std::tie(R, t) = pcp::algorithm::icp_best_fit_transform(
        ref.begin(),
        ref.end(),
        down_src.begin(),
        down_src.end(),
        point_map,
        point_map);

    std::transform(
        down_src.begin(),
        down_src.end(),
        points_src.begin(),
        [&](pcp::point_t const& p) {
            auto const converted_point      = vector_3_type(p.x(), p.y(), p.z());
            vector_3_type transformed_point = R * converted_point;
            transformed_point += t; // translation
            return pcp::point_t{transformed_point(0), transformed_point(1), transformed_point(2)};
        });
}

void merge(std::vector<pcp::point_t>& ref, std::vector<pcp::point_t>& src)
{
    std::transform(src.begin(), src.end(), std::back_inserter(ref), [&](pcp::point_t const& p) {
        return p;
    });
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