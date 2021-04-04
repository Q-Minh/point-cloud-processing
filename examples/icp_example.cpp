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

Eigen::MatrixXf from_point_cloud(std::vector<pcp::point_t> const& points);

template <class ScalarType, class KnnMap>
ScalarType step_icp(
    Eigen::MatrixXf& m_src,
    Eigen::MatrixXf& m_ref,
    Eigen::Matrix4f& m_src_tm,
    Eigen::Matrix4f& m_init_tm,
    KnnMap knn_map,
    std::atomic<float>& progress);

auto const coordinate_map = [](pcp::point_t const& p) {
    return std::array<float, 3u>{p.x(), p.y(), p.z()};
};
using normal_type = pcp::normal_t;
using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
int main(int argc, char** argv)
{
    pcp::point_t shift                       = pcp::point_t{3.00, 0, 0};
    static std::atomic<float> recon_progress = 0.f;
    static std::vector<pcp::point_t> points;
    static std::vector<pcp::point_t> points_B;

    static std::future<void> execution_handle{};
    static std::string progress_str{""};
    pcp::common::basic_timer_t timer;

    static Eigen::MatrixXf m_ref;
    static Eigen::MatrixXf m_src;
    // transformation done by gui
    static Eigen::Matrix4f m_init_tm = Eigen::MatrixXf::Identity(4, 4);
    // calculated via best fit transform
    static Eigen::Matrix4f m_src_tm = Eigen::MatrixXf::Identity(4, 4);

    static pcp::kdtree::construction_params_t params{};
    params.max_depth      = 4u;
    params.construction   = pcp::kdtree::construction_t::nth_element;
    auto const is_running = [&]() {
        return execution_handle.valid();
    };

    static std::string execution_report;
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    auto const reset = [&]() {
        m_init_tm = Eigen::MatrixXf::Identity(4, 4);
        m_src_tm  = Eigen::MatrixXf::Identity(4, 4);
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
                auto [p, _] = pcp::io::read_ply<pcp::point_t, normal_type>(ply_point_cloud);
                points      = std::move(p);
                points_B.resize(points.size());

                // progress_forward();

                // translate original point_cloud by shift
                for (auto i = 0; i < points.size(); ++i)
                {
                    auto const p = (points[i]);
                    points_B[i]  = p + shift;
                }

                m_ref = from_point_cloud(points);
                m_src = from_point_cloud(points_B);
                viewer.data().clear();
                viewer.data().add_points(m_ref, Eigen::RowVector3f(1.0, 1.0, 0.0));
                viewer.data().add_points(m_src, Eigen::RowVector3f(1.0, 1.0, 0.0));
                viewer.data().point_size = 1.f;
                viewer.core().align_camera_center(m_ref);
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

            // static ImU64 const step = 1;

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

                        static kdtree_type kdtree{
                            points.begin(),
                            points.end(),
                            coordinate_map,
                            params};
                        auto const knn_map = [&](Eigen::Vector4f const& p) {
                            auto converted_point      = pcp::point_t{p(0), p(1), p(2)};
                            std::size_t num_neighbors = static_cast<std::size_t>(1);
                            return kdtree.nearest_neighbours(converted_point, num_neighbors)[0];
                        };
                        step_icp<float, decltype(knn_map)>(
                            m_src,
                            m_ref,
                            m_src_tm,
                            m_init_tm,
                            knn_map,
                            recon_progress);
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
                viewer.data().add_points(V, Eigen::RowVector3f(1.0, 1.0, 0.0));
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

template <class ScalarType, class KnnMap>
ScalarType step_icp(
    Eigen::MatrixXf& m_src,
    Eigen::MatrixXf& m_ref,
    Eigen::Matrix4f& m_src_tm,
    Eigen::Matrix4f& m_init_tm,
    KnnMap knn_map,
    std::atomic<float>& progress)
{
    auto const n = m_src.rows();
    Eigen::MatrixXf A(n);
    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector4f p(m_src(i, 0), m_src(i, 1), m_src(i, 2), 0);

        p = m_src_tm * m_init_tm * p;
        // converted_pt = = pcp::point_t{p(0), p(1), p(2)};
        auto const k = knn_map(p);

        m_src(i, 0) = p(0);
        m_src(i, 1) = p(1);
        m_src(i, 2) = p(2);
        A(i, 0)     = k.x();
        A(i, 1)     = k.y();
        A(i, 2)     = k.z();
    }
    auto const m_t = pcp::algorithm::icp_best_fit_Transform<float>(A, m_src);
    m_src_tm       = m_t * m_src_tm;
    return pcp::algorithm::icp_error<float>(m_ref, m_src);
}

Eigen::MatrixXf from_point_cloud(std::vector<pcp::point_t> const& points)
{
    Eigen::MatrixXf P;
    P.resize(points.size(), 3u);
    for (std::size_t i = 0u; i < points.size(); ++i)
    {
        P(i, 0) = points[i].x();
        P(i, 1) = points[i].y();
        P(i, 2) = points[i].z();
    }
    return P;
}