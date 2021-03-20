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

void apply_filter(std::vector<pcp::point_t>& points, std::atomic<float>& progress);

Eigen::MatrixXd from_point_cloud(std::vector<pcp::point_t> const& points);

// params for statistical filter
static float std_dev_multiplier_threshold = 1.0f;
static std::size_t mean_k                 = 3u;

// params for pass through filter
static std::size_t index_to_filter = 1u;
static float filter_limit_max      = 2.0f;
static float filter_limit_min      = 0.5;

// params for radius outlier filter
static float radius                        = 1.0f;
static std::size_t min_neighbors_in_radius = 1u;

// params for density filter
static float density_threshold = 1.0f;
static float radius_multiplier = 1.0f;
static std::size_t density_k   = 3u;
static int filters_variant     = 0;
int main(int argc, char** argv)
{
    static std::atomic<float> recon_progress = 0.f;
    static std::vector<pcp::point_t> points;
    static std::future<void> execution_handle{};
    static std::string progress_str{""};
    pcp::common::basic_timer_t timer;
    auto const is_filter_running = [&]() {
        return execution_handle.valid();
    };

    static std::string execution_report;
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_window = [&]() {
        ImGui::Begin("Point Cloud Processing");
        if (ImGui::CollapsingHeader("Filters example", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::TreePush();
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;
            if (ImGui::Button("Load##PointCloud", ImVec2((w - p) / 2.f, 0)) && !is_filter_running())
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
            if (ImGui::Button("Save##PointCloud", ImVec2((w - p) / 2.f, 0.f)))
            {
                /*               std::filesystem::path ply_mesh = igl::file_dialog_save();
                               pcp::io::write_ply(
                                   ply_mesh,
                                   points,
                                   std::vector<normal_type>{},
                                   pcp::io::ply_format_t::binary_little_endian);*/
            }

            static ImU64 const step = 1;

            if (ImGui::CollapsingHeader("Display##PointCloud", ImGuiTreeNodeFlags_DefaultOpen))
            {
                std::string const points_str =
                    std::string("Points: ") + std::to_string(points.size());
                ImGui::Text(points_str.c_str());
            }

            if (ImGui::CollapsingHeader("Filters", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::RadioButton("statistical", &filters_variant, 0);

                ImGui::InputFloat(
                    "Standard dev multiplier threshold",
                    &std_dev_multiplier_threshold,
                    0.1f,
                    1.0f,
                    "%0.2f");

                ImGui::InputScalar("mean k", ImGuiDataType_U64, &mean_k, &step);

                ImGui::RadioButton("pass through", &filters_variant, 1);

                ImGui::InputScalar("index", ImGuiDataType_U64, &index_to_filter, &step);
                ImGui::InputFloat("min", &filter_limit_min, 0.1f, 1.0f, "%0.2f");
                ImGui::InputFloat("max", &filter_limit_max, 0.1f, 1.0f, "%0.2f");

                ImGui::RadioButton("radius outlier", &filters_variant, 2);

                ImGui::InputFloat("radius", &radius, 0.1f, 1.0f, "%0.2f");
                ImGui::InputScalar(
                    "min neighbors in radius",
                    ImGuiDataType_U64,
                    &min_neighbors_in_radius,
                    &step);

                ImGui::RadioButton("density", &filters_variant, 3);

                ImGui::InputFloat("density threshold", &density_threshold, 0.1f, 1.0f, "%0.2f");
                ImGui::InputFloat("radius multiplier", &radius_multiplier, 0.1f, 1.0f, "%0.2f");
            }

            if (ImGui::Button("Reset", ImVec2((w - p) / 2.f, 0)) && !is_filter_running())
            {
                points.clear();
                progress_str = ""; 
                viewer.data().clear();
                recon_progress = 0.0f;
            }
            if (ImGui::Button("Execute", ImVec2((w - p) / 2.f, 0)) && !is_filter_running())
            {
                if (points.size() > 0)
                {
                    progress_str     = "Executing ...";
                    execution_handle = std::async(std::launch::async, [&]() {
                        //     timer.register_op("filter3");
                        timer.register_op("filter");
                        timer.start();
                        apply_filter(points, recon_progress);
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
void apply_filter(std::vector<pcp::point_t>& points, std::atomic<float>& progress)
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    progress                           = 0.f;
    float constexpr num_ops            = 4.f;
    float constexpr progress_increment = 1.f / 2.f;
    auto const progress_forward        = [&]() {
        progress = progress + progress_increment;
    };
    using stats_filter =
        pcp::basic_statistical_outlier_filter_t<pcp::point_t, float, decltype(coordinate_map)>;
    using pass_through_filter =
        pcp::basic_pass_through_filter_t<pcp::point_t, float, decltype(coordinate_map)>;
    using radius_outlier_filter =
        pcp::basic_radius_outlier_filter_t<pcp::point_t, float, decltype(coordinate_map)>;
    using density_filter =
        pcp::basic_density_filter_t<pcp::point_t, float, decltype(coordinate_map)>;

    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;
    pcp::kdtree::construction_params_t params{};
    params.max_depth    = 4u;
    params.construction = pcp::kdtree::construction_t::nth_element;

    pcp::statistical_outlier_filter::construction_params_t<float> statistical_filter_params{};
    pcp::pass_through_filter::construction_params_t<float> pass_through_filter_params{};
    pcp::radius_outlier_filter::construction_params_t<float> radius_outlier_filter_params{};
    pcp::density_filter::construction_params_t<float> density_filter_params{};

    kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};
    progress_forward();
    if (filters_variant == 0)
    {
        statistical_filter_params.std_dev_multiplier_threshold_ = std_dev_multiplier_threshold;
        statistical_filter_params.mean_k_                       = mean_k;

        stats_filter statistical_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            statistical_filter_params);
        auto it =
            std::remove_if(std::execution::par, points.begin(), points.end(), statistical_filter);
        points.erase(it, points.end());
    }
    else if (filters_variant == 1)
    {
        pass_through_filter_params.index_to_filter_  = index_to_filter;
        pass_through_filter_params.filter_limit_max_ = filter_limit_max;
        pass_through_filter_params.filter_limit_min_ = filter_limit_min;

        pass_through_filter pass_through_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            pass_through_filter_params);
        auto it =
            std::remove_if(std::execution::par, points.begin(), points.end(), pass_through_filter);
        points.erase(it, points.end());
        // apply filter here
    }
    else if (filters_variant == 2)
    {
        radius_outlier_filter_params.radius_                  = radius;
        radius_outlier_filter_params.min_neighbors_in_radius_ = min_neighbors_in_radius;

        radius_outlier_filter radius_outlier_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            radius_outlier_filter_params);
        auto it = std::remove_if(
            std::execution::par,
            points.begin(),
            points.end(),
            radius_outlier_filter);
        points.erase(it, points.end());
    }
    else
    {
        density_filter_params.density_threshold_ = density_threshold;
        density_filter_params.radius_multiplier_ = radius_multiplier;
        density_filter_params.k_                 = density_k;

        density_filter density_filter(
            points.begin(),
            points.end(),
            coordinate_map,
            kdtree,
            density_filter_params);
        auto it = std::remove_if(std::execution::par, points.begin(), points.end(), density_filter);
        points.erase(it, points.end());
    }
    progress_forward();
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