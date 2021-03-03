#include <filesystem>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <pcp/algorithm/average_distance_to_neighbors.hpp>
#include <pcp/algorithm/wlop.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/io/ply.hpp>

Eigen::MatrixXd from_point_cloud(std::vector<pcp::point_t> const& points);

template <class PointMap>
float compute_average_distance_to_neighbors(
    std::vector<std::size_t> const& indices,
    PointMap const& point_map,
    std::size_t k = 15u);

std::pair<float, float> compute_mean_distance_variance(
    std::vector<pcp::point_t> const& input,
    std::vector<pcp::point_t> const& output,
    std::size_t k = 15u);

int main(int argc, char** argv)
{
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    std::vector<point_type> input_point_cloud;
    std::vector<std::size_t> indices;
    std::vector<point_type> output_point_cloud;
    float input_mu = 0.f;
    int knn        = 15;

    auto const point_map = [&](std::size_t const i) {
        return input_point_cloud[i];
    };

    menu.callback_draw_viewer_window = [&]() {
        ImGui::Begin("Point Cloud Processing");

        auto const draw_point_cloud = [&](bool show_output_point_cloud = false) {
            Eigen::MatrixXd V = show_output_point_cloud ? from_point_cloud(output_point_cloud) :
                                                          from_point_cloud(input_point_cloud);
            viewer.data().clear();
            viewer.data().add_points(V, Eigen::RowVector3d(1.0, 1.0, 0.0));
            viewer.data().point_size = 1.f;
            viewer.core().align_camera_center(V);
        };

        if (ImGui::CollapsingHeader("IO", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;
            if (ImGui::Button("Load##PointCloud", ImVec2((w - p) / 2.f, 0.f)))
            {
                std::string const filename = igl::file_dialog_open();
                std::filesystem::path ply_point_cloud{filename};
                auto [p, _]       = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);
                input_point_cloud = std::move(p);
                indices.clear();
                indices.resize(input_point_cloud.size());
                std::iota(indices.begin(), indices.end(), 0u);
                input_mu = compute_average_distance_to_neighbors(
                    indices,
                    point_map,
                    static_cast<std::size_t>(knn));
                draw_point_cloud();
            }
            ImGui::SameLine();
            if (ImGui::Button("Save##PointCloud", ImVec2((w - p) / 2.f, 0.f)))
            {
                std::filesystem::path ply_mesh = igl::file_dialog_save();
                pcp::io::write_ply(
                    ply_mesh,
                    output_point_cloud,
                    std::vector<normal_type>{},
                    pcp::io::ply_format_t::binary_little_endian);
            }
        }

        if (ImGui::CollapsingHeader("WLOP", ImGuiTreeNodeFlags_DefaultOpen))
        {
            static int k    = 0;
            static float mu = 0.45f;
            static float h  = 0.f;
            static int I    = 0;
            static bool uniform{false};

            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;

            std::string const num_points_str =
                "Input points: " + std::to_string(input_point_cloud.size());
            ImGui::BulletText(num_points_str.c_str());

            ImGui::PushItemWidth(w / 2);

            ImGui::InputInt("Number of iterations", &k);
            ImGui::InputFloat("mu", &mu, 0.01f, 0.1f, 2);
            ImGui::InputFloat("Radial support radius", &h, 0.001f, 0.01f, "%.5f");
            ImGui::InputInt("Downsampled size", &I);
            ImGui::Checkbox("Require uniform", &uniform);
            ImGui::InputInt("KNN for mean computation", &knn);

            static float input_point_cloud_variance  = 0.f;
            static float output_point_cloud_variance = 0.f;

            if (ImGui::Button("Downsample", ImVec2((w - p) / 2.f, 0.f)))
            {
                pcp::algorithm::wlop::params_t params;
                params.I       = static_cast<std::size_t>(I);
                params.k       = static_cast<std::size_t>(k);
                params.mu      = static_cast<double>(mu);
                params.uniform = uniform;

                if (params.h == 0.f)
                {
                    float const mu = compute_average_distance_to_neighbors(
                        indices,
                        point_map,
                        static_cast<std::size_t>(knn));
                    params.h = mu * 8.f;
                }

                output_point_cloud.clear();
                output_point_cloud.reserve(params.I);
                pcp::algorithm::wlop::wlop(
                    indices.begin(),
                    indices.end(),
                    std::back_inserter(output_point_cloud),
                    point_map,
                    params);

                auto const [input_var, output_var] = compute_mean_distance_variance(
                    input_point_cloud,
                    output_point_cloud,
                    static_cast<std::size_t>(knn));

                input_point_cloud_variance  = input_var;
                output_point_cloud_variance = output_var;

                draw_point_cloud(true);
            }

            ImGui::BulletText("Input mu: %.7f", input_mu);
            ImGui::BulletText("Input variance: %.7f", input_point_cloud_variance);
            ImGui::BulletText("Output variance: %.7f", output_point_cloud_variance);

            ImGui::PopItemWidth();
        }

        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float w = ImGui::GetContentRegionAvailWidth();
            float p = ImGui::GetStyle().FramePadding.x;
            if (ImGui::Button("Show input point cloud##Visualization", ImVec2((w - p) / 2.f, 0.f)))
            {
                draw_point_cloud(false);
            }
            if (ImGui::Button("Show output point cloud##Visualization", ImVec2((w - p) / 2.f, 0.f)))
            {
                draw_point_cloud(true);
            }
        }

        ImGui::End();
    };

    viewer.core().rotation_type = igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL;
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

template <class PointMap>
float compute_average_distance_to_neighbors(
    std::vector<std::size_t> const& indices,
    PointMap const& point_map,
    std::size_t k)
{
    auto const coordinate_map = [&](std::size_t const i) {
        pcp::point_t const& p = point_map(i);
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t kd_params;
    kd_params.compute_max_depth = true;

    pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        kd_params};

    auto const knn_map = [&](std::size_t const i) {
        std::size_t num_neighbors = static_cast<std::size_t>(k);
        return kdtree.nearest_neighbours(i, num_neighbors);
    };

    float const h = pcp::algorithm::average_distance_to_neighbors(
        indices.begin(),
        indices.end(),
        point_map,
        knn_map);

    return h;
}

std::pair<float, float> compute_mean_distance_variance(
    std::vector<pcp::point_t> const& input_point_cloud,
    std::vector<pcp::point_t> const& output_point_cloud,
    std::size_t k)
{
    std::vector<std::size_t> input_indices(input_point_cloud.size());
    std::vector<std::size_t> output_indices(output_point_cloud.size());
    std::iota(input_indices.begin(), input_indices.end(), 0u);
    std::iota(output_indices.begin(), output_indices.end(), 0u);

    auto const input_point_map = [&](std::size_t const i) {
        return input_point_cloud[i];
    };
    auto const output_point_map = [&](std::size_t const i) {
        return output_point_cloud[i];
    };
    auto const input_coordinate_map = [&](std::size_t const i) {
        pcp::point_t const& p = input_point_cloud[i];
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };
    auto const output_coordinate_map = [&](std::size_t const i) {
        pcp::point_t const& p = output_point_cloud[i];
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    pcp::kdtree::construction_params_t kd_params;
    kd_params.compute_max_depth = true;

    pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(input_coordinate_map)> input_kdtree{
        input_indices.begin(),
        input_indices.end(),
        input_coordinate_map,
        kd_params};

    pcp::basic_linked_kdtree_t<std::size_t, 3u, decltype(output_coordinate_map)> output_kdtree{
        output_indices.begin(),
        output_indices.end(),
        output_coordinate_map,
        kd_params};

    auto const input_knn_map = [&](std::size_t const i) {
        std::size_t num_neighbors = static_cast<std::size_t>(k);
        return input_kdtree.nearest_neighbours(i, num_neighbors);
    };
    auto const output_knn_map = [&](std::size_t const i) {
        std::size_t num_neighbors = static_cast<std::size_t>(k);
        return output_kdtree.nearest_neighbours(i, num_neighbors);
    };

    std::vector<float> input_mean_distances = pcp::algorithm::average_distances_to_neighbors(
        input_indices.begin(),
        input_indices.end(),
        input_point_map,
        input_knn_map);

    std::vector<float> output_mean_distances = pcp::algorithm::average_distances_to_neighbors(
        output_indices.begin(),
        output_indices.end(),
        output_point_map,
        output_knn_map);

    float const input_mu = std::reduce(input_mean_distances.begin(), input_mean_distances.end()) /
                           static_cast<float>(input_mean_distances.size());
    float const output_mu =
        std::reduce(output_mean_distances.begin(), output_mean_distances.end()) /
        static_cast<float>(output_mean_distances.size());

    float const input_variance = std::accumulate(
        input_mean_distances.begin(),
        input_mean_distances.end(),
        0.f,
        [&input_mu](float const sum, float const mean) {
            float const diff = mean - input_mu;
            return sum + diff * diff;
        });

    float const output_variance = std::accumulate(
        output_mean_distances.begin(),
        output_mean_distances.end(),
        0.f,
        [&output_mu](float const sum, float const mean) {
            float const diff = mean - output_mu;
            return sum + diff * diff;
        });

    return {input_variance, output_variance};
}
