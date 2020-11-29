#include <catch2/catch.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/io/obj.hpp>

SCENARIO("obj file manipulation", "[obj]")
{
    pcp::point_t const point_truth{1.1f, 2.2f, 3.3f};
    pcp::normal_t const normal_truth{1.f, 0.f, 0.f};
    GIVEN("a valid point cloud with normals obj file")
    {
        std::ostringstream oss{};
        oss << "v 1.1 2.2 3.3\n"
            << "v 1.1 2.2 3.3\n"
            << "v 1.1 2.2 3.3\n"
            << "vn 1.0 0.0 0.0\n"
            << "vn 1.0 0.0 0.0\n"
            << "vn 1.0 0.0 0.0\n";

        std::istringstream iss{oss.str()};
        WHEN("parsing the obj's vertices and normals")
        {
            auto [vertices, normals] = pcp::io::read_obj<pcp::point_t, pcp::normal_t>(iss);
            THEN("the correct vertices and normals are recovered")
            {
                REQUIRE(vertices.size() == 3u);
                REQUIRE(normals.size() == 3u);
                for (std::size_t i = 0; i < 3u; ++i)
                {
                    REQUIRE(pcp::common::are_vectors_equal(vertices[i], point_truth));
                    REQUIRE(pcp::common::are_vectors_equal(normals[i], normal_truth));
                }
            }
        }
    }
    GIVEN("a valid point cloud without normals obj file")
    {
        std::ostringstream oss{};
        oss << "v 1.1 2.2 3.3\n"
            << "v 1.1 2.2 3.3\n"
            << "v 1.1 2.2 3.3\n";

        std::istringstream iss{oss.str()};
        WHEN("parsing the obj's vertices")
        {
            auto [vertices, normals] = pcp::io::read_obj<pcp::point_t, pcp::normal_t>(iss);
            THEN("the correct vertices are recovered and there are no normals")
            {
                REQUIRE(vertices.size() == 3u);
                REQUIRE(normals.empty());
                for (std::size_t i = 0; i < 3u; ++i)
                {
                    REQUIRE(pcp::common::are_vectors_equal(vertices[i], point_truth));
                }
            }
        }
    }
    GIVEN("a vector of points")
    {
        auto const num_points = 3u;
        std::vector<pcp::point_t> vertices(num_points, pcp::point_t{1.0f, 1.0f, 1.0f});
        std::ostringstream oss{};

        WHEN("writing the vector of points as an obj file")
        {
            pcp::io::write_obj<pcp::point_t, pcp::normal_t>(oss, vertices, {});
            THEN("the resulting obj file is valid and encodes all points")
            {
                std::ostringstream truth_stream{};
                for (unsigned int i = 0; i < num_points; ++i)
                {
                    truth_stream << "v " << std::to_string(1.f) << " " << std::to_string(1.f) << " "
                                 << std::to_string(1.f) << "\n";
                }

                std::string const truth = truth_stream.str();
                std::string const obj   = oss.str();
                REQUIRE(obj == truth);
            }
        }
        GIVEN("a vector of normals")
        {
            std::vector<pcp::normal_t> normals(num_points, pcp::normal_t{1.0f, 0.0f, 0.0f});
            WHEN("writing the vector of points and normals as an obj file")
            {
                pcp::io::write_obj<pcp::point_t, pcp::normal_t>(oss, vertices, normals);
                THEN("the resulting obj file is valid and encodes all points with normals")
                {
                    std::ostringstream truth_stream{};
                    for (unsigned int i = 0; i < num_points; ++i)
                    {
                        truth_stream << "v " << std::to_string(1.f) << " " << std::to_string(1.f)
                                     << " " << std::to_string(1.f) << "\n";
                        truth_stream << "vn " << std::to_string(1.f) << " " << std::to_string(0.f)
                                     << " " << std::to_string(0.f) << "\n";
                    }

                    std::string const truth = truth_stream.str();
                    std::string const obj   = oss.str();
                    REQUIRE(obj == truth);
                }
            }
        }
    }
}
