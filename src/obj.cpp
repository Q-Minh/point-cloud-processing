#include <catch2/catch.hpp>

#include <obj.hpp>

SCENARIO("obj file manipulation", "[obj]") {
	GIVEN("a valid point cloud with normals obj file") {
		std::ostringstream oss{};
		oss << "v 1.1 2.2 3.3\n"
			<< "v 1.1 2.2 3.3\n"
			<< "v 1.1 2.2 3.3\n"
			<< "vn 1.0 0.0 0.0\n"
			<< "vn 1.0 0.0 0.0\n"
			<< "vn 1.0 0.0 0.0\n";
		
		std::istringstream iss{ oss.str() };
		WHEN("parsing the obj's vertices and normals") {
			auto [vertices, normals] = pcp::io::read_obj<float, float>(iss);
			THEN("the correct vertices and normals are recovered") {
				REQUIRE(vertices.size() == 3u);
				REQUIRE(normals.size() == 3u);
				for (auto i = 0; i < 3; ++i)
				{
					REQUIRE(vertices[i].x == 1.1f);
					REQUIRE(vertices[i].y == 2.2f);
					REQUIRE(vertices[i].z == 3.3f);
					REQUIRE(normals[i].x == 1.0f);
					REQUIRE(normals[i].y == 0.0f);
					REQUIRE(normals[i].z == 0.0f);
				}
			}
		}
	}
	GIVEN("a valid point cloud without normals obj file") {
		std::ostringstream oss{};
		oss << "v 1.1 2.2 3.3\n"
			<< "v 1.1 2.2 3.3\n"
			<< "v 1.1 2.2 3.3\n";

		std::istringstream iss{ oss.str() };
		WHEN("parsing the obj's vertices") {
			auto [vertices, normals] = pcp::io::read_obj<float, float>(iss);
			THEN("the correct vertices are recovered and there are no normals") {
				REQUIRE(vertices.size() == 3u);
				REQUIRE(normals.empty());
				for (auto i = 0; i < 3; ++i)
				{
					REQUIRE(vertices[i].x == 1.1f);
					REQUIRE(vertices[i].y == 2.2f);
					REQUIRE(vertices[i].z == 3.3f);
				}
			}
		}
	}
	GIVEN("a vector of points") {
		auto const num_points = 3u;
		std::vector<pcp::point_t> vertices(num_points, pcp::point_t{ 1.0f, 1.0f, 1.0f });
		std::ostringstream oss{};

		WHEN("writing the vector of points as an obj file") {
			pcp::io::write_obj<float, float>(vertices, {}, oss);
			THEN("the resulting obj file is valid and encodes all points") {
				std::ostringstream truth_stream{};
				for (auto i = 0; i < num_points; ++i)
				{
					truth_stream << "v " 
						<< std::to_string(1.f) << " " 
						<< std::to_string(1.f) << " " 
						<< std::to_string(1.f) << "\n";
				}

				std::string const truth = truth_stream.str();
				std::string const obj = oss.str();
				REQUIRE(obj == truth);
			}
		}
		GIVEN("a vector of normals") {
			std::vector<pcp::normal_t> normals(num_points, pcp::normal_t{ 1.0f, 0.0f, 0.0f });
			WHEN("writing the vector of points and normals as an obj file") {
				pcp::io::write_obj<float, float>(vertices, normals, oss);
				THEN("the resulting obj file is valid and encodes all points with normals") {
					std::ostringstream truth_stream{};
					for (auto i = 0; i < num_points; ++i)
					{
						truth_stream << "v "
							<< std::to_string(1.f) << " "
							<< std::to_string(1.f) << " "
							<< std::to_string(1.f) << "\n";
						truth_stream << "vn "
							<< std::to_string(1.f) << " "
							<< std::to_string(0.f) << " "
							<< std::to_string(0.f) << "\n";
					}

					std::string const truth = truth_stream.str();
					std::string const obj = oss.str();
					REQUIRE(obj == truth);
				}
			}
		}
	}
}
