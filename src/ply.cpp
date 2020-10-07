#include <catch2/catch.hpp>

#include <ply.hpp>

float const x = 1.1f, y = 2.2f, z = 3.3f;
float const nx = 1.f, ny = 0.f, nz = 0.f;

static std::string get_ascii_ply_file(
	std::size_t num_points,
	bool const with_normals = true);

static std::string get_binary_big_endian_ply_file(
	std::size_t num_points,
	bool const with_normals = true);

static std::string get_binary_little_endian_ply_file(
	std::size_t num_points,
	bool const with_normals = true);

SCENARIO("ply file manipulation", "[ply]") {
	auto const num_points = 3u;
	GIVEN("a valid point cloud ply file") {
		auto const validate_vertices = [=](std::istream& is)
		{
			WHEN("parsing the ply's vertices") {
				auto [vertices, normals] = ply::read_ply(is);
				THEN("the correct vertices are recovered") {
					REQUIRE(normals.empty());
					REQUIRE(vertices.size() == num_points);
					for (auto i = 0; i < 3; ++i)
					{
						REQUIRE(vertices[i].x == x);
						REQUIRE(vertices[i].y == y);
						REQUIRE(vertices[i].z == z);
					}
				}
			}
		};
		auto const validate_vertices_and_normals = [=](std::istream& is)
		{
			WHEN("parsing the ply's vertices and normals") {
				auto [vertices, normals] = ply::read_ply(is);
				THEN("the correct vertices and normals are recovered") {
					REQUIRE(vertices.size() == num_points);
					REQUIRE(normals.size() == num_points);
					for (auto i = 0; i < 3; ++i)
					{
						REQUIRE(vertices[i].x == x);
						REQUIRE(vertices[i].y == y);
						REQUIRE(vertices[i].z == z);
						REQUIRE(normals[i].x == nx);
						REQUIRE(normals[i].y == ny);
						REQUIRE(normals[i].z == nz);
					}
				}
			}
		};

		GIVEN("the file is in ascii format") {
			std::istringstream iss{ get_ascii_ply_file(num_points, false) };
			validate_vertices(iss);
		}
		GIVEN("the file is in binary little endian format") {
			std::istringstream iss{ get_binary_little_endian_ply_file(num_points, false) };
			validate_vertices(iss);
		}
		GIVEN("the file is in binary big endian format") {
			std::istringstream iss{ get_binary_big_endian_ply_file(num_points, false) };
			validate_vertices(iss);
		}
		GIVEN("vertex normals in the ply file") {
			GIVEN("the file is in ascii format") {
				std::istringstream iss{ get_ascii_ply_file(num_points, true) };
				validate_vertices_and_normals(iss);
			}
			GIVEN("the file is in binary little endian format") {
				std::istringstream iss{ get_binary_little_endian_ply_file(num_points, true) };
				validate_vertices_and_normals(iss);
			}
			GIVEN("the file is in binary big endian format") {
				std::istringstream iss{ get_binary_big_endian_ply_file(num_points, true) };
				validate_vertices_and_normals(iss);
			}
		}
	}
	GIVEN("a vector of points") {
		std::vector<point_t> vertices(num_points, point_t{ x, y, z });
		std::ostringstream oss{};

		WHEN("writing the vector of points as a ply file") {
			WHEN("writing in ascii format") {
				ply::write_ply<float, float>(oss, vertices, {}, ply::format_t::ascii);
				THEN("the resulting ply file is valid and encodes all points") {
					std::string const truth = get_ascii_ply_file(num_points, false);
					std::string const ply = oss.str();
					REQUIRE(ply == truth);
				}
			}
			WHEN("writing in binary_little_endian format") {

			}
			WHEN("writing in binary_big_endian format") {

			}
		}
		GIVEN("a vector of normals") {
			std::vector<normal_t> normals(num_points, normal_t{ nx, ny, nz });
			WHEN("writing the vector of points and normals as a ply file") {
				WHEN("writing in ascii format") {
					ply::write_ply<float, float>(oss, vertices, normals, ply::format_t::ascii);
					THEN("the resulting ply file is valid and encodes all points") {
						std::string const truth = get_ascii_ply_file(num_points, true);
						std::string const ply = oss.str();
						REQUIRE(ply == truth);
					}
				}
				WHEN("writing in binary_little_endian format") {
					ply::write_ply<float, float>(oss, vertices, normals, ply::format_t::binary_little_endian);
					THEN("the resulting ply file is valid and encodes all points") {
						std::string const truth = get_binary_little_endian_ply_file(num_points, true);
						std::string const ply = oss.str();
						REQUIRE(ply == truth);
					}
				}
				WHEN("writing in binary_big_endian format") {
					ply::write_ply<float, float>(oss, vertices, normals, ply::format_t::binary_big_endian);
					THEN("the resulting ply file is valid and encodes all points") {
						std::string const truth = get_binary_big_endian_ply_file(num_points, true);
						std::string const ply = oss.str();
						REQUIRE(ply == truth);
					}
				}
			}
		}
	}
}

static std::string get_ascii_ply_file(
	std::size_t num_points,
	bool const with_normals)
{
	std::ostringstream oss{};
	oss << "ply\n"
		<< "format ascii 1.0\n"
		<< "element vertex " << num_points << "\n"
		<< "property float x\n"
		<< "property float y\n"
		<< "property float z\n"
		<< "element normal " << (with_normals ? num_points : 0) << "\n"
		<< "property float nx\n"
		<< "property float ny\n"
		<< "property float nz\n"
		<< "end_header\n";

	for (auto i = 0; i < num_points; ++i)
		oss << std::to_string(x) << " " << std::to_string(y) << " " << std::to_string(z) << "\n";

	if (!with_normals)
		return oss.str();

	for (auto i = 0; i < num_points; ++i)
		oss << std::to_string(nx) << " " << std::to_string(ny) << " " << std::to_string(nz) << "\n";

	return oss.str();
}

static std::string get_binary_little_endian_ply_file(
	std::size_t num_points,
	bool const with_normals)
{
	std::ostringstream oss{};
	oss << "ply\n"
		<< "format binary_little_endian 1.0\n"
		<< "element vertex " << num_points << "\n"
		<< "property float x\n"
		<< "property float y\n"
		<< "property float z\n"
		<< "element normal " << (with_normals ? num_points : 0) << "\n"
		<< "property float nx\n"
		<< "property float ny\n"
		<< "property float nz\n"
		<< "end_header\n";

	auto const reverse_endianness_if_needed = [](float v)
	{
		return !is_machine_little_endian() ? reverse_endianness(v) : v;
	};
	for (auto i = 0; i < num_points; ++i)
	{
		float const reversed_coords[3] =
		{
			reverse_endianness_if_needed(x),
			reverse_endianness_if_needed(y),
			reverse_endianness_if_needed(z)
		};
		oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
	}

	if (!with_normals)
		return oss.str();

	for (auto i = 0; i < num_points; ++i)
	{
		float const reversed_coords[3] =
		{
			reverse_endianness_if_needed(nx),
			reverse_endianness_if_needed(ny),
			reverse_endianness_if_needed(nz)
		};
		oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
	}
	return oss.str();
}

static std::string get_binary_big_endian_ply_file(
	std::size_t num_points,
	bool const with_normals)
{
	std::ostringstream oss{};
	oss << "ply\n"
		<< "format binary_big_endian 1.0\n"
		<< "element vertex " << num_points << "\n"
		<< "property float x\n"
		<< "property float y\n"
		<< "property float z\n"
		<< "element normal " << (with_normals ? num_points : 0) << "\n"
		<< "property float nx\n"
		<< "property float ny\n"
		<< "property float nz\n"
		<< "end_header\n";

	auto const reverse_endianness_if_needed = [](float v)
	{
		return !is_machine_big_endian() ? reverse_endianness(v) : v;
	};
	for (auto i = 0; i < num_points; ++i)
	{
		float const reversed_coords[3] =
		{
			reverse_endianness_if_needed(x),
			reverse_endianness_if_needed(y),
			reverse_endianness_if_needed(z)
		};
		oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
	}

	if (!with_normals)
		return oss.str();

	for (auto i = 0; i < num_points; ++i)
	{
		float const reversed_coords[3] =
		{
			reverse_endianness_if_needed(nx),
			reverse_endianness_if_needed(ny),
			reverse_endianness_if_needed(nz)
		};
		oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
	}

	return oss.str();
}
