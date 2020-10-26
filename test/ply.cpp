#include <catch2/catch.hpp>
#include <pcp/normal.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/point.hpp>

static std::string get_ascii_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    bool const with_normals = true);

static std::string get_binary_big_endian_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    bool const with_normals = true);

static std::string get_binary_little_endian_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    bool const with_normals = true);

SCENARIO("ply file manipulation", "[ply]")
{
    float const x = 1.1f, y = 2.2f, z = 3.3f;
    float const nx = 1.f, ny = 0.f, nz = 0.f;
    pcp::point_t const p{x, y, z};
    pcp::normal_t const n{nx, ny, nz};
    auto const num_points = 3u;
    GIVEN("a valid point cloud ply file")
    {
        auto const validate_vertices = [=](std::istream& is) {
            WHEN("parsing the ply's vertices")
            {
                auto [vertices, normals] = pcp::io::read_ply<pcp::point_t, pcp::normal_t>(is);
                THEN("the correct vertices are recovered")
                {
                    REQUIRE(normals.empty());
                    REQUIRE(vertices.size() == num_points);
                    for (std::size_t i = 0; i < 3u; ++i)
                    {
                        REQUIRE(vertices[i] == p);
                    }
                }
            }
        };
        auto const validate_vertices_and_normals = [=](std::istream& is) {
            WHEN("parsing the ply's vertices and normals")
            {
                auto [vertices, normals] = pcp::io::read_ply<pcp::point_t, pcp::normal_t>(is);
                THEN("the correct vertices and normals are recovered")
                {
                    REQUIRE(vertices.size() == num_points);
                    REQUIRE(normals.size() == num_points);
                    for (std::size_t i = 0; i < 3u; ++i)
                    {
                        REQUIRE(vertices[i] == p);
                        REQUIRE(normals[i] == n);
                    }
                }
            }
        };

        GIVEN("the file is in ascii format")
        {
            std::istringstream iss{get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, false)};
            validate_vertices(iss);
        }
        GIVEN("the file is in binary little endian format")
        {
            std::istringstream iss{
                get_binary_little_endian_ply_file(num_points, x, y, z, nx, ny, nz, false)};
            validate_vertices(iss);
        }
        GIVEN("the file is in binary big endian format")
        {
            std::istringstream iss{
                get_binary_big_endian_ply_file(num_points, x, y, z, nx, ny, nz, false)};
            validate_vertices(iss);
        }
        GIVEN("vertex normals in the ply file")
        {
            GIVEN("the file is in ascii format")
            {
                std::istringstream iss{get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, true)};
                validate_vertices_and_normals(iss);
            }
            GIVEN("the file is in binary little endian format")
            {
                std::istringstream iss{
                    get_binary_little_endian_ply_file(num_points, x, y, z, nx, ny, nz, true)};
                validate_vertices_and_normals(iss);
            }
            GIVEN("the file is in binary big endian format")
            {
                std::istringstream iss{
                    get_binary_big_endian_ply_file(num_points, x, y, z, nx, ny, nz, true)};
                validate_vertices_and_normals(iss);
            }
        }
    }
    GIVEN("a vector of points")
    {
        std::vector<pcp::point_t> vertices(num_points, pcp::point_t{x, y, z});
        std::ostringstream oss{};

        WHEN("writing the vector of points as a ply file")
        {
            WHEN("writing in ascii format")
            {
                pcp::io::write_ply<pcp::point_t, pcp::normal_t>(
                    oss,
                    vertices,
                    {},
                    pcp::io::ply_format_t::ascii);
                THEN("the resulting ply file is valid and encodes all points")
                {
                    std::string const truth =
                        get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, false);
                    std::string const ply = oss.str();
                    REQUIRE(ply == truth);
                }
            }
            WHEN("writing in binary_little_endian format")
            {
                pcp::io::write_ply<pcp::point_t, pcp::normal_t>(
                    oss,
                    vertices,
                    {},
                    pcp::io::ply_format_t::binary_little_endian);
                THEN("the resulting ply file is valid and encodes all points")
                {
                    std::string const truth =
                        get_binary_little_endian_ply_file(num_points, x, y, z, nx, ny, nz, false);
                    std::string const ply = oss.str();
                    REQUIRE(ply == truth);
                }
            }
            WHEN("writing in binary_big_endian format")
            {
                pcp::io::write_ply<pcp::point_t, pcp::normal_t>(
                    oss,
                    vertices,
                    {},
                    pcp::io::ply_format_t::binary_big_endian);
                THEN("the resulting ply file is valid and encodes all points")
                {
                    std::string const truth =
                        get_binary_big_endian_ply_file(num_points, x, y, z, nx, ny, nz, false);
                    std::string const ply = oss.str();
                    REQUIRE(ply == truth);
                }
            }
        }
        GIVEN("a vector of normals")
        {
            std::vector<pcp::normal_t> normals(num_points, pcp::normal_t{nx, ny, nz});
            WHEN("writing the vector of points and normals as a ply file")
            {
                WHEN("writing in ascii format")
                {
                    pcp::io::write_ply<pcp::point_t, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        pcp::io::ply_format_t::ascii);
                    THEN("the resulting ply file is valid and encodes all points")
                    {
                        std::string const truth =
                            get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, true);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
                WHEN("writing in binary_little_endian format")
                {
                    pcp::io::write_ply<pcp::point_t, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        pcp::io::ply_format_t::binary_little_endian);
                    THEN("the resulting ply file is valid and encodes all points")
                    {
                        std::string const truth = get_binary_little_endian_ply_file(
                            num_points,
                            x,
                            y,
                            z,
                            nx,
                            ny,
                            nz,
                            true);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
                WHEN("writing in binary_big_endian format")
                {
                    pcp::io::write_ply<pcp::point_t, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        pcp::io::ply_format_t::binary_big_endian);
                    THEN("the resulting ply file is valid and encodes all points")
                    {
                        std::string const truth =
                            get_binary_big_endian_ply_file(num_points, x, y, z, nx, ny, nz, true);
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
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
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

    for (std::size_t i = 0; i < num_points; ++i)
        oss << std::to_string(x) << " " << std::to_string(y) << " " << std::to_string(z) << "\n";

    if (!with_normals)
        return oss.str();

    for (std::size_t i = 0; i < num_points; ++i)
        oss << std::to_string(nx) << " " << std::to_string(ny) << " " << std::to_string(nz) << "\n";

    return oss.str();
}

static std::string get_binary_little_endian_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
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

    auto const reverse_endianness_if_needed = [](float v) {
        return !pcp::is_machine_little_endian() ? pcp::reverse_endianness(v) : v;
    };
    for (std::size_t i = 0; i < num_points; ++i)
    {
        float const reversed_coords[3] = {
            reverse_endianness_if_needed(x),
            reverse_endianness_if_needed(y),
            reverse_endianness_if_needed(z)};
        oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
    }

    if (!with_normals)
        return oss.str();

    for (std::size_t i = 0; i < num_points; ++i)
    {
        float const reversed_coords[3] = {
            reverse_endianness_if_needed(nx),
            reverse_endianness_if_needed(ny),
            reverse_endianness_if_needed(nz)};
        oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
    }
    return oss.str();
}

static std::string get_binary_big_endian_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
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

    auto const reverse_endianness_if_needed = [](float v) {
        return !pcp::is_machine_big_endian() ? pcp::reverse_endianness(v) : v;
    };
    for (std::size_t i = 0; i < num_points; ++i)
    {
        float const reversed_coords[3] = {
            reverse_endianness_if_needed(x),
            reverse_endianness_if_needed(y),
            reverse_endianness_if_needed(z)};
        oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
    }

    if (!with_normals)
        return oss.str();

    for (std::size_t i = 0; i < num_points; ++i)
    {
        float const reversed_coords[3] = {
            reverse_endianness_if_needed(nx),
            reverse_endianness_if_needed(ny),
            reverse_endianness_if_needed(nz)};
        oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
    }

    return oss.str();
}
