#include "custom_point.hpp"

#include <catch2/catch.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/io/ply.hpp>

static std::string get_ascii_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    std::uint8_t r,
    std::uint8_t g,
    std::uint8_t b,
    bool const with_normals = true,
    bool const with_colors  = false);

static std::string get_binary_big_endian_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    std::uint8_t r,
    std::uint8_t g,
    std::uint8_t b,
    bool const with_normals = true,
    bool const with_colors  = false);

static std::string get_binary_little_endian_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    std::uint8_t r,
    std::uint8_t g,
    std::uint8_t b,
    bool const with_normals = true,
    bool const with_colors  = false);

#if defined(_MSC_VER)
    #pragma warning(push)
    #pragma warning(disable : 26444)
#endif
TEMPLATE_TEST_CASE(
    "ply file manipulation",
    "[ply][template]",
    pcp::point_t,
    pcp::test::custom_point_t)
{
    float const x = 1.1f, y = 2.2f, z = 3.3f;
    float const nx = 1.f, ny = 0.f, nz = 0.f;
    std::uint8_t r = static_cast<std::uint8_t>(255u), g = static_cast<std::uint8_t>(0u),
                 b   = static_cast<std::uint8_t>(125u);
    using point_type = TestType;
    point_type const p{x, y, z};
    pcp::normal_t const n{nx, ny, nz};
    auto const num_points = 3u;
    GIVEN("a valid point cloud ply file")
    {
        auto const validate_vertices = [=](std::istream& is) {
            WHEN("parsing the ply's vertices")
            {
                auto [vertices, normals, _] = pcp::io::read_ply<point_type, pcp::normal_t>(is);
                THEN("the correct vertices are recovered")
                {
                    REQUIRE(normals.empty());
                    REQUIRE(vertices.size() == num_points);
                    for (std::size_t i = 0; i < 3u; ++i)
                    {
                        REQUIRE(pcp::common::are_vectors_equal(vertices[i], p));
                    }
                }
            }
        };
        auto const validate_vertices_and_normals = [=](std::istream& is) {
            WHEN("parsing the ply's vertices and normals")
            {
                auto [vertices, normals, _] = pcp::io::read_ply<point_type, pcp::normal_t>(is);
                THEN("the correct vertices and normals are recovered")
                {
                    REQUIRE(vertices.size() == num_points);
                    REQUIRE(normals.size() == num_points);
                    for (std::size_t i = 0; i < 3u; ++i)
                    {
                        REQUIRE(pcp::common::are_vectors_equal(vertices[i], p));
                        REQUIRE(pcp::common::are_vectors_equal(normals[i], n));
                    }
                }
            }
        };
        auto const validate_vertices_and_normals_and_colors = [=](std::istream& is) {
            WHEN("parsing the ply's vertices, normals and colors")
            {
                auto [vertices, normals, colors] = pcp::io::read_ply<point_type, pcp::normal_t>(is);
                THEN("the correct vertices, normals and colors are recovered")
                {
                    REQUIRE(vertices.size() == num_points);
                    REQUIRE(normals.size() == num_points);
                    REQUIRE(colors.size() == num_points);
                    for (std::size_t i = 0; i < 3u; ++i)
                    {
                        REQUIRE(pcp::common::are_vectors_equal(vertices[i], p));
                        REQUIRE(pcp::common::are_vectors_equal(normals[i], n));
                        REQUIRE(std::get<0>(colors[i]) == r);
                        REQUIRE(std::get<1>(colors[i]) == g);
                        REQUIRE(std::get<2>(colors[i]) == b);
                    }
                }
            }
        };

        GIVEN("the file is in ascii format")
        {
            std::istringstream iss{
                get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, r, g, b, false, false)};
            validate_vertices(iss);
        }
        GIVEN("the file is in binary little endian format")
        {
            std::istringstream iss{get_binary_little_endian_ply_file(
                num_points,
                x,
                y,
                z,
                nx,
                ny,
                nz,
                r,
                g,
                b,
                false,
                false)};
            validate_vertices(iss);
        }
        GIVEN("the file is in binary big endian format")
        {
            std::istringstream iss{get_binary_big_endian_ply_file(
                num_points,
                x,
                y,
                z,
                nx,
                ny,
                nz,
                r,
                g,
                b,
                false,
                false)};
            validate_vertices(iss);
        }
        GIVEN("vertex normals in the ply file")
        {
            GIVEN("the file is in ascii format")
            {
                std::istringstream iss{
                    get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, r, g, b, true, false)};
                validate_vertices_and_normals(iss);
            }
            GIVEN("the file is in binary little endian format")
            {
                std::istringstream iss{get_binary_little_endian_ply_file(
                    num_points,
                    x,
                    y,
                    z,
                    nx,
                    ny,
                    nz,
                    r,
                    g,
                    b,
                    true,
                    false)};
                validate_vertices_and_normals(iss);
            }
            GIVEN("the file is in binary big endian format")
            {
                std::istringstream iss{get_binary_big_endian_ply_file(
                    num_points,
                    x,
                    y,
                    z,
                    nx,
                    ny,
                    nz,
                    r,
                    g,
                    b,
                    true,
                    false)};
                validate_vertices_and_normals(iss);
            }
            GIVEN("vertex colors in the ply file")
            {
                GIVEN("the file is in ascii format")
                {
                    std::istringstream iss{
                        get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, r, g, b, true, true)};
                    validate_vertices_and_normals_and_colors(iss);
                }
                GIVEN("the file is in binary little endian format")
                {
                    std::istringstream iss{get_binary_little_endian_ply_file(
                        num_points,
                        x,
                        y,
                        z,
                        nx,
                        ny,
                        nz,
                        r,
                        g,
                        b,
                        true,
                        true)};
                    validate_vertices_and_normals_and_colors(iss);
                }
                GIVEN("the file is in binary big endian format")
                {
                    std::istringstream iss{get_binary_big_endian_ply_file(
                        num_points,
                        x,
                        y,
                        z,
                        nx,
                        ny,
                        nz,
                        r,
                        g,
                        b,
                        true,
                        true)};
                    validate_vertices_and_normals_and_colors(iss);
                }
            }
        }
    }
    GIVEN("a vector of points")
    {
        std::vector<point_type> vertices(num_points, point_type{x, y, z});
        std::ostringstream oss{};

        WHEN("writing the vector of points as a ply file")
        {
            WHEN("writing in ascii format")
            {
                pcp::io::write_ply<point_type, pcp::normal_t>(
                    oss,
                    vertices,
                    {},
                    {},
                    pcp::io::ply_format_t::ascii);
                THEN("the resulting ply file is valid and encodes all points")
                {
                    std::string const truth =
                        get_ascii_ply_file(num_points, x, y, z, nx, ny, nz, r, g, b, false, false);
                    std::string const ply = oss.str();
                    REQUIRE(ply == truth);
                }
            }
            WHEN("writing in binary_little_endian format")
            {
                pcp::io::write_ply<point_type, pcp::normal_t>(
                    oss,
                    vertices,
                    {},
                    {},
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
                        r,
                        g,
                        b,
                        false,
                        false);
                    std::string const ply = oss.str();
                    REQUIRE(ply == truth);
                }
            }
            WHEN("writing in binary_big_endian format")
            {
                pcp::io::write_ply<point_type, pcp::normal_t>(
                    oss,
                    vertices,
                    {},
                    {},
                    pcp::io::ply_format_t::binary_big_endian);
                THEN("the resulting ply file is valid and encodes all points")
                {
                    std::string const truth = get_binary_big_endian_ply_file(
                        num_points,
                        x,
                        y,
                        z,
                        nx,
                        ny,
                        nz,
                        r,
                        g,
                        b,
                        false,
                        false);
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
                    pcp::io::write_ply<point_type, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        {},
                        pcp::io::ply_format_t::ascii);
                    THEN("the resulting ply file is valid and encodes all points")
                    {
                        std::string const truth = get_ascii_ply_file(
                            num_points,
                            x,
                            y,
                            z,
                            nx,
                            ny,
                            nz,
                            r,
                            g,
                            b,
                            true,
                            false);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
                WHEN("writing in binary_little_endian format")
                {
                    pcp::io::write_ply<point_type, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        {},
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
                            r,
                            g,
                            b,
                            true,
                            false);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
                WHEN("writing in binary_big_endian format")
                {
                    pcp::io::write_ply<point_type, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        {},
                        pcp::io::ply_format_t::binary_big_endian);
                    THEN("the resulting ply file is valid and encodes all points")
                    {
                        std::string const truth = get_binary_big_endian_ply_file(
                            num_points,
                            x,
                            y,
                            z,
                            nx,
                            ny,
                            nz,
                            r,
                            g,
                            b,
                            true,
                            false);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
            }
            GIVEN("a vector of colors")
            {
                using color_type = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;
                std::vector<color_type> colors(num_points, color_type{r, g, b});

                WHEN("writing in ascii format")
                {
                    pcp::io::write_ply<point_type, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        colors,
                        pcp::io::ply_format_t::ascii);
                    THEN("the resulting ply file is valid and encodes all points")
                    {
                        std::string const truth = get_ascii_ply_file(
                            num_points,
                            x,
                            y,
                            z,
                            nx,
                            ny,
                            nz,
                            r,
                            g,
                            b,
                            true,
                            true);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
                WHEN("writing in binary_little_endian format")
                {
                    pcp::io::write_ply<point_type, pcp::normal_t>(
                        oss,
                        vertices,
                        normals,
                        colors,
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
                            r,
                            g,
                            b,
                            true,
                            true);
                        std::string const ply = oss.str();
                        REQUIRE(ply == truth);
                    }
                }
                // In the "truth" binary big endian ply string, we have written red,green,blue
                // property names instead of r,g,b and we have written them before normals.
                // This was done to test if our parsers could handle these different cases.
                // As such, we cannot use these same truth strings to test our ply writing
                // for the binary big endian format, since we always write with the same order,
                // that is positions, normals and colors, with r,g,b property names.
                // WHEN("writing in binary_big_endian format")
                //{
                //    pcp::io::write_ply<point_type, pcp::normal_t>(
                //        oss,
                //        vertices,
                //        normals,
                //        pcp::io::ply_format_t::binary_big_endian);
                //    THEN("the resulting ply file is valid and encodes all points")
                //    {
                //        std::string const truth = get_binary_big_endian_ply_file(
                //            num_points,
                //            x,
                //            y,
                //            z,
                //            nx,
                //            ny,
                //            nz,
                //            r,
                //            g,
                //            b,
                //            true,
                //            false);
                //        std::string const ply = oss.str();
                //        REQUIRE(ply == truth);
                //    }
                //}
            }
        }
    }
}
#if defined(_MSC_VER)
    #pragma warning(pop)
#endif

static std::string get_ascii_ply_file(
    std::size_t num_points,
    float x,
    float y,
    float z,
    float nx,
    float ny,
    float nz,
    std::uint8_t r,
    std::uint8_t g,
    std::uint8_t b,
    bool const with_normals,
    bool const with_colors)
{
    std::ostringstream oss{};
    oss << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << num_points << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n";

    if (with_normals)
    {
        oss << "property float nx\n"
            << "property float ny\n"
            << "property float nz\n";
    }
    if (with_colors)
    {
        oss << "property uchar r\n"
            << "property uchar g\n"
            << "property uchar b\n";
    }
    oss << "end_header\n";

    for (std::size_t i = 0; i < num_points; ++i)
    {
        oss << std::to_string(x) << " " << std::to_string(y) << " " << std::to_string(z);
        if (with_normals)
        {
            oss << " " << std::to_string(nx) << " " << std::to_string(ny) << " "
                << std::to_string(nz);
        }
        if (with_colors)
        {
            oss << " " << std::to_string(r) << " " << std::to_string(g) << " " << std::to_string(b);
        }
        oss << "\n";
    }

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
    std::uint8_t r,
    std::uint8_t g,
    std::uint8_t b,
    bool const with_normals,
    bool const with_colors)
{
    std::ostringstream oss{};
    oss << "ply\n"
        << "format binary_little_endian 1.0\n"
        << "element vertex " << num_points << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n";

    if (with_normals)
    {
        oss << "property float nx\n"
            << "property float ny\n"
            << "property float nz\n";
    }
    if (with_colors)
    {
        oss << "property uchar r\n"
            << "property uchar g\n"
            << "property uchar b\n";
    }
    oss << "end_header\n";

    auto const reverse_endianness_if_needed = [](float v) {
        return !pcp::is_machine_little_endian() ? pcp::reverse_endianness(v) : v;
    };
    auto const reverse_endianness_if_needed_uchar = [](std::uint8_t v) {
        return !pcp::is_machine_little_endian() ? pcp::reverse_endianness(v) : v;
    };
    for (std::size_t i = 0; i < num_points; ++i)
    {
        {
            float const reversed_coords[3] = {
                reverse_endianness_if_needed(x),
                reverse_endianness_if_needed(y),
                reverse_endianness_if_needed(z)};
            oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
        }
        if (with_normals)
        {
            float const reversed_coords[3] = {
                reverse_endianness_if_needed(nx),
                reverse_endianness_if_needed(ny),
                reverse_endianness_if_needed(nz)};
            oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
        }
        if (with_colors)
        {
            std::uint8_t const reversed_colors[3] = {
                reverse_endianness_if_needed_uchar(r),
                reverse_endianness_if_needed_uchar(g),
                reverse_endianness_if_needed_uchar(b)};
            oss.write(reinterpret_cast<char const*>(reversed_colors), sizeof(reversed_colors));
        }
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
    std::uint8_t r,
    std::uint8_t g,
    std::uint8_t b,
    bool const with_normals,
    bool const with_colors)
{
    std::ostringstream oss{};
    oss << "ply\n"
        << "format binary_big_endian 1.0\n"
        << "element vertex " << num_points << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n";

    if (with_colors)
    {
        oss << "property uchar red\n"
            << "property uchar green\n"
            << "property uchar blue\n";
    }
    if (with_normals)
    {
        oss << "property float nx\n"
            << "property float ny\n"
            << "property float nz\n";
    }
    oss << "end_header\n";

    auto const reverse_endianness_if_needed = [](float v) {
        return !pcp::is_machine_big_endian() ? pcp::reverse_endianness(v) : v;
    };
    auto const reverse_endianness_if_needed_uchar = [](std::uint8_t v) {
        return !pcp::is_machine_big_endian() ? pcp::reverse_endianness(v) : v;
    };

    for (std::size_t i = 0; i < num_points; ++i)
    {
        {
            float const reversed_coords[3] = {
                reverse_endianness_if_needed(x),
                reverse_endianness_if_needed(y),
                reverse_endianness_if_needed(z)};
            oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
        }
        if (with_colors)
        {
            std::uint8_t const reversed_colors[3] = {
                reverse_endianness_if_needed_uchar(r),
                reverse_endianness_if_needed_uchar(g),
                reverse_endianness_if_needed_uchar(b)};
            oss.write(reinterpret_cast<char const*>(reversed_colors), sizeof(reversed_colors));
        }
        if (with_normals)
        {
            float const reversed_coords[3] = {
                reverse_endianness_if_needed(nx),
                reverse_endianness_if_needed(ny),
                reverse_endianness_if_needed(nz)};
            oss.write(reinterpret_cast<char const*>(reversed_coords), sizeof(reversed_coords));
        }
    }

    return oss.str();
}
