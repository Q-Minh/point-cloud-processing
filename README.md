# Point Cloud Processing Toolkit

| Windows | Ubuntu | MacOS |
| --- | --- | --- |
| [![Build status](https://ci.appveyor.com/api/projects/status/vlci0d4xfeo0p4y9/branch/master?svg=true)](https://ci.appveyor.com/project/Q-Minh/point-cloud-processing/branch/master) | [![Build status](https://ci.appveyor.com/api/projects/status/vto8v29cpp5v9jrt/branch/master?svg=true)](https://ci.appveyor.com/project/Q-Minh/point-cloud-processing-05x2j/branch/master) | [![Build status](https://ci.appveyor.com/api/projects/status/hnb1af22xdu51vtv/branch/master?svg=true)](https://ci.appveyor.com/project/Q-Minh/point-cloud-processing-m4p2m/branch/master) |
   
[![codecov](https://codecov.io/gh/Q-Minh/point-cloud-processing/branch/master/graph/badge.svg?token=ICLU539TV2)](https://codecov.io/gh/Q-Minh/point-cloud-processing) 
[![License](https://img.shields.io/badge/License-Boost%201.0-lightblue.svg)](https://www.boost.org/LICENSE_1_0.txt)

## Overview

`pcp` is a toolkit of common point cloud processing algorithms.

## Prerequisites

- [CMake](https://cmake.org/)
- [C++17](https://en.cppreference.com/w/cpp/17)

## Configuring

| CMake Option | Help |
| --- | --- |
| `-DPCP_BUILD_TESTS` | If set to `ON`, downloads [Catch2](https://github.com/catchorg/Catch2) and builds the `pcp-tests` target |
| `-DPCP_BUILD_BENCHMARKS` | If set to `ON`, downloads [Google Benchmark](https://github.com/google/benchmark) and builds the `pcp-benchmarks` target |
| `-DPCP_BUILD_EXAMPLES` | If set to `ON`, builds the examples. The examples use libigl and imgui as cmake subprojects. |
| `-DCMAKE_BUILD_TYPE` | Set to one of `Release\|Debug\|RelWithDebInfo`. If you are using a Visual Studio [generator](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html), all configuration types are included. See the [docs](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html). |  
  
```
$ mkdir build
$ cmake -S . -B build -DPCP_BUILD_TESTS=ON -DPCP_BUILD_BENCHMARKS=ON -DPCP_BUILD_EXAMPLES=ON
```

## Building

| CMake Target | Help |
| --- | --- |
| `pcp-tests` | The [Catch2](https://github.com/catchorg/Catch2) test runner executable. Commands line options are documented [here](https://github.com/catchorg/Catch2/blob/devel/docs/command-line.md#top). |
| `pcp-benchmarks` | The [Google Benchmark](https://github.com/google/benchmark) benchmark runner executable. Command line options are documented [here](https://github.com/google/benchmark#command-line). |
| `pcp` | The `pcp` library. Currently header-only, so building is a no-op. |
| `pcp-example-ply` | Ply read/write example. |
| `pcp-example-normals-estimation` | Reads a ply point cloud and exports it back with normals. |
| `pcp-example-tangent-plane-surface-reconstruction` | Reads ply point cloud and performs surface reconstruction using tangent plane SDFs. Uses libigl and imgui for visualization. |
| `pcp-example-add-noise-to-point-cloud` | Adds gaussian noise to a ply point cloud. |
| `pcp-example-filter-point-cloud-by-density` | Filters a ply point cloud using a density threshold. |
```
$ cmake --build build --target <target_name> # --config Release|Debug if you're on Windows
```

## Installing
```
$ cmake --install build
```

## Usage
In your CMake project's CMakeLists.txt:
```
find_package(pcp CONFIG REQUIRED)
target_link_library(<your target> PRIVATE pcp::pcp)
```

Single include:
```
#include <execution>
#include <filesystem>
#include <pcp.hpp>

int main(int argc, char** argv)
{
    // read in point cloud
    std::filesystem::path input_ply{argv[1]};
    auto [points, normals] = pcp::io::read_ply<pcp::point_t, pcp::normal_t>(input_ply);

    // setup acceleration structure
    pcp::octree_t octree{points.cbegin(), points.cend()};

    // compute point densities in parallel in 0.01f radius balls
    std::vector<float> density(points.size(), 0.f);
    std::transform(std::execution::par, points.cbegin(), points.cend(), density.begin(), [&](auto const& p) {
        pcp::common::sphere_t sphere{};
        sphere.radius = 0.01f;
        sphere.position = p;
        auto const points_in_range = octree.range_search(sphere);
        return static_cast<float>(points_in_range.size()) / (4.f/3.f*3.14159f*sphere.radius*sphere.radius*sphere.radius);
    });

    // do something else ...
}
```

Multiple includes:
```
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_tangent_planes.hpp>
#include <pcp/algorithm/surface_nets.hpp>
#include <pcp/common/points/point_view.hpp>
#include <pcp/octree/octree.hpp>
#include <pcp/graph/vertex.hpp>
```

### Examples
You can find more usage examples of `pcp` [here](./examples/).

### Tests
Explore `pcp`'s tests [here](./test/) for even more usage examples. The tests use [Catch2](https://github.com/catchorg/Catch2).

## Running
Run tests, benchmarks and visualize benchmark results:
```
# In the build tree, the executables will either be in "./build/Release/" or "./build/" depending on the platform.
# In the install tree, you can find the executables in "/bin".

# to run the tests
$ <path to executable>/pcp-tests.exe

# to run the benchmarks and export them to benchmarks.json
$ <path to executable>/pcp-benchmarks.exe --benchmark_format=json --benchmark_out=benchmarks.json
```

## Documentation

See [docs](./doc/)
