# Point Cloud Processing Toolkit

## Overview

`pcp` is a toolkit of common point cloud processing algorithms.

## Prerequisites

- [CMake](https://cmake.org/)
- [C++17](https://en.cppreference.com/w/cpp/17)

## Configuring

| CMake Option | Help |
| --- | --- |
| `-DPCP_BUILD_TESTS` | If set to `ON`, downloads [Catch2](https://github.com/catchorg/Catch2) and builds the `tests` target |
| `-DPCP_BUILD_BENCHMARKS` | If set to `ON`, downloads [Google Benchmark](https://github.com/google/benchmark) and builds the `benchmarks` target |
| `-DCMAKE_BUILD_TYPE` | Set to one of `Release\|Debug\|RelWithDebInfo`. If you are using a Visual Studio [generator](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html), all configuration types are included. See the [docs](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html). |  
  
```
$ cd <path to repo>
$ mkdir build
$ cd build
$ cmake .. -DPCP_BUILD_TESTS=ON -DPCP_BUILD_BENCHMARKS=ON
```

## Building

| CMake Targets | Help |
| --- | --- |
| `tests` | The [Catch2](https://github.com/catchorg/Catch2) test runner executable. Commands line options are documented [here](https://github.com/catchorg/Catch2/blob/devel/docs/command-line.md#top). |
| `benchmarks` | The [Google Benchmark](https://github.com/google/benchmark) benchmark runner executable. Command line options are documented [here](https://github.com/google/benchmark#command-line). |
| `pcp` | The `pcp` library. Currently header-only, so building is a no-op. |

```
$ cd <path to repo>
$ cd build
$ cmake --build . --target <target to build>
```

## Installing
```
$ cd <path to repo>
$ cd build
$ cmake --install .
```

## Usage
In your CMake project's CMakeLists.txt:
```
find_package(pcp CONFIG REQUIRED)
target_link_library(<your target> PRIVATE pcp::pcp)
```

Single include:
```
#include <pcp.hpp>
```

Multiple includes:
```
#include <pcp/octree.hpp>
// #include <pcp/${some_other_header}>
```

Find code usage examples [here](./test/)

## Running
Run tests, benchmarks and visualize benchmark results:
```
# In the build tree, the executables will either be in "./build/Release/" or "./build/" depending on the platform.
# In the install tree, you can find the executables in "/bin".

# to run the tests
$ <path to executable>/tests.exe

# to run the benchmarks and export them to benchmarks.json
$ <path to executable>/benchmarks --benchmark_format=json --benchmark_out=benchmarks.json
```

## Documentation

See [docs](./doc/)
