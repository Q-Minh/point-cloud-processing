# Point Cloud Processing Toolkit

## Overview

Toolkit of common point cloud processing algorithms.

## Dependencies

To build with `-DBUILD_TESTS=ON`, install [Catch2](https://github.com/catchorg/Catch2).
```
# using vcpkg, for example
$ vcpkg.exe install catch2
```
To build with `-DBUILD_BENCHMARKS=ON`, install [Google Benchmark](https://github.com/google/benchmark)
```
# using vcpkg, for example
$ vcpkg.exe install benchmark
```
To visualize the benchmark results with Python 3
```
$ pip install -r scripts/requirements.txt
```

## Building
Build tests and benchmarks using [CMake](https://cmake.org/):
```
$ cd <path to repo>
$ mkdir build
$ cd build
$ cmake .. # -G <generator> -DCMAKE_BUILD_TYPE=Release|Debug -DCMAKE_TOOLCHAIN_FILE=<path to vcpkg>/scripts/buildsystems/vcpkg.cmake -DBUILD_TESTS=ON|OFF -DBUILD_BENCHMARKS=ON|OFF
$ cmake --build .
```

## Installing
```
$ cd <path to repo>
$ cd build
$ cmake --install .
```
If the tests and/or benchmarks were built, the tests and/or benchmarks executables will also be installed.

## Usage
In your CMake project's CMakeLists.txt:
```
find_package(pcp CONFIG REQUIRED)
target_link_library(<your target> PRIVATE|PUBLIC|INTERFACE pcp::pcp)
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

Find more usage examples [here](./test/)

## Running
Run tests, benchmarks and visualize benchmark results:
```
# the executables will either be in ./build/Release/ or ./build/ depending on the platform
# we will assume the executables are found in ./build/Release/

# to run the tests
$ ./build/Release/tests.exe # pass in any catch2 command line arguments additionally

# to run the benchmarks and export them to benchmarks.json
$ ./build/Release/benchmarks --benchmark_format=json --benchmark_out=benchmarks.json
```

## Documentation

See [docs](./docs/)
