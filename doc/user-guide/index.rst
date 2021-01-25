.. toctree::
   :maxdepth: 2

User Guide
==========

Prerequisites
-------------

- `CMake <https://cmake.org/>`__
- `C++17 <https://en.cppreference.com/w/cpp/17>`__

Dependencies
------------

- `Eigen <http://eigen.tuxfamily.org/index.php?title=Main_Page>`__
- `range-v3 <https://github.com/ericniebler/range-v3>`__

Configuration
-------------

+---------------------------+-------------------------------------------------------------------------------------+
| CMake Option              | Help                                                                                |
+===========================+=====================================================================================+
| -DPCP_BUILD_TESTS         | If set to ``ON``, downloads                                                         |
|                           | `Catch2 <https://github.com/catchorg/Catch2>`__ and builds the                      |
|                           | ``pcp-tests`` target                                                                |
+---------------------------+-------------------------------------------------------------------------------------+
| -DPCP_BUILD_BENCHMARKS    | If set to ``ON``, downloads                                                         |
|                           | `Google Benchmark <https://github.com/google/benchmark>`__ and                      |
|                           | builds the ``pcp-benchmarks`` target                                                |
+---------------------------+-------------------------------------------------------------------------------------+
| -DPCP_BUILD_EXAMPLES      | If set to ``ON``, builds the examples. The examples use libigl and                  |
|                           | imgui as cmake subprojects.                                                         |
+---------------------------+-------------------------------------------------------------------------------------+
| -DPCP_BUILD_DOC           | If set to ``ON``, builds the documentation.                                         |
+---------------------------+-------------------------------------------------------------------------------------+
| -DCMAKE_BUILD_TYPE        | Set to one of ``Release | Debug | RelWithDebInfo``. If you are using a Visual       |
|                           |                                                                                     |
|                           | Studio                                                                              |
|                           | `generator <https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html>`__, |
|                           | all configuration types are included. See the                                       |
|                           | `docs <https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html>`__.      |
+---------------------------+-------------------------------------------------------------------------------------+
  
.. code-block:: bash

   $ mkdir build
   $ cmake -S . -B build -DPCP_BUILD_TESTS=ON -DPCP_BUILD_BENCHMARKS=ON -DPCP_BUILD_EXAMPLES=ON


Building
--------

+------------------------------------------------------+-------------------------------------------------------------------------------------+
| CMake Target                                         | Help                                                                                |
+======================================================+=====================================================================================+
| ``pcp``                                              | The ``pcp`` library. Currently header-only, so building is a no-op.                 |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-tests``                                        | The `Catch2 <https://github.com/catchorg/Catch2>`__ test runner executable.         |
|                                                      | Commands line options are documented                                                |
|                                                      | `here <https://github.com/catchorg/Catch2/blob/devel/docs/command-line.md#top>`__.  |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-benchmarks``                                   | The `Google Benchmark <https://github.com/google/benchmark>`__ benchmark runner     |
|                                                      | executable. Command line options are documented                                     |
|                                                      | `here <https://github.com/google/benchmark#command-line>`__.                        |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-doxygen``                                      | Generate sphinx documentation to ./build/doc/sphinx/index.html. Runs doxygen        |
|                                                      | previously automatically if changes were detected.                                  |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-sphinx``                                       | Generate doxygen documentation to ./build/doc/doxygen/html/index.html.              |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-example-ply``                                  | Ply read/write example.                                                             |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-example-normals-estimation``                   | Reads a ply point cloud and exports it back with normals.                           |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-example-tangent-plane-surface-reconstruction`` | Reads ply point cloud and performs surface reconstruction using tangent plane SDFs. |
|                                                      |                                                                                     |
|                                                      | Uses libigl and imgui for visualization.                                            |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-example-add-noise-to-point-cloud``             | Adds gaussian noise to a ply point cloud.                                           |
+------------------------------------------------------+-------------------------------------------------------------------------------------+
| ``pcp-example-filter-point-cloud-by-density``        | Filters a ply point cloud using a density threshold.                                |
+------------------------------------------------------+-------------------------------------------------------------------------------------+

.. code-block:: bash
   
   $ cmake --build build --target <target_name> # --config Release|Debug if you're on Windows


Installing
----------

.. code-block:: bash

   $ cmake --install build

Usage
-----

In your CMake project's CMakeLists.txt:

.. code-block:: cmake

   find_package(pcp CONFIG REQUIRED)
   target_link_library(<your target> PRIVATE pcp::pcp)

Examples
--------

I/O
~~~

.. literalinclude:: ../../examples/ply.cpp
   :language: cpp

Normals Estimation
~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../examples/normals_estimation.cpp
   :language: cpp

Filtering
~~~~~~~~~

.. literalinclude:: ../../examples/filter_point_cloud_noise_by_density.cpp
   :language: cpp
   :lines: 1-29,33,35,38-48,50,53-76,81-91,93,96,98,107-108
