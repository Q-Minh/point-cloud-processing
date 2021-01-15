.. toctree::
   :maxdepth: 2

Introduction
============


Overview
--------

**pcp** is a toolkit of common point cloud processing algorithms. 

Using modern C++17 as a foundation, **pcp** integrates as much as 
possible with the STL, enabling the reuse of iterator, container 
and algorithm constructs that C++ users are accustomed to, and 
applying them to point cloud processing. In doing so, parallelization, 
composition, lazy evaluation and memory efficiency can be achieved 
trivially.

Additionally, **pcp** exposes a modern C++ template interface that 
attempts to maximize flexibility to ease integration by 
imposing relaxed type restrictions on user input 
while also providing human-readable compile errors when said type 
restrictions are not met instead of the cryptic long error 
messages one usually witnesses with template programming. This 
enables customization points at every layer of the library.

Let's take a simple example to illustrate our points:

.. highlight:: cpp

.. code-block:: cpp

   #include <execution>
   #include <filesystem>
   #include <pcp/pcp.hpp>

   int main(int argc, char** argv)
   {
      using point_type      = pcp::point_t;
      using point_view_type = pcp::point_view_t;
      using normal_type     = pcp::normal_t;

      /**
       * We take care of parsing point clouds, and you tell us 
       * which point and normal types you want the points to be 
       * parsed into.
       */
      std::filesystem::path input_ply{argv[1]};
      auto [points, normals] = pcp::io::read_ply<point_type, normal_type>(input_ply);

      /**
       * Point views adapt points and store a single pointer.
       * Here, we use ranges to defer the transform to later. 
       * We don't need to make any copies of points.
       */
      auto point_views =
         points | ranges::views::transform([](auto& point) { return point_view_type{&point}; });

      /**
       * We provide spatial query acceleration containers for you 
       * using STL-like interfaces. Any begin/end iterator pairs 
       * of points are treated as point clouds by pcp. 
       * Containers don't need to actually store the points, so 
       * we construct an octree of point views here which are 
       * evaluated on the fly. Instead of holding aligned/padded 
       * points of 3 floats or 3 doubles, we hold only a single 
       * pointer for each point in the octree, reducing memory 
       * footprint and minimizing copy bandwidth.
       */
      using octree_type = pcp::basic_linked_octree_t<point_view_type>;
      octree_type octree{point_views.begin(), point_views.end()};

      /**
       * It's easy to associate quantities to points of our point clouds 
       * because point clouds are sequential and indexed. 
       * By exposing point clouds as iterator pairs, it's trivial to integrate 
       * with the STL and parallelize operations using execution policies.
       */
      std::vector<float> density(points.size(), 0.f);
      std::transform(
         std::execution::par,
         point_views.begin(),
         point_views.end(),
         density.begin(),
         [&](auto const& p) {
               pcp::common::sphere_t sphere{};
               sphere.radius              = 0.01f;
               sphere.position            = point_type{p};
               auto const points_in_range = octree.range_search(sphere);
               auto const pi              = 3.14159f;
               auto const r3              = sphere.radius * sphere.radius * sphere.radius;
               auto const volume          = 4.f / 3.f * pi * r3;
               return static_cast<float>(points_in_range.size()) / volume;
         });

      normals.resize(points.size());

      /**
       * Since we don't force you to use our spatial query 
       * acceleration data structures, our interface requires 
       * callable types instead, so you can use any type of 
       * knn search you want.
       */
      auto const knn = [&octree](auto const& p) {
         return octree.nearest_neighbours(p, 15u);
      };

      /**
       * Our algorithms expose an STL-like interface. 
       * We see here that parallelization is trivial 
       * using C++17 execution policies.
       */
      pcp::algorithm::estimate_normals(
            std::execution::par,
            point_views.begin(),
            point_views.end(),
            normals.begin(),
            knn,
            pcp::algorithm::default_normal_transform<point_view_type, normal_type>);

      /**
       * write point cloud with normals to disk in ply format
       */
      pcp::io::write_ply(
         std::filesystem::path{argv[2]},
         points,
         normals,
         pcp::io::ply_format_t::binary_little_endian);

      return 0;
   }

Features
--------

- common 3d operations
   - | geometric primitives such as points, normals, 
     | triangles, bounding boxes, planes
   - intersection tests
   - basic vector algebra
- I/O for point clouds and meshes
   - ply
   - obj
- graph data structures and algorithms
   - adjacency list
   - knn adjacency list
   - prim's minimum spanning tree
   - breadth/depth first search
- spatial query acceleration data structures
   - linked node octree
   - fixed depth morton encoded octree
   - kd-tree
- 3d point cloud algorithms
   - surface reconstruction
   - filtering & outlier removal
   - resampling
   - registration

Because of **pcp**'s support for composability, there is full transparency 
of our features from low-level to high-level. This means you can use 
functionality at every layer and we do not force you to use functionality 
at other layers of the stack. For example, you don't *need* to use our 
geometric primitives to use our spatial data structures or our algorithms. 
You also don't *need* to use our spatial data structures to use our algorithms 
and vice versa.