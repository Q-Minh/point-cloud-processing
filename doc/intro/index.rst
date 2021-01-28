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

.. literalinclude:: ../../examples/simple_example.cpp
   :language: cpp

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