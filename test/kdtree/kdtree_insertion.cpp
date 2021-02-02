#include <catch2/catch.hpp>
#include <iostream>
#include <pcp/kdtree/kdtree.hpp>
#include "pcp/common/points/point.hpp"

SCENARIO("kdtree insertion", "[kdtree]")
{

    



       std::vector<pcp::point_t> points{};
       pcp::point_t first({0.1f, 0.2f, 0.3f});

       points.push_back({0.6f, 0.6f, 0.6f});
       points.push_back({0.7f, 0.7f, 0.7f});
       points.push_back({0.8f, 0.8f, 0.8f});
       points.push_back({0.9f, 0.9f, 0.9f});
       points.push_back({1.0f, 2.0f, 3.0f});
       points.push_back({2.0f, 3.0f, 4.0f});
       points.push_back({0.1f, 0.1f, 0.1f});
       points.push_back({0.2f, 0.2f, 0.2f});
       points.push_back({0.3f, 0.3f, 0.3f});
       points.push_back({0.4f, 0.4f, 0.4f});
       points.push_back({0.5f, 0.5f, 0.5f});



        kdtree_t kd = kdtree_t(std::begin(points), std::end(points),3);
        int size    = kd.size();
        kd.print_in_order();
       


       std::cout << "nb of nodes " << size << '\n';
       std::cout << "nb of points " << points.size() << '\n';

       
  




    
}
