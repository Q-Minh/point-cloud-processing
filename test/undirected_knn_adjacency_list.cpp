#include <catch2/catch.hpp>
#include <pcp/common/point_predicates.hpp>
#include <pcp/graph/directed_adjacency_list.hpp>
#include <pcp/graph/undirected_knn_adjacency_list.hpp>
#include <pcp/octree.hpp>
#include <pcp/point.hpp>
#include <pcp/traits/graph_traits.hpp>
#include <pcp/vertex.hpp>

SCENARIO("k nearest neighbours undirected adjacency list", "[undirected_knn_adjacency_list]")
{
    GIVEN("an octree of 8 clusters of 3 points")
    {
        using point_type  = pcp::point_t;
        using vertex_type = pcp::basic_vertex_t<point_type>;
        using graph_type  = pcp::graph::directed_adjacency_list_t<vertex_type>;

        static_assert(
            pcp::traits::is_directed_graph_v<graph_type>,
            "DirectedGraph concept not satisfied");

        auto const push_back_cluster =
            [](std::vector<point_type>& points, pcp::point_t const& c, float radius) {
                points.push_back(c);
                pcp::point_t neighbor1{c.x() + radius, c.y() + radius, c.z() + radius};
                pcp::point_t neighbor2{c.x() - radius, c.y() - radius, c.z() - radius};
                points.push_back(neighbor1);
                points.push_back(neighbor2);
            };

        std::uint32_t k = 2u;
        std::vector<point_type> points{};

        /**
         * Add points in 8 clusters of 3 points
         */
        float radius = .1f;

        std::array<pcp::point_t, 8> centers = {
            pcp::point_t{.5f, .5f, .5f},
            pcp::point_t{.5f, .5f, -.5f},
            pcp::point_t{.5f, -.5f, .5f},
            pcp::point_t{.5f, -.5f, -.5f},
            pcp::point_t{-.5f, .5f, .5f},
            pcp::point_t{-.5f, .5f, -.5f},
            pcp::point_t{-.5f, -.5f, .5f},
            pcp::point_t{-.5f, -.5f, -.5f}};

        for (auto const& c : centers)
            push_back_cluster(points, c, radius);

        std::vector<vertex_type> vertices;
        vertices.reserve(points.size());
        std::transform(
            std::begin(points),
            std::end(points),
            std::back_inserter(vertices),
            [](auto& p) { return vertex_type(&p); });

        // generate vertex ids
        for (std::uint32_t i = 0u; i < vertices.size(); ++i)
            vertices[i].id(i);

        THEN("collections of point_view_t and point_t compare equal")
        {
            bool const are_vertices_and_points_equal = std::equal(
                std::cbegin(vertices),
                std::cend(vertices),
                std::cbegin(points),
                [](auto const& p1, auto const& p2) { return pcp::are_points_equal(p1, p2); });
            REQUIRE(are_vertices_and_points_equal);
        }

        pcp::octree_parameters_t<point_type> params;
        using params_type    = decltype(params);
        params.max_depth     = static_cast<std::uint8_t>(21u);
        params.node_capacity = 2u;
        params.voxel_grid    = {{-1.f, -1.f, -1.f}, {1.f, 1.f, 1.f}};

        pcp::basic_octree_t<vertex_type, params_type> octree{
            std::cbegin(vertices),
            std::cend(vertices),
            params};

        auto knn = [&octree](vertex_type const& v, std::uint64_t k) {
            return octree.nearest_neighbours(v, k);
        };

        using octree_type          = decltype(octree);
        using octree_iterator_type = typename octree_type::iterator;
        using knn_type             = decltype(knn);

        WHEN("creating a directed_knn_adjacency_list_t from the octree")
        {
            auto graph = pcp::graph::undirected_knn_graph(vertices.begin(), vertices.end(), knn, k);

            THEN("the graph's edge count is 'vertex_count * k nearest neighbours * 2'")
            {
                auto const [edges_begin, edges_end] = graph.edges();
                auto const edge_count =
                    static_cast<std::size_t>(std::distance(edges_begin, edges_end));
                REQUIRE(edge_count == vertices.size() * k * 2u);
                REQUIRE(edge_count == octree.size() * k * 2u);
            }
            THEN("the graph's vertex count is equal to the octree's size")
            {
                auto const [vertices_begin, vertices_end] = graph.vertices();
                auto const vertex_count =
                    static_cast<std::size_t>(std::distance(vertices_begin, vertices_end));
                REQUIRE(vertex_count == vertices.size());
                REQUIRE(vertex_count == octree.size());
            }
            THEN("the graph has the correct topology")
            {
                auto const [vertices_begin, vertices_end] = graph.vertices();

                bool has_correct_topology = true;
                for (auto it = vertices_begin; it != vertices_end; ++it)
                {
                    auto const [edges_begin, edges_end] = graph.out_edges_of(it);
                    for (auto eit = edges_begin; eit != edges_end; ++eit)
                    {
                        auto const [u, v] = *eit;
                        has_correct_topology &= (*u == *it);
                        has_correct_topology &= (*u != *v);
                        has_correct_topology &=
                            (std::find(std::cbegin(vertices), std::cend(vertices), *v) !=
                             std::cend(vertices));
                    }
                }
                REQUIRE(has_correct_topology);
            }
        }
    }
}