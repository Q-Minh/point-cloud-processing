#include <algorithm>
#include <catch2/catch.hpp>
#include <pcp/graph/directed_adjacency_list.hpp>
#include <pcp/graph/vertex.hpp>
#include <pcp/traits/graph_traits.hpp>

SCENARIO("mutable adjacency list", "[adjacency_list]")
{
    GIVEN("a collection of vertices")
    {
        using id_type              = std::uint32_t;
        using vertex_type          = pcp::graph::vertex_t<id_type>;
        using graph_type           = pcp::graph::directed_adjacency_list_t<vertex_type>;
        using vertex_iterator_type = typename graph_type::vertex_iterator_type;

        static_assert(
            pcp::traits::is_mutable_directed_graph_v<graph_type>,
            "graph_type must satisfy MutableDirectedGraph concept");

        std::size_t n = 10u;
        std::vector<vertex_type> vertices;
        for (id_type i = 0u; i < n; ++i)
            vertices.emplace_back(i);

        WHEN("adding the vertices to the adjacency list")
        {
            graph_type graph{vertices.cbegin(), vertices.cend()};
            THEN("the adjacency list's vertex and edge counts are correct")
            {
                REQUIRE(graph.vertex_count() == vertices.size());
                REQUIRE(graph.edge_count() == 0u);
            }
            THEN("the vertices are correctly added")
            {
                auto const reduce_op = [](auto cur, auto const& v) {
                    return cur + v.id();
                };
                auto const sum1 = std::accumulate(vertices.begin(), vertices.end(), 0u, reduce_op);
                auto const [begin, end] = graph.vertices();
                auto const sum2         = std::accumulate(begin, end, 0u, reduce_op);
                REQUIRE(sum1 == sum2);
            }
            WHEN("removing vertices")
            {
                THEN("remaining vertices can be retrieved")
                {
                    auto const [begin1, end1] = graph.vertices();
                    REQUIRE(std::distance(begin1, end1) == 10);
                    auto it = graph.remove_vertex(begin1);
                    REQUIRE(graph.vertex_count() == 9u);
                    REQUIRE(it->id() == 1u);

                    auto const [begin2, end2] = graph.vertices();
                    REQUIRE(std::distance(begin2, end2) == 9);
                    REQUIRE(it == begin2);
                    it = graph.remove_vertex(it + 4);
                    REQUIRE(graph.vertex_count() == 8u);
                    REQUIRE(it->id() == 6u);

                    auto const [begin3, end3] = graph.vertices();
                    REQUIRE(std::distance(begin3, end3) == 8);
                    REQUIRE(it == begin3 + 4);
                    it = graph.remove_vertex(end3 - 1);
                    REQUIRE(graph.vertex_count() == 7u);
                    auto const [begin4, end4] = graph.vertices();
                    REQUIRE(it == end4);
                    REQUIRE((begin4 + 6)->id() == 8u);
                    REQUIRE(graph.edge_count() == 0u);
                }
            }
            WHEN("adding edges to the adjacency list")
            {
                auto [vertices_begin, vertices_end] = graph.vertices();
                std::vector<std::pair<vertex_iterator_type, vertex_iterator_type>> edges;

                // 1 + 3 + 4 + 6 = 14
                edges.emplace_back(vertices_begin, vertices_begin + vertices[1].id());
                edges.emplace_back(vertices_begin, vertices_begin + vertices[3].id());
                edges.emplace_back(vertices_begin, vertices_begin + vertices[4].id());
                edges.emplace_back(vertices_begin, vertices_begin + vertices[6].id());
                // 0 + 3 + 6 + 7 = 16
                edges.emplace_back(
                    vertices_begin + vertices[1].id(),
                    vertices_begin + vertices[0].id());
                edges.emplace_back(
                    vertices_begin + vertices[1].id(),
                    vertices_begin + vertices[3].id());
                edges.emplace_back(
                    vertices_begin + vertices[1].id(),
                    vertices_begin + vertices[6].id());
                edges.emplace_back(
                    vertices_begin + vertices[1].id(),
                    vertices_begin + vertices[7].id());
                // 2 + 3 + 4 + 4 + 6 + 8 + 9 = 36
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[2].id());
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[3].id());
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[4].id());
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[4].id());
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[6].id());
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[8].id());
                edges.emplace_back(
                    vertices_begin + vertices[6].id(),
                    vertices_begin + vertices[9].id());

                for (auto const& e : edges)
                    graph.add_edge(e.first, e.second);

                auto const reduce_op = [](auto cur, auto const& e) {
                    auto [u, v] = e;
                    return cur + v->id();
                };

                THEN("the adjacency list's vertex and edge counts are correct")
                {
                    REQUIRE(graph.vertex_count() == vertices.size());
                    REQUIRE(graph.edge_count() == 15u);
                }
                THEN("the adjacency list's edges are correctly retrievable")
                {
                    auto [begin, end]     = graph.edges();
                    auto const edge_count = std::distance(begin, end);
                    REQUIRE(edge_count == 15);
                    // using random access
                    for (auto i = 0; i < edge_count; ++i)
                    {
                        auto [u, v]   = *(begin + i);
                        auto [up, vp] = *(edges.cbegin() + i);
                        REQUIRE(u->id() == up->id());
                        REQUIRE(v->id() == vp->id());
                    }
                    // using forward access
                    for (auto i = 0; i < edge_count; ++i)
                    {
                        auto [u, v]   = *begin++;
                        auto [up, vp] = *(edges.cbegin() + i);
                        REQUIRE(u->id() == up->id());
                        REQUIRE(v->id() == vp->id());
                    }
                }
                WHEN("recovering out edges of vertices of degree > 0")
                {
                    auto [begin1, end1] = graph.out_edges_of(vertices_begin);
                    REQUIRE(std::distance(begin1, end1) == 4);
                    auto sum = std::accumulate(begin1, end1, 0u, reduce_op);
                    REQUIRE(sum == 14u);
                    auto [begin2, end2] = graph.out_edges_of(vertices_begin + 1);
                    REQUIRE(std::distance(begin2, end2) == 4);
                    sum = std::accumulate(begin2, end2, 0u, reduce_op);
                    REQUIRE(sum == 16u);
                    auto [begin3, end3] = graph.out_edges_of(vertices_begin + 6);
                    REQUIRE(std::distance(begin3, end3) == 7);
                    sum = std::accumulate(begin3, end3, 0u, reduce_op);
                    REQUIRE(sum == 36u);
                }
                WHEN("removing vertices")
                {
                    auto const [begin1, end1] = graph.vertices();
                    /**
                     * Delete first vertex of original graph.
                     * Any edge that referenced the first vertex should be deleted.
                     * In this test case, the first four edges (0, 1), (0, 3), (0, 4), (0, 6)
                     * should be deleted. But the fifth edge (1, 0) should also be deleted, since
                     * vertex 0 no longer exists.
                     */
                    auto const it1 = graph.remove_vertex(begin1);
                    /**
                     * Delete third vertex of original graph.
                     * Any edge that referenced the third vertex should be deleted.
                     * In this test case, the edge (6, 2) references the third vertex
                     * and should be deleted.
                     */
                    auto const it2 = graph.remove_vertex(it1 + 1);

                    THEN("remaining vertices can be retrieved")
                    {
                        REQUIRE(graph.vertex_count() == 8u);
                        REQUIRE(it2->id() == 3u);
                    }
                    THEN("correct edges are removed")
                    {
                        REQUIRE(graph.edge_count() == 9u);
                        auto const [edges_begin, edges_end] = graph.edges();
                        REQUIRE(std::distance(edges_begin, edges_end) == 9);
                        auto const sum1 =
                            std::accumulate(edges_begin, edges_begin + 3, 0u, reduce_op);
                        auto const sum2 =
                            std::accumulate(edges_begin + 3, edges_end, 0u, reduce_op);
                        REQUIRE(sum1 == 16u);
                        REQUIRE(sum2 == 34u);
                    }
                }
            }
        }
    }
}
