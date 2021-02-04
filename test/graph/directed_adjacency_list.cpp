#include <algorithm>
#include <catch2/catch.hpp>
#include <numeric>
#include <pcp/graph/directed_adjacency_list.hpp>
#include <pcp/traits/graph_traits.hpp>

SCENARIO("mutable adjacency list", "[adjacency_list]")
{
    GIVEN("a collection of vertices")
    {
        using vertex_type    = std::uint64_t;
        auto const index_map = [](vertex_type const& vertex) {
            return vertex;
        };
        using graph_type =
            pcp::graph::directed_adjacency_list_t<std::uint64_t, decltype(index_map)>;
        using vertex_iterator_type = typename graph_type::vertex_iterator_type;

        static_assert(
            pcp::traits::is_mutable_directed_graph_v<graph_type>,
            "graph_type must satisfy MutableDirectedGraph concept");

        std::size_t n = 10u;
        std::vector<vertex_type> vertices;
        for (std::uint64_t i = 0u; i < n; ++i)
            vertices.emplace_back(i);

        WHEN("adding the vertices to the adjacency list")
        {
            graph_type graph{vertices.begin(), vertices.end(), index_map};
            THEN("the adjacency list's vertex and edge counts are correct")
            {
                REQUIRE(graph.vertex_count() == vertices.size());
                REQUIRE(graph.edge_count() == 0u);
            }
            THEN("the vertices are correctly added")
            {
                auto const reduce_op = [](std::uint64_t cur, std::uint64_t v) {
                    return cur + v;
                };

                auto const sum1 =
                    std::accumulate(vertices.begin(), vertices.end(), std::uint64_t{0u}, reduce_op);
                auto const [begin, end] = graph.vertices();
                auto const sum2         = std::accumulate(begin, end, std::uint64_t{0u}, reduce_op);
                REQUIRE(sum1 == sum2);
            }
            /*WHEN("removing vertices")
            {
                THEN("remaining vertices can be retrieved")
                {
                    auto const [begin1, end1] = graph.vertices();
                    REQUIRE(std::distance(begin1, end1) == 10);
                    auto it = graph.remove_vertex(begin1);
                    REQUIRE(graph.vertex_count() == 9u);
                    auto id = index_map(*it);
                    REQUIRE(id == 1u);

                    auto const [begin2, end2] = graph.vertices();
                    REQUIRE(std::distance(begin2, end2) == 9);
                    REQUIRE(it == begin2);
                    it = graph.remove_vertex(it + 4);
                    REQUIRE(graph.vertex_count() == 8u);
                    id = index_map(*it);
                    REQUIRE(id == 6u);

                    auto const [begin3, end3] = graph.vertices();
                    REQUIRE(std::distance(begin3, end3) == 8);
                    REQUIRE(it == begin3 + 4);
                    it = graph.remove_vertex(end3 - 1);
                    REQUIRE(graph.vertex_count() == 7u);
                    auto const [begin4, end4] = graph.vertices();
                    REQUIRE(it == end4);
                    id = index_map(*(begin4 + 6));
                    REQUIRE(id == 8u);
                    REQUIRE(graph.edge_count() == 0u);
                }
            }*/
            WHEN("adding edges to the adjacency list")
            {
                auto [vertices_begin, vertices_end] = graph.vertices();
                std::vector<std::pair<vertex_iterator_type, vertex_iterator_type>> edges;

                // 1 + 3 + 4 + 6 = 14
                edges.emplace_back(vertices_begin, vertices_begin + index_map(vertices[1]));
                edges.emplace_back(vertices_begin, vertices_begin + index_map(vertices[3]));
                edges.emplace_back(vertices_begin, vertices_begin + index_map(vertices[4]));
                edges.emplace_back(vertices_begin, vertices_begin + index_map(vertices[6]));
                // 0 + 3 + 6 + 7 = 16
                edges.emplace_back(
                    vertices_begin + index_map(vertices[1]),
                    vertices_begin + index_map(vertices[0]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[1]),
                    vertices_begin + index_map(vertices[3]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[1]),
                    vertices_begin + index_map(vertices[6]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[1]),
                    vertices_begin + index_map(vertices[7]));
                // 2 + 3 + 4 + 4 + 6 + 8 + 9 = 36
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[2]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[3]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[4]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[4]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[6]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[8]));
                edges.emplace_back(
                    vertices_begin + index_map(vertices[6]),
                    vertices_begin + index_map(vertices[9]));

                for (auto const& e : edges)
                    graph.add_edge(e.first, e.second);

                auto const reduce_op = [&](vertex_type cur, auto e) {
                    auto [u, v] = e;
                    auto id     = index_map(*v);
                    return cur + id;
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
                    std::array<std::uint8_t, 15> edge_exists{
                        0u,
                    };
                    for (auto it = begin; it != end; ++it)
                    {
                        auto [v1, v2] = *it;
                        auto found_it = std::find_if(
                            edges.begin(),
                            edges.end(),
                            [v1 = v1, v2 = v2](auto const& edge) {
                                return (*v1 == *edge.first) && (*v2 == *edge.second);
                            });
                        if (found_it == edges.end())
                            continue;

                        auto const index =
                            static_cast<std::size_t>(std::distance(edges.begin(), found_it));
                        ++edge_exists[index];
                    }

                    auto const sum = std::accumulate(edge_exists.begin(), edge_exists.end(), 0u);
                    bool const all_edges_retrieved = (sum == 15u);
                    REQUIRE(all_edges_retrieved);
                }
                WHEN("recovering out edges of vertices of degree > 0")
                {
                    auto [begin1, end1] = graph.out_edges_of(vertices_begin);
                    REQUIRE(std::distance(begin1, end1) == 4);
                    auto sum = std::accumulate(begin1, end1, vertex_type{0u}, reduce_op);
                    REQUIRE(sum == 14u);
                    auto [begin2, end2] = graph.out_edges_of(vertices_begin + 1);
                    REQUIRE(std::distance(begin2, end2) == 4);
                    sum = std::accumulate(begin2, end2, vertex_type{0u}, reduce_op);
                    REQUIRE(sum == 16u);
                    auto [begin3, end3] = graph.out_edges_of(vertices_begin + 6);
                    REQUIRE(std::distance(begin3, end3) == 7);
                    sum = std::accumulate(begin3, end3, vertex_type{0u}, reduce_op);
                    REQUIRE(sum == 36u);
                }
                // WHEN("removing vertices")
                //{
                //    auto const [begin1, end1] = graph.vertices();
                //    /**
                //     * Delete first vertex of original graph.
                //     * Any edge that referenced the first vertex should be deleted.
                //     * In this test case, the first four edges (0, 1), (0, 3), (0, 4), (0, 6)
                //     * should be deleted. But the fifth edge (1, 0) should also be deleted, since
                //     * vertex 0 no longer exists.
                //     */
                //    auto const it1 = graph.remove_vertex(begin1);
                //    /**
                //     * Delete third vertex (index 2) of original graph.
                //     * Any edge that referenced the third vertex should be deleted.
                //     * In this test case, the edges (6, 2) should be deleted.
                //     */
                //    auto const it2 = graph.remove_vertex(it1 + 1);

                //    THEN("remaining vertices can be retrieved")
                //    {
                //        REQUIRE(graph.vertex_count() == 8u);
                //        REQUIRE(index_map(*it2) == 3u);
                //    }
                //    THEN("correct edges are removed")
                //    {
                //        REQUIRE(graph.edge_count() == 9u);
                //        auto const [edges_begin, edges_end] = graph.edges();
                //        REQUIRE(std::distance(edges_begin, edges_end) == 9);
                //        auto const sum1 =
                //            std::accumulate(edges_begin, std::next(edges_begin, 3), 0u,
                //            reduce_op);
                //        auto const sum2 =
                //            std::accumulate(std::next(edges_begin, 3), edges_end, 0u, reduce_op);
                //        REQUIRE(sum1 == 16u);
                //        REQUIRE(sum2 == 34u);
                //    }
                //}
            }
        }
    }
}
