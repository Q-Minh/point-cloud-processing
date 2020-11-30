#include "graph/dumb_vertex.hpp"

#include <catch2/catch.hpp>
#include <pcp/graph/directed_adjacency_list.hpp>
#include <pcp/graph/minimum_spanning_tree.hpp>

SCENARIO("minimum spanning tree algorithms", "[minimum_spanning_tree]")
{
    GIVEN("an adjacency list graph")
    {
        /**
         * Graph:                    Minimum Spanning Tree (starting from e):
         *
         *                4                         4
         *        ----a---------b           ----a---------b
         *        |   |\        |           |   |
         *        |   | \       |           |   |
         *        |   |  \      |           |   |
         *        |   |   \     |           |   |
         *        |   | 3  \ 8  | 5         |   | 3
         *        |   |     \   |           |   |
         *      4 |   |      \  |         4 |   |
         *        |   |       \ |           |   |
         *        |   |    2   \|           |   |    2
         *        |   d---------c           |   d---------c
         *        |   |                     |
         *        |   |                     |
         *        |   | 11                  |
         *        |   |                     |
         *        ----e                     ----e
         */
        using vertex_type = pcp::test::dumb_vertex_t;
        using id_type     = typename vertex_type::id_type;
        using graph_type  = pcp::graph::directed_adjacency_list_t<vertex_type>;
        graph_type G;
        auto v1 = 0u;
        auto v2 = 1u;
        auto v3 = 2u;
        auto v4 = 3u;
        auto v5 = 4u;
        G.add_vertex(v1);
        G.add_vertex(v2);
        G.add_vertex(v3);
        G.add_vertex(v4);
        G.add_vertex(v5);
        auto [vbegin, vend] = G.vertices();
        auto a              = vbegin + v1;
        auto b              = vbegin + v2;
        auto c              = vbegin + v3;
        auto d              = vbegin + v4;
        auto e              = vbegin + v5;

        // we make the graph undirected by adding both directions
        // to every directed edge of the graph
        G.add_edge(a, b);
        G.add_edge(b, a);

        G.add_edge(a, c);
        G.add_edge(c, a);

        G.add_edge(a, d);
        G.add_edge(d, a);

        G.add_edge(a, e);
        G.add_edge(e, a);

        G.add_edge(b, c);
        G.add_edge(c, b);

        G.add_edge(c, d);
        G.add_edge(d, c);

        G.add_edge(d, e);
        G.add_edge(e, d);

        WHEN("computing prim's minimum spanning tree")
        {
            auto const cost = [](vertex_type const& u, vertex_type const& v) -> uint8_t {
                if ((u.id() == 0u && v.id() == 1u) || (u.id() == 1u && v.id() == 0u))
                    return 4u;
                if ((u.id() == 0u && v.id() == 2u) || (u.id() == 2u && v.id() == 0u))
                    return 8u;
                if ((u.id() == 0u && v.id() == 3u) || (u.id() == 3u && v.id() == 0u))
                    return 3u;
                if ((u.id() == 0u && v.id() == 4u) || (u.id() == 4u && v.id() == 0u))
                    return 4u;
                if ((u.id() == 1u && v.id() == 2u) || (u.id() == 2u && v.id() == 1u))
                    return 5u;
                if ((u.id() == 3u && v.id() == 2u) || (u.id() == 2u && v.id() == 3u))
                    return 2u;
                if ((u.id() == 4u && v.id() == 3u) || (u.id() == 3u && v.id() == 4u))
                    return 11u;

                return std::numeric_limits<uint8_t>::max();
            };

            auto [MST, get_root] = pcp::graph::prim_minimum_spanning_tree(G, cost, e);

            THEN("the correct minimum spanning tree is found")
            {
                REQUIRE(MST.vertex_count() == 5u);
                auto const [mst_vbegin, mst_vvend] = MST.vertices();
                auto const root                    = get_root(MST);

                // the root should be vertex e, which has value 4u
                REQUIRE(root->id() == 4u);
                REQUIRE(std::find(mst_vbegin, mst_vvend, 0u) != mst_vvend);
                REQUIRE(std::find(mst_vbegin, mst_vvend, 1u) != mst_vvend);
                REQUIRE(std::find(mst_vbegin, mst_vvend, 2u) != mst_vvend);
                REQUIRE(std::find(mst_vbegin, mst_vvend, 3u) != mst_vvend);
                REQUIRE(std::find(mst_vbegin, mst_vvend, 4u) != mst_vvend);

                auto const [ebegin, eend] = MST.edges();
                REQUIRE(MST.edge_count() == 4u);
                REQUIRE(std::distance(ebegin, eend) == 4);

                // vertex e should have 1 out edge only, which is (e,a)
                auto [out_edges_of_root_begin, out_edges_of_root_end] = MST.out_edges_of(root);
                REQUIRE(std::distance(out_edges_of_root_begin, out_edges_of_root_end) == 1);
                auto [root_edge_v1, root_edge_v2] = *out_edges_of_root_begin;
                REQUIRE(root_edge_v1 == root);
                // should be vertex a
                REQUIRE(root_edge_v2->id() == 0u);

                // vertex a should have 2 out edges, which are (a,b), (a,d)
                auto [out_edges_of_a_begin, out_edges_of_a_end] = MST.out_edges_of(root_edge_v2);

                REQUIRE(std::distance(out_edges_of_a_begin, out_edges_of_a_end) == 2);
                // check for edge (a,b)
                auto const is_vertex_b = [](auto const& edge) {
                    auto [u, v] = edge;
                    return *v == 1u;
                };
                REQUIRE(
                    std::find_if(out_edges_of_a_begin, out_edges_of_a_end, is_vertex_b) !=
                    out_edges_of_a_end);

                // check for edge (a,d)
                auto const is_vertex_d = [](auto const& edge) {
                    auto [u, v] = edge;
                    return *v == 3u;
                };
                REQUIRE(
                    std::find_if(out_edges_of_a_begin, out_edges_of_a_end, is_vertex_d) !=
                    out_edges_of_a_end);

                // get the edge (a,d) to extract vertex d
                auto [ad_edge_v1, ad_edge_v2] =
                    *std::find_if(out_edges_of_a_begin, out_edges_of_a_end, is_vertex_d);
                // vertex d should have 1 out edge which is (d,c)
                auto [out_edges_of_d_begin, out_edges_of_d_end] = MST.out_edges_of(ad_edge_v2);
                REQUIRE(std::distance(out_edges_of_d_begin, out_edges_of_d_end) == 1);
                auto [dc_edge_v1, dc_edge_v2] = *out_edges_of_d_begin;
                // should be vertex c
                REQUIRE(dc_edge_v2->id() == 2u);
            }
        }
    }
}