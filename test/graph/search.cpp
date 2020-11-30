#include "dumb_vertex.hpp"

#include <catch2/catch.hpp>
#include <pcp/graph/directed_adjacency_list.hpp>
#include <pcp/graph/search.hpp>

SCENARIO("graph searching algorithms", "[graph]")
{
    GIVEN("a graph with cycles")
    {
        using vertex_type = pcp::test::dumb_vertex_t;
        using id_type     = typename vertex_type::id_type;
        using graph_type  = pcp::graph::directed_adjacency_list_t<vertex_type>;
        graph_type G;
        auto v1  = 0u;
        auto v2  = 1u;
        auto v3  = 2u;
        auto v4  = 3u;
        auto v5  = 4u;
        auto v6  = 5u;
        auto v7  = 6u;
        auto v8  = 7u;
        auto v9  = 8u;
        auto v10 = 9u;
        auto v11 = 10u;

        G.add_vertex({v1});
        G.add_vertex({v2});
        G.add_vertex({v3});
        G.add_vertex({v4});
        G.add_vertex({v5});
        G.add_vertex({v6});
        G.add_vertex({v7});
        G.add_vertex({v8});
        G.add_vertex({v9});
        G.add_vertex({v10});
        G.add_vertex({v11});

        auto [vbegin, vend] = G.vertices();
        auto const n        = std::distance(vbegin, vend);
        auto a              = vbegin + v1;
        auto b              = vbegin + v2;
        auto c              = vbegin + v3;
        auto d              = vbegin + v4;
        auto e              = vbegin + v5;
        auto f              = vbegin + v6;
        auto g              = vbegin + v7;
        auto h              = vbegin + v8;
        auto i              = vbegin + v9;
        auto j              = vbegin + v10;
        auto k              = vbegin + v11;

        /**
         * Graph: (cycles are denoted by edges with arrows)
         *    b         f----------
         *   /         /          |
         * a -- c -- e -- g <------
         *   \       | \
         *    d      i  h--j
         *    ^      |   \
         *    |      |    k
         *    --------
         */
        G.add_edge(a, b);
        G.add_edge(a, c);
        G.add_edge(a, d);
        G.add_edge(c, e);
        G.add_edge(e, f);
        G.add_edge(e, g);
        G.add_edge(e, h);
        G.add_edge(e, i);
        G.add_edge(h, j);
        G.add_edge(h, k);

        G.add_edge(f, g);
        G.add_edge(i, d);

        auto const count = static_cast<std::size_t>(n);
        std::vector<std::uint8_t> visits(count, 0u);
        std::vector<std::uint8_t> visiting_order(count, 0u);
        auto visited_nodes_counter = 0u;
        auto starting_id           = -1;

        auto const visitor = [&visits, &visiting_order, &visited_nodes_counter, &starting_id](
                                 vertex_type const& source,
                                 vertex_type const& dest) {
            auto const source_id = source.id();
            auto const dest_id   = dest.id();

            // the root of the graph is never 
            // visited as a dest vertex, so 
            // if this is the root node, 
            // let us update the visit stats
            // for the it.
            if (starting_id == -1)
            {
                starting_id = source_id;
                ++visits[source_id];
                ++visited_nodes_counter;
            }

            ++visits[dest_id];
            visiting_order[dest_id] = visited_nodes_counter;
            ++visited_nodes_counter;
        };

        WHEN("visiting the graph in breadth first order")
        {
            pcp::graph::breadth_first_search(G, a, visitor);
            THEN("the visitor visits nodes in breadth first order")
            {
                REQUIRE(visited_nodes_counter == count);
                REQUIRE(starting_id == 0);

                bool const all_nodes_visited_once =
                    std::all_of(visits.cbegin(), visits.cend(), [](auto const& count) {
                        return count == 1u;
                    });
                REQUIRE(all_nodes_visited_once);

                bool const all_nodes_visited_in_breadth_first_order =
                    visiting_order[v1] == 0u && visiting_order[v2] == 1u &&
                    visiting_order[v3] == 2u && visiting_order[v4] == 3u &&
                    visiting_order[v5] == 4u && visiting_order[v6] == 5u &&
                    visiting_order[v7] == 6u && visiting_order[v8] == 7u &&
                    visiting_order[v9] == 8u && visiting_order[v10] == 9u &&
                    visiting_order[v11] == 10u;
                REQUIRE(all_nodes_visited_in_breadth_first_order);
            }
        }
        WHEN("traversing the graph in depth first order")
        {
            pcp::graph::depth_first_search(G, a, visitor);
            THEN("the visitor visits nodes in depth first order")
            {
                REQUIRE(visited_nodes_counter == count);
                REQUIRE(starting_id == 0);

                bool const all_nodes_visited_once =
                    std::all_of(visits.cbegin(), visits.cend(), [](auto const& count) {
                        return count == 1u;
                    });
                REQUIRE(all_nodes_visited_once);

                bool const bcd_not_visited_sequentially =
                    std::abs(visiting_order[v2] - visiting_order[v4]) > 2;

                bool const ghi_not_visited_sequentially =
                    std::abs(visiting_order[v7] - visiting_order[v9]) > 2;

                bool const all_nodes_visited_in_depth_first_order =
                    bcd_not_visited_sequentially && ghi_not_visited_sequentially;
                REQUIRE(all_nodes_visited_in_depth_first_order);
            }
        }
    }
}