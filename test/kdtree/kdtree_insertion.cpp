#include <catch2/catch.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/vector3d.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/kdtree/linked_kdtree.hpp>

SCENARIO("kdtree insertion", "[kdtree]")
{
    auto const coordinate_map = [](pcp::point_t const& p) {
        return std::array<float, 3u>{p.x(), p.y(), p.z()};
    };

    using kdtree_type = pcp::basic_linked_kdtree_t<pcp::point_t, 3u, decltype(coordinate_map)>;

    GIVEN("A point cloud with 2 points in each octant of a [0, 4]^3 grid")
    {
        /**
         * Points live in a 4x4x4 grid [0, 4]^3
         * Octants of this grid are numbered from 0 to 7
         * in counter clockwise fashion around the z-axis,
         * and ascending in the y direction,
         * with coordinate frame:
         *
         *     z  y
         *     o o
         *     |/
         *     o---o x
         *
         * So octants 0, 1, 2, 3 are all octants around the z-axis
         * in counterclockwise order at y = [0, 2], and
         * octants 4, 5, 6, 7 are all octants around the z-axis
         * in counterclockwise order at y = [2, 4]
         *
         */
        std::vector<pcp::point_t> points{};
        pcp::point_t p00{0.1f, 0.1f, 0.1f};
        pcp::point_t p01{0.2f, 0.2f, 0.2f};

        pcp::point_t p10{2.1f, 0.3f, 0.3f};
        pcp::point_t p11{2.2f, 0.4f, 0.4f};

        pcp::point_t p20{2.3f, 2.1f, 0.5f};
        pcp::point_t p21{2.4f, 2.2f, 0.6f};

        pcp::point_t p30{0.3f, 2.3f, 0.7f};
        pcp::point_t p31{0.4f, 2.4f, 0.8f};

        pcp::point_t p40{0.5f, 0.5f, 2.1f};
        pcp::point_t p41{0.6f, 0.6f, 2.2f};

        pcp::point_t p50{2.5f, 0.7f, 2.3f};
        pcp::point_t p51{2.6f, 0.8f, 2.4f};

        pcp::point_t p60{2.7f, 2.5f, 2.5f};
        pcp::point_t p61{2.8f, 2.6f, 2.6f};

        pcp::point_t p70{0.7f, 2.7f, 2.7f};
        pcp::point_t p71{0.8f, 2.8f, 2.8f};

        points.assign(
            {p00, p10, p20, p30, p40, p50, p60, p70, p01, p11, p21, p31, p41, p51, p61, p71});

        WHEN("Constructing a kd-tree using nth element approach")
        {
            pcp::kdtree::construction_params_t params{};
            params.max_depth    = 4u;
            params.construction = pcp::kdtree::construction_t::nth_element;

            kdtree_type kdtree{points.begin(), points.end(), coordinate_map, params};

            THEN("The tree's storage contains all the inserted points")
            {
                auto const has_point = [&](pcp::point_t const& p) {
                    auto begin        = kdtree.begin();
                    auto end          = kdtree.end();
                    auto const equals = [p1 = p](pcp::point_t const& p2) {
                        return pcp::common::are_vectors_equal(p1, p2);
                    };
                    return std::find_if(begin, end, equals) != end;
                };
                for (auto const& p : points)
                {
                    REQUIRE(has_point(p));
                }
            }
            THEN("The tree has the correct underlying structure")
            {
                // level 0
                auto const& root = kdtree.root();
                REQUIRE(root->is_internal());
                REQUIRE(root->points().size() == 1u);
                auto median = root->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p10));

                // level 1
                auto const& root_left = root->left();
                REQUIRE(root_left->is_internal());
                REQUIRE(root_left->points().size() == 1u);
                median = root_left->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p30));

                auto const& root_right = root->right();
                REQUIRE(root_right->is_internal());
                REQUIRE(root_right->points().size() == 1u);
                median = root_right->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p20));

                // level 2
                auto const& root_left_left = root_left->left();
                REQUIRE(root_left_left->is_internal());
                REQUIRE(root_left_left->points().size() == 1u);
                median = root_left_left->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p40));

                auto const& root_left_right = root_left->right();
                REQUIRE(root_left_right->is_internal());
                REQUIRE(root_left_right->points().size() == 1u);
                median = root_left_right->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p70));

                auto const& root_right_left = root_right->left();
                REQUIRE(root_right_left->is_internal());
                REQUIRE(root_right_left->points().size() == 1u);
                median = root_right_left->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p50));

                auto const& root_right_right = root_right->right();
                REQUIRE(root_right_right->is_internal());
                REQUIRE(root_right_right->points().size() == 1u);
                median = root_right_right->points().front();
                REQUIRE(pcp::common::are_vectors_equal(*median, p60));

                // level 3
                auto const& root_left_left_left = root_left_left->left();
                REQUIRE(root_left_left_left->is_leaf());
                REQUIRE(root_left_left_left->points().size() == 2u);
                auto point1        = *(root_left_left_left->points()[0]);
                auto point2        = *(root_left_left_left->points()[1]);
                bool const has_p00 = pcp::common::are_vectors_equal(point1, p00) ||
                                     pcp::common::are_vectors_equal(point1, p01);
                bool const has_p01 = pcp::common::are_vectors_equal(point2, p00) ||
                                     pcp::common::are_vectors_equal(point2, p01);
                REQUIRE(has_p00);
                REQUIRE(has_p01);

                auto const& root_left_left_right = root_left_left->right();
                REQUIRE(root_left_left_right->is_leaf());
                REQUIRE(root_left_left_right->points().size() == 1u);
                auto point         = *(root_left_left_right->points().front());
                bool const has_p41 = pcp::common::are_vectors_equal(point, p41);
                REQUIRE(has_p41);

                auto const& root_left_right_left = root_left_right->left();
                REQUIRE(root_left_right_left->is_leaf());
                REQUIRE(root_left_right_left->points().size() == 1u);
                point              = *(root_left_right_left->points().front());
                bool const has_p31 = pcp::common::are_vectors_equal(point, p31);
                REQUIRE(has_p31);

                auto const& root_left_right_right = root_left_right->right();
                REQUIRE(root_left_right_right->is_leaf());
                REQUIRE(root_left_right_right->points().size() == 1u);
                point              = *(root_left_right_right->points().front());
                bool const has_p71 = pcp::common::are_vectors_equal(point, p71);
                REQUIRE(has_p71);

                auto const& root_right_left_left = root_right_left->left();
                REQUIRE(root_right_left_left->is_leaf());
                REQUIRE(root_right_left_left->points().size() == 1u);
                point              = *(root_right_left_left->points().front());
                bool const has_p11 = pcp::common::are_vectors_equal(point, p11);
                REQUIRE(has_p11);

                auto const& root_right_left_right = root_right_left->right();
                REQUIRE(root_right_left_right->is_leaf());
                REQUIRE(root_right_left_right->points().size() == 1u);
                point              = *(root_right_left_right->points().front());
                bool const has_p51 = pcp::common::are_vectors_equal(point, p51);
                REQUIRE(has_p51);

                auto const& root_right_right_left = root_right_right->left();
                REQUIRE(root_right_right_left->is_leaf());
                REQUIRE(root_right_right_left->points().size() == 1u);
                point              = *(root_right_right_left->points().front());
                bool const has_p21 = pcp::common::are_vectors_equal(point, p21);
                REQUIRE(has_p21);

                auto const& root_right_right_right = root_right_right->right();
                REQUIRE(root_right_right_right->is_leaf());
                REQUIRE(root_right_right_right->points().size() == 1u);
                point              = *(root_right_right_right->points().front());
                bool const has_p61 = pcp::common::are_vectors_equal(point, p61);
                REQUIRE(has_p61);
            }
        }
    }
}
