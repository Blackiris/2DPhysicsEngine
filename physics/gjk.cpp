#include "gjk.h"
#include "vector3d.h"
#include <iostream>
#include <list>
#include <ranges>

namespace gjk
{
    CollisionInfo are_polys_colliding(const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2) {
        bool has_collision = false;
        Vector2D support = get_support(poly1, poly2, Vector2D(1, 0));

        std::array<Vector2D, 3> simplex = {support};
        unsigned int simplex_size = 1;

        Vector2D direction = -support;

        while(true) {
            support = get_support(poly1, poly2, direction);
            if (!same_direction(support, direction)) {
                // no collision
                has_collision = false;
                break;
            }

            simplex = {support, simplex[0], simplex[1]};
            simplex_size++;

            if (simplex_size == 2) {
                // line, not yet a triangle

                const Vector2D ao = -simplex[0];
                const Vector2D ab = simplex[1] - simplex[0];


                // If origin is after the new point, no collision possible
                if (!same_direction(ab, ao)) {
                    has_collision = false;
                    break;
                }

                const Vector3D ao3(ao);
                const Vector3D ab3(ab);

                direction = ab3.cross(ao3).cross(ab3).toVec2D();
            }

            if (simplex_size == 3) {
                const Vector2D ao = -simplex[0];
                const Vector2D ab = simplex[1] - simplex[0];
                const Vector2D ac = simplex[2] - simplex[0];

                const Vector3D ao3(ao);
                const Vector3D ab3(ab);
                const Vector3D ac3(ac);

                const Vector2D ab_perp = ac3.cross(ab3).cross(ab3).toVec2D();
                const Vector2D ac_perp = ab3.cross(ac3).cross(ac3).toVec2D();

                if (same_direction(ab_perp, ao)) {
                    // Replace simplex index 2 (C)
                    simplex = {simplex[0], simplex[1]};
                    simplex_size = 2;
                    direction = ab_perp;
                } else if (same_direction(ac_perp, ao)) {
                    // Replace simplex index 1 (B)
                    simplex = {simplex[0], simplex[2]};
                    simplex_size = 2;
                    direction = ac_perp;
                } else {
                    has_collision = true;
                    break;
                }

            }
        }
        std::cout << has_collision << std::endl;
        return {has_collision, Vector2D(0, 0), 0};
    }

    Vector2D get_support(const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2, const Vector2D &direction) {
        Vector2D point1 = get_furthest_point(poly1, direction);
        Vector2D point2 = get_furthest_point(poly2, -direction);
        return point1 - point2;
    }

    Vector2D get_furthest_point(const ConvexPolygonShape2D &poly, const Vector2D &direction) {
        Vector2D max_point = poly.points[0];
        float max_dot = direction.dot(max_point);

        for (auto& point : poly.points | std::views::drop(1)) {
            float current_dot = direction.dot(point);
            if (current_dot > max_dot) {
                max_dot = current_dot;
                max_point = point;
            }
        }
        return max_point;
    }

    bool same_direction(const Vector2D &v1, const Vector2D &v2) {
        return v1.dot(v2) > 0;
    }
}
