#include "gjk.h"
#include "vector3d.h"
#include <ranges>
#include <limits>

namespace gjk
{
struct ClosestSegmentInfo {
    unsigned index;
    Vector2D normal;
    float depth;
};

bool same_direction(const Vector2D &v1, const Vector2D &v2) {
    return v1.dot(v2) > 0;
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

Vector2D get_support(const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2, const Vector2D &direction) {
    Vector2D point1 = get_furthest_point(poly1, direction);
    Vector2D point2 = get_furthest_point(poly2, -direction);
    return point1 - point2;
}

ClosestSegmentInfo find_closest_segment(const std::vector<Vector2D> &polytope, const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2) {
    Vector2D min_normal{0, 0};
    float min_depth = std::numeric_limits<float>::max();
    unsigned min_index = 0;

    for (unsigned i=0; i<polytope.size(); i++) {
        unsigned j = (i+1) % polytope.size();

        Vector2D ij = polytope[j] - polytope[i];
        Vector2D normal_ij = Vector2D(ij.y, -ij.x).normalize();

        float ij_depth = normal_ij.dot(polytope[i]);
        if (ij_depth < 0) {
            ij_depth *= -1;
            normal_ij = -normal_ij;
        }

        if (ij_depth < min_depth) {
            min_depth = ij_depth;
            min_normal = normal_ij;
            min_index = i;
        }
    }

    return {min_index, min_normal, min_depth};
}

/**
 * @brief get_collision_info based on EPA algorithm
 * @param simplex
 * @param poly1
 * @param poly2
 * @return collision info.
 */
CollisionInfo get_collision_info(const std::array<Vector2D, 3> &simplex, const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2) {

    std::vector<Vector2D> polytope{std::begin(simplex), std::end(simplex)};
    ClosestSegmentInfo min_closest_segment_info;

    while (true) {
        ClosestSegmentInfo closest_segment_info = find_closest_segment(polytope, poly1, poly2);
        Vector2D support = get_support(poly1, poly2, closest_segment_info.normal);


        float support_distance = closest_segment_info.normal.dot(support);

        if (std::abs(support_distance - closest_segment_info.depth) > 0.001) {
            polytope.insert(polytope.begin() + closest_segment_info.index + 1, support);
        } else {
            min_closest_segment_info = closest_segment_info;
            break;
        }
    }


    return {true, min_closest_segment_info.normal, min_closest_segment_info.depth};
}

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
    if (has_collision) {
        return get_collision_info(simplex, poly1, poly2);
    } else {
        return {false, Vector2D(0, 0), 0};
    }
}

}
