#include "collision_resolver.h"
#include "gjk.h"

#include <algorithm>
#include <iostream>

CollisionResolver::CollisionResolver() {}

void CollisionResolver::update_locations(std::list<RigidBody*> &rigid_bodies, std::list<StaticBody*> &static_bodies, const float &dt) {
    for (auto &obj: rigid_bodies) {
        obj->update_location(dt);

        for (auto &other_rigid: rigid_bodies) {
            if (obj == other_rigid) {
                break;
            }

            CollisionInfo collision_info = CollisionResolver::are_colliding(*obj, *other_rigid);

            if (collision_info.are_colliding) {
                float masses_sum = obj->mass + other_rigid->mass;
                Vector2D velocity_diff = other_rigid->velocity - obj->velocity;

                obj->move(collision_info.penetration * collision_info.normal);

                obj->velocity += 2*other_rigid->mass * velocity_diff.dot(collision_info.normal) / masses_sum * collision_info.normal;
                other_rigid->velocity -= 2*obj->mass * velocity_diff.dot(collision_info.normal) / masses_sum * collision_info.normal;
            }
        }

        for (auto &other_static: static_bodies) {
            CollisionInfo collision_info = CollisionResolver::are_colliding(*obj, *other_static);

            if (collision_info.are_colliding) {

                obj->move(collision_info.penetration * collision_info.normal);

                obj->velocity -= 2 * (obj->elastic_coeff + other_static->elastic_coeff) / 2 * obj->velocity.dot(collision_info.normal) * collision_info.normal;
            }
        }
    }
}

CollisionInfo CollisionResolver::are_colliding(const PhysicBody &body1, const PhysicBody &body2) {

    const Transform2D &location1 = body1.location;
    const Shape2D &shape1 = body1.shape;

    const Transform2D &location2 = body2.location;
    const Shape2D &shape2 = body2.shape;


    const CircleShape2D* circle1 = dynamic_cast<const CircleShape2D*>(&shape1);
    if (circle1 != nullptr) {
        const CircleShape2D* circle2 = dynamic_cast<const CircleShape2D*>(&shape2);
        if (circle2 != nullptr) {
            return are_spheres_colliding(location1, *circle1, location2, *circle2);
        }

        const ConvexPolygonShape2D* poly2 = dynamic_cast<const ConvexPolygonShape2D*>(&shape2);
        if (poly2 != nullptr) {
            return are_sphere_poly_colliding(location1, *circle1, location2, *poly2);
        }
    } else {
        const ConvexPolygonShape2D* poly1 = dynamic_cast<const ConvexPolygonShape2D*>(&shape1);
        if (poly1 != nullptr) {
            const CircleShape2D* circle2 = dynamic_cast<const CircleShape2D*>(&shape2);
            if (circle2 != nullptr) {
                return are_sphere_poly_colliding(location2, *circle2, location1, *poly1);
            }

            const ConvexPolygonShape2D* poly2 = dynamic_cast<const ConvexPolygonShape2D*>(&shape2);
            if (poly2 != nullptr) {
                return are_polys_colliding(location1, *poly1, location2, *poly2);
            }
        }
    }

    return {false, Vector2D(0, 0), 0};
}



CollisionInfo CollisionResolver::are_spheres_colliding(const Transform2D &loc1, const CircleShape2D &circle1,
                                const Transform2D &loc2, const CircleShape2D &circle2) {
    Vector2D loc_diff = loc1.point2d - loc2.point2d;
    float loc_dist = loc_diff.length();
    float circle_radiuses = circle1.r + circle2.r;
    float penetration = circle_radiuses - loc_dist;

    return {penetration > 0, loc_diff.get_unit_vector(), penetration};
}

CollisionInfo CollisionResolver::are_sphere_poly_colliding(const Transform2D &loc1, const CircleShape2D &circle1,
                                               const Transform2D &loc2, const ConvexPolygonShape2D &poly2) {
    const unsigned int nb_points = poly2.points.size();
    bool has_collision = false;
    Vector2D normal;
    float penetration = 0;

    for (unsigned int i=0; i<nb_points; i++) {
        Vector2D point1 = loc2*poly2.points[i];
        Vector2D point2 = loc2*poly2.points[(i+1) % nb_points];

        Vector2D segment = point2 - point1;
        const float segment_length = segment.length();
        Vector2D unit_vect = segment.get_unit_vector();

        Vector2D seg_to_circle = loc1.point2d - point1;

        const float projection = unit_vect.dot(seg_to_circle);

        if (projection <= 0) {
            const float seg_to_circle_length = seg_to_circle.length();
            if (seg_to_circle_length < circle1.r) {
                has_collision = true;
                normal = seg_to_circle.get_unit_vector();
                penetration = circle1.r - seg_to_circle_length;
            }
        } else if (projection >= segment_length) {
            Vector2D seg_to_circle_2nd = loc1.point2d - point2;
            const float seg_to_circle_2nd_length = seg_to_circle_2nd.length();

            if (seg_to_circle_2nd_length < circle1.r) {
                has_collision = true;
                normal = seg_to_circle_2nd.get_unit_vector();
                penetration = circle1.r - seg_to_circle_2nd_length;
            }
        } else {
            Vector2D middle_point = point1 + projection * unit_vect;

            Vector2D seg_to_circle_2nd = loc1.point2d - middle_point;
            const float seg_to_circle_2nd_length = seg_to_circle_2nd.length();

            if (seg_to_circle_2nd_length < circle1.r) {
                has_collision = true;
                normal = seg_to_circle_2nd.get_unit_vector();
                penetration = circle1.r - seg_to_circle_2nd_length;
            }
        }

        if (has_collision) {
            break;
        }
    }

    return {has_collision, normal, penetration};
}


CollisionInfo CollisionResolver::are_polys_colliding(const Transform2D &loc1, const ConvexPolygonShape2D &poly1,
                                                     const Transform2D &loc2, const ConvexPolygonShape2D &poly2) {
    std::vector<Vector2D> new_points1(poly1.points);
    auto loc1_applier = [&loc1](Vector2D point) {return loc1 * point;};
    std::transform(new_points1.begin(), new_points1.end(), new_points1.begin(), loc1_applier);

    std::vector<Vector2D> new_points2(poly2.points);
    auto loc2_applier = [&loc2](Vector2D point) {return loc2 * point;};
    std::transform(new_points2.begin(), new_points2.end(), new_points2.begin(), loc2_applier);

    ConvexPolygonShape2D poly1_after_transform(new_points1);
    ConvexPolygonShape2D poly2_after_transform(new_points2);
    return gjk::are_polys_colliding(poly1_after_transform, poly2_after_transform);
}
