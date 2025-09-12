#include "collision_resolver.h"
#include "gjk.h"
#include "rigidbody.h"

#include <algorithm>
#include <memory>

CollisionResolver::CollisionResolver(Backend &backend) : backend(backend) {}

void CollisionResolver::solve_collision(PhysicBody* obj, PhysicBody* other_rigid, const CollisionInfo &collision_info) {
    backend.display_vector(collision_info.collision_point, collision_info.penetration * collision_info.normal);

    obj->move(collision_info.penetration * collision_info.normal);

    const float e = (obj->elastic_coeff + other_rigid->elastic_coeff) /2;

    const Vector2D r_to_colpoint = collision_info.collision_point - obj->location.point2d;
    const Vector2D r_to_colpoint_perp{r_to_colpoint.y, -r_to_colpoint.x};
    const float r_perp_normal_dot = r_to_colpoint_perp.dot(collision_info.normal);

    const Vector2D r_other_to_colpoint = collision_info.collision_point - other_rigid->location.point2d;
    const Vector2D r_other_to_colpoint_perp{r_other_to_colpoint.y, -r_other_to_colpoint.x};
    const float r_other_perp_normal_dot = r_other_to_colpoint_perp.dot(collision_info.normal);

    const Vector2D speed_ab = obj->velocity - obj->rotation_speed * r_to_colpoint_perp
                              - (other_rigid->velocity - other_rigid->rotation_speed * r_other_to_colpoint_perp);

    const float j = - (1+e) * speed_ab.dot(collision_info.normal) /
                    (obj->get_mass_inverse() + other_rigid->get_mass_inverse()
                     + r_perp_normal_dot*r_perp_normal_dot*obj->get_inertia_inverse()
                     + r_other_perp_normal_dot*r_other_perp_normal_dot*other_rigid->get_inertia_inverse());

    const Vector2D impulse = j * collision_info.normal;

    obj->apply_impulse(collision_info.collision_point, impulse);
    other_rigid->apply_impulse(collision_info.collision_point, -impulse);


    // Add friction
    const float average_static_friction = (obj->static_friction + other_rigid->static_friction) / 2;
    const float average_dync_friction = (obj->dynamic_friction + other_rigid->dynamic_friction) / 2;
    Vector2D tangent = speed_ab - speed_ab.dot(collision_info.normal) * collision_info.normal;

    if (tangent.length() <= 0.001) {
        return;
    }
    tangent.normalize();
    const float jt = -speed_ab.dot(tangent);

    float final_jt = jt;
    if (std::abs(jt) > j * average_static_friction) {
        final_jt = j * average_dync_friction;
    }

    const Vector2D impulse_friction = final_jt * tangent;
    backend.display_vector(collision_info.collision_point, impulse_friction);

    obj->apply_impulse(collision_info.collision_point, impulse_friction);
    other_rigid->apply_impulse(collision_info.collision_point, -impulse_friction);
}


void CollisionResolver::update_locations(std::vector<std::unique_ptr<PhysicBody>> &physic_bodies, const float &dt) {

    for (auto &obj: physic_bodies) {
        const RigidBody* rigid_body = dynamic_cast<const RigidBody*>(obj.get());
        if (rigid_body == nullptr) {
            continue;
        }

        obj->update_location(dt);

        for (auto &other_obj: physic_bodies) {
            if (obj == other_obj) {
                continue;
            }

            CollisionInfo collision_info = CollisionResolver::are_colliding(*obj, *other_obj);

            if (collision_info.are_colliding) {
                solve_collision(obj.get(), other_obj.get(), collision_info);
            }
        }
    }
}

CollisionInfo CollisionResolver::are_colliding(const PhysicBody &body1, const PhysicBody &body2) {

    const Transform2D &location1 = body1.location;
    const Shape2D &shape1 = *body1.shape;

    const Transform2D &location2 = body2.location;
    const Shape2D &shape2 = *body2.shape;


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
                CollisionInfo coll_info = are_sphere_poly_colliding(location2, *circle2, location1, *poly1);
                return {coll_info.are_colliding, -coll_info.normal, coll_info.collision_point, coll_info.penetration};
            }

            const ConvexPolygonShape2D* poly2 = dynamic_cast<const ConvexPolygonShape2D*>(&shape2);
            if (poly2 != nullptr) {
                return are_polys_colliding(location1, *poly1, location2, *poly2);
            }
        }
    }

    return {false, Vector2D(0, 0), Vector2D(0, 0), 0};
}



CollisionInfo CollisionResolver::are_spheres_colliding(const Transform2D &loc1, const CircleShape2D &circle1,
                                const Transform2D &loc2, const CircleShape2D &circle2) {
    Vector2D loc_diff = loc1.point2d - loc2.point2d;
    float loc_dist = loc_diff.length();
    float circle_radiuses = circle1.r + circle2.r;
    float penetration = circle_radiuses - loc_dist;

    return {penetration > 0, loc_diff.get_unit_vector(),
            (loc1.point2d + loc2.point2d) /2, penetration};
}

CollisionInfo CollisionResolver::are_sphere_poly_colliding(const Transform2D &loc1, const CircleShape2D &circle1,
                                               const Transform2D &loc2, const ConvexPolygonShape2D &poly2) {
    const unsigned int nb_points = poly2.points.size();
    bool has_collision = false;
    Vector2D normal;
    Vector2D collision_point;
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
                collision_point = point1;
            }
        } else if (projection >= segment_length) {
            Vector2D seg_to_circle_2nd = loc1.point2d - point2;
            const float seg_to_circle_2nd_length = seg_to_circle_2nd.length();

            if (seg_to_circle_2nd_length < circle1.r) {
                has_collision = true;
                normal = seg_to_circle_2nd.get_unit_vector();
                penetration = circle1.r - seg_to_circle_2nd_length;
                collision_point = point2;
            }
        } else {
            Vector2D middle_point = point1 + projection * unit_vect;

            Vector2D seg_to_circle_2nd = loc1.point2d - middle_point;
            const float seg_to_circle_2nd_length = seg_to_circle_2nd.length();

            if (seg_to_circle_2nd_length < circle1.r) {
                has_collision = true;
                normal = seg_to_circle_2nd.get_unit_vector();
                penetration = circle1.r - seg_to_circle_2nd_length;
                collision_point = middle_point;
            }
        }

        if (has_collision) {
            break;
        }
    }

    return {has_collision, normal, collision_point, penetration};
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
