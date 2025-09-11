#ifndef COLLISION_RESOLVER_H
#define COLLISION_RESOLVER_H

#include "circleshape2d.h"
#include "collision_info.h"
#include "convex_polygon_shape2d.h"
#include "physicbody.h"
#include "rigidbody.h"
#include "staticbody.h"

#include "../display_backend/backend.h"

#include <vector>
#include <memory>

class CollisionResolver
{
public:
    CollisionResolver(Backend &backend);
    void update_locations(std::vector<std::unique_ptr<RigidBody>> &rigid_bodies, std::vector<std::unique_ptr<StaticBody>> &static_bodies, const float &dt);
    static CollisionInfo are_colliding(const PhysicBody &body1, const PhysicBody &body2);

private:
    static CollisionInfo are_spheres_colliding(const Transform2D &loc1, const CircleShape2D &circle1,
                                    const Transform2D &loc2, const CircleShape2D &circle2);
    static CollisionInfo are_sphere_poly_colliding(const Transform2D &loc1, const CircleShape2D &circle1,
                                               const Transform2D &loc2, const ConvexPolygonShape2D &poly2);
    static CollisionInfo are_polys_colliding(const Transform2D &loc1, const ConvexPolygonShape2D &poly1,
                                             const Transform2D &loc2, const ConvexPolygonShape2D &poly2);

    Backend &backend;
};

#endif // COLLISION_RESOLVER_H
