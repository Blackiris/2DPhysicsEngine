#ifndef GJK_H
#define GJK_H

#include "collision_info.h"
#include "convex_polygon_shape2d.h"

namespace gjk {
    CollisionInfo are_polys_colliding(const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2);
    Vector2D get_support(const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2, const Vector2D &direction);
    Vector2D get_furthest_point(const ConvexPolygonShape2D &poly, const Vector2D &direction);
    bool same_direction(const Vector2D &v1, const Vector2D &v2);
};

#endif // GJK_H
