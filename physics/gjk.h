#ifndef GJK_H
#define GJK_H

#include "collision_info.h"
#include "convex_polygon_shape2d.h"

namespace gjk {
    CollisionInfo are_polys_colliding(const ConvexPolygonShape2D &poly1, const ConvexPolygonShape2D &poly2);
};

#endif // GJK_H
