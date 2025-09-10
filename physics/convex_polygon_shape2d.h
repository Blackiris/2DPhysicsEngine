#ifndef CONVEX_POLYGON_SHAPE2D_H
#define CONVEX_POLYGON_SHAPE2D_H

#include "shape2d.h"
#include "vector2d.h"

#include <vector>
class ConvexPolygonShape2D : public Shape2D
{
public:
    ConvexPolygonShape2D(const std::vector<Vector2D> points);

    std::vector<Vector2D> points;

    float compute_intertia(const float &mass) const;
};

#endif // CONVEX_POLYGON_SHAPE2D_H
