#include "convex_polygon_shape2d.h"
#include <numeric>

ConvexPolygonShape2D::ConvexPolygonShape2D(const std::vector<Vector2D> points): points(points) {}

float ConvexPolygonShape2D::compute_inertia(const float &mass) const {
    auto lambda = [&](float a, Vector2D b){return a + mass * b.length_squared(); };
    return std::accumulate(points.begin(), points.end(), 0, lambda) / points.size();
}
