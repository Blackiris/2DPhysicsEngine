#include "raylib_display_backend.h"
#include "raylib.h"

#include "../physics/circleshape2d.h"
#include "../physics/convex_polygon_shape2d.h"

RaylibDisplayBackend::RaylibDisplayBackend() {}

void RaylibDisplayBackend::init(const int &screen_width, const int &screen_height) {
    InitWindow(screen_width, screen_height, "2D Physics");
    SetTargetFPS(30);
}

void RaylibDisplayBackend::begin() {
    BeginDrawing();
    ClearBackground(RAYWHITE);
}

void RaylibDisplayBackend::end() {
    EndDrawing();
}

void RaylibDisplayBackend::close() {
    CloseWindow();
}

bool RaylibDisplayBackend::should_window_closed() {
    return WindowShouldClose();
}

float RaylibDisplayBackend::get_frame_time() {
    return GetFrameTime();
}

void RaylibDisplayBackend::display_shape(const Transform2D &location, const Shape2D &shape) {

    const CircleShape2D* circle = dynamic_cast<const CircleShape2D*>(&shape);
    if (circle != nullptr) {
        DrawCircleLines(location.point2d.x, location.point2d.y, circle->r, DARKBLUE);
        return;
    }

    const ConvexPolygonShape2D* convex = dynamic_cast<const ConvexPolygonShape2D*>(&shape);
    if (convex != nullptr) {
        std::vector<Vector2> points_rl;
        for (const auto& point: convex->points) {
            const Vector2D new_point = location * point;
            points_rl.emplace_back(Vector2(new_point.x, new_point.y));
        }
        const Vector2D new_point = location * convex->points[0];
        points_rl.emplace_back(Vector2(new_point.x, new_point.y));
        Vector2* points_array = &points_rl[0];

        DrawLineStrip(points_array, points_rl.size(), DARKBLUE);
    }
}

void RaylibDisplayBackend::display_vector(const Vector2D &origin, const Vector2D &direction) {
    Vector2D target = origin+direction;
    DrawCircle(origin.x, origin.y, 5, RED);
    DrawLineEx(Vector2(origin.x, origin.y), Vector2(target.x, target.y), 3, RED);
}
