#include "raylib_display_backend.h"


#include "../physics/circleshape2d.h"
#include "../physics/rectangleshape2d.h"
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
        DrawCircle(location.point2d.x, location.point2d.y, circle->r, DARKBLUE);
        return;
    }

    const RectangleShape2D* rectangle = dynamic_cast<const RectangleShape2D*>(&shape);
    if (rectangle != nullptr) {
        DrawRectanglePro(Rectangle(location.point2d.x, location.point2d.y, rectangle->width, rectangle->height), Vector2(0, 0), 0, DARKBLUE);
        return;
    }

    const ConvexPolygonShape2D* convex = dynamic_cast<const ConvexPolygonShape2D*>(&shape);
    if (convex != nullptr) {
        std::vector<Vector2> points_rl;
        for (const auto& point: convex->points) {
            points_rl.emplace_back(Vector2(location.point2d.x+point.x, location.point2d.y+point.y));
        }
        points_rl.emplace_back(Vector2(location.point2d.x+convex->points[0].x, location.point2d.y+convex->points[0].y));
        Vector2* points_array = &points_rl[0];

        DrawLineStrip(points_array, points_rl.size(), DARKBLUE);
    }
}

