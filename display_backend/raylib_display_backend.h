#ifndef RAYLIB_DISPLAY_BACKEND_H
#define RAYLIB_DISPLAY_BACKEND_H

#include "backend.h"

class RaylibDisplayBackend : public Backend
{
public:
    RaylibDisplayBackend();

    void init(const int &screen_width, const int &screen_height) override;
    void begin() override;
    void end() override;
    void close() override;
    bool should_window_closed() override;
    float get_frame_time() override;
    void display_shape(const Transform2D &location, const Shape2D &shape) override;
    void display_vector(const Vector2D &origin, const Vector2D &direction) override;
    Vector2D get_mouse_pos() const override;
    bool is_mouse_button_pressed(const MouseButtonName &button_name) const override;
};

#endif // RAYLIB_DISPLAY_BACKEND_H
