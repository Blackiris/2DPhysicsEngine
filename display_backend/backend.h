#ifndef BACKEND_H
#define BACKEND_H

#include "../physics/transform.h"
#include "../physics/shape2d.h"

enum class MouseButtonName {
    MOUSE_LEFT, MOUSE_RIGHT
};

class Backend {
public:
    virtual void init(const int &screen_width, const int &screen_height) = 0;
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void close() = 0;
    virtual bool should_window_closed() = 0;
    virtual float get_frame_time() = 0;
    virtual void display_shape(const Transform2D &location, const Shape2D &shape) = 0;
    virtual void display_vector(const Vector2D &origin, const Vector2D &direction) = 0;
    virtual Vector2D get_mouse_pos() const = 0;
    virtual bool is_mouse_button_pressed(const MouseButtonName &button_name) const = 0;
};

#endif // BACKEND_H
