#ifndef BACKEND_H
#define BACKEND_H

#include "../physics/transform.h"
#include "../physics/shape2d.h"

class Backend {
public:
    virtual void init(const int &screen_width, const int &screen_height) = 0;
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void close() = 0;
    virtual bool should_window_closed() = 0;
    virtual float get_frame_time() = 0;
    virtual void display_shape(const Transform2D &location, const Shape2D &shape) = 0;
};

#endif // BACKEND_H
