#ifndef RAYLIB_DISPLAY_BACKEND_H
#define RAYLIB_DISPLAY_BACKEND_H

#include "backend.h"
#include "raylib.h"

class RaylibDisplayBackend : public Backend
{
public:
    RaylibDisplayBackend();

    void init(const int &screen_width, const int &screen_height);
    void begin();
    void end();
    void close();
    bool should_window_closed();
    float get_frame_time();
    void display_shape(const Transform2D &location, const Shape2D &shape);
};

#endif // RAYLIB_DISPLAY_BACKEND_H
