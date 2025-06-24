#ifndef RECTANGLESHAPE2D_H
#define RECTANGLESHAPE2D_H

#include "shape2d.h"

class RectangleShape2D : public Shape2D
{
public:
    RectangleShape2D(const float &width, const float &height);

    float width, height;
};

#endif // RECTANGLESHAPE2D_H
