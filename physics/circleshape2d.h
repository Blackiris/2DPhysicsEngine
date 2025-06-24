#ifndef CIRCLESHAPE2D_H
#define CIRCLESHAPE2D_H

#include "shape2d.h"

class CircleShape2D : public Shape2D
{
public:
    CircleShape2D(const float &r);
    float r;
};

#endif // CIRCLESHAPE2D_H
