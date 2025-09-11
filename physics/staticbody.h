#ifndef STATICBODY_H
#define STATICBODY_H

#include "physicbody.h"

class StaticBody : public PhysicBody
{
public:
    StaticBody(const Transform2D &location, const std::shared_ptr<Shape2D> shape);
};

#endif // STATICBODY_H
