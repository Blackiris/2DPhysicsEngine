#ifndef PHYSICBODY_H
#define PHYSICBODY_H

#include "transform.h"
#include "shape2d.h"

class PhysicBody
{
public:
    PhysicBody(const Transform2D &location, const Shape2D &shape, const float &elastic_coeff);

    Transform2D location;
    const Shape2D &shape;
    float elastic_coeff = 1;

    virtual void update_location(float delta_time);
    virtual void move(const Vector2D &vectorToNewLocation);

};

#endif // PHYSICBODY_H
