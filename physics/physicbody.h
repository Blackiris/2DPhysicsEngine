#ifndef PHYSICBODY_H
#define PHYSICBODY_H

#include "shape2d.h"
#include "transform.h"
#include <memory>

class PhysicBody
{
public:
    PhysicBody(const Transform2D &location, const std::shared_ptr<Shape2D> shape, const float &elastic_coeff);

    Transform2D location;
    const std::shared_ptr<Shape2D> shape;
    float elastic_coeff = 1;
    float static_friction = 0.6;
    float dynamic_friction = 0.5;

    virtual void update_location(float delta_time);
    virtual void move(const Vector2D &vectorToNewLocation);

};

#endif // PHYSICBODY_H
