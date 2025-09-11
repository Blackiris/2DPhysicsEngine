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

    virtual void update_location(const float &delta_time);
    virtual void move(const Vector2D &vectorToNewLocation);
    virtual void apply_impulse(const Vector2D &application_point, const Vector2D &impulse);

};

#endif // PHYSICBODY_H
