#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "physicbody.h"
#include "shape2d.h"
#include "vector2d.h"

class RigidBody : public PhysicBody
{
public:
    RigidBody(const Transform2D &location, const Shape2D &shape, const float &mass, const float &elastic_coeff);

    Transform2D location_old;
    Vector2D acc;
    Vector2D velocity;
    float mass;

    void update_location(float delta_time) override;
    void move(const Vector2D &vectorToNewLocation) override;
};

#endif // RIGIDBODY_H
