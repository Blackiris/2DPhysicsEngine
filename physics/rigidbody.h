#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "physicbody.h"
#include "shape2d.h"
#include "vector2d.h"

class RigidBody : public PhysicBody
{
private:
    float mass;
public:
    RigidBody(const Transform2D &location, const std::shared_ptr<Shape2D>shape, const float &mass, const float &elastic_coeff);

    Transform2D location_old;
    Vector2D acc{0, 0};

    float rotation_acc = 0;
    float inertia;


    void update_location(const float &delta_time) override;
    void move(const Vector2D &vectorToNewLocation) override;
    void apply_impulse(const Vector2D &application_point, const Vector2D &impulse) override;
    float get_mass() const noexcept override;
    float get_mass_inverse() const noexcept override;

    float get_inertia_inverse() const noexcept;
};

#endif // RIGIDBODY_H
