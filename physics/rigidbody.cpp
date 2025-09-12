#include "rigidbody.h"

RigidBody::RigidBody(const Transform2D &location, const std::shared_ptr<Shape2D> shape, const float &mass, const float &elastic_coeff):
    PhysicBody(location, shape, elastic_coeff), mass(mass) {
    inertia = shape->compute_inertia(mass);
}


void RigidBody::update_location(const float &delta_time) {
    velocity += acc;
    location.point2d += velocity * delta_time;

    rotation_speed += rotation_acc;
    location.rotation_rad += rotation_speed * delta_time;
}

void RigidBody::move(const Vector2D &vectorToNewLocation) {
    location.point2d += vectorToNewLocation;
}

void RigidBody::apply_impulse(const Vector2D &application_point, const Vector2D &impulse) {
    if (!is_kinematic) {
        const Vector2D r_to_colpoint = application_point - location.point2d;
        const Vector2D r_to_colpoint_perp{r_to_colpoint.y, -r_to_colpoint.x};

        velocity += impulse/mass;
        rotation_speed -= r_to_colpoint_perp.dot(impulse) / inertia;
    }
}

float RigidBody::get_mass() const noexcept {
    return mass;
}

float RigidBody::get_mass_inverse() const noexcept {
    return 1.0 / mass;
}

float RigidBody::get_inertia_inverse() const noexcept {
    return 1.0 / inertia;
}
