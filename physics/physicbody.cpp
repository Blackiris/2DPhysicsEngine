#include "physicbody.h"

PhysicBody::PhysicBody(const Transform2D &location, const std::shared_ptr<Shape2D> shape, const float &elastic_coeff)
    : location(location), shape(shape), elastic_coeff(elastic_coeff) {}

void PhysicBody::move(const Vector2D &vectorToNewLocation) {
    // Do nothing
}

void PhysicBody::update_location(const float &delta_time) {
    // Do nothing
}

void PhysicBody::apply_impulse(const Vector2D &application_point, const Vector2D &impulse) {
    // Do nothing
}

float PhysicBody::get_mass() const noexcept {
    return 0.0;
}

float PhysicBody::get_mass_inverse() const noexcept {
    return 0.0;
}

float PhysicBody::get_inertia_inverse() const noexcept {
    return 0.0;
}
