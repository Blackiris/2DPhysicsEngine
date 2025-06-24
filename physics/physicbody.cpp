#include "physicbody.h"

PhysicBody::PhysicBody(const Transform2D &location, const Shape2D &shape, const float &elastic_coeff)
    : location(location), shape(shape), elastic_coeff(elastic_coeff) {}

void PhysicBody::move(const Vector2D &vectorToNewLocation) {
    // Do nothing
}

void PhysicBody::update_location(float delta_time) {
    // Do nothing
}
