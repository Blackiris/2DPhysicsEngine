#include "rigidbody.h"

RigidBody::RigidBody(const Transform2D &location, const Shape2D &shape, const float &mass, const float &elastic_coeff):
    PhysicBody(location, shape, elastic_coeff), location_old(location), mass(mass) {
    inertia = shape.compute_intertia(mass);
}


void RigidBody::update_location(float delta_time) {
    // Verlet integration
    //Location current_location_before_upd = location;
    //location.point2d = location.point2d * 2 - location_old.point2d + acc * delta_time * delta_time;
    //location_old = current_location_before_upd;
    velocity += acc;
    location.point2d += velocity * delta_time;

    rotation_speed += rotation_acc;
    location.rotation_rad += rotation_speed * delta_time;
}

void RigidBody::move(const Vector2D &vectorToNewLocation) {
    location.point2d += vectorToNewLocation;
}
