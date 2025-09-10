#include "circleshape2d.h"

CircleShape2D::CircleShape2D(const float &r) : r(r) {}

float CircleShape2D::compute_intertia(const float &mass) const {
    return mass * r * r / 2;
}
