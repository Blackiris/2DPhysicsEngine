#ifndef COLLISION_INFO_H
#define COLLISION_INFO_H

#include "vector2d.h"

struct CollisionInfo {
    bool are_colliding;
    Vector2D normal;
    float penetration;
};

#endif // COLLISION_INFO_H
