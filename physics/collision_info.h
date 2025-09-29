#ifndef COLLISION_INFO_H
#define COLLISION_INFO_H

#include "vector2d.h"

struct CollisionInfo {
    Vector2D normal;
    Vector2D collision_point;
    float penetration;
};

#endif // COLLISION_INFO_H
