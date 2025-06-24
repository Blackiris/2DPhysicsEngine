#ifndef LOCATION_H
#define LOCATION_H

#include "vector2d.h"

struct Transform2D {
    Vector2D point2d;
    float rotation_rad;

    // Surcharge de l'opérateur + en tant que membre
    Vector2D operator*(const Vector2D& p) const {
        return {point2d.x + p.x, point2d.y + p.y};
    }

};

#endif // LOCATION_H
