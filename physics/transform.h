#ifndef LOCATION_H
#define LOCATION_H

#include "matrix3x3.h"
#include "vector2d.h"
#include <cmath>

struct Transform2D {
    Vector2D point2d;
    float rotation_rad;

    // Surcharge de l'opérateur + en tant que membre
    Vector2D operator*(const Vector2D& p) const {
        const double cos_rot = std::cos(rotation_rad);
        const double sin_rot = std::sin(rotation_rad);
        Matrix3x3 transform_matrix({{cos_rot, sin_rot, 0},
                                    {-sin_rot, cos_rot, 0},
                                    {point2d.x, point2d.y, 1}});

        Vector3D pos = transform_matrix * Vector3D(p.x, p.y, 1);


        return {pos.x, pos.y};
    }

};

#endif // LOCATION_H
