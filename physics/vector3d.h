#ifndef VECTOR3D_H
#define VECTOR3D_H

#include "vector2d.h"

class Vector3D
{
public:
    Vector3D();
    Vector3D(const double &x, const double &y, const double &z);
    Vector3D(const Vector2D &vec2d);
    double x = 0, y = 0, z = 0;

    Vector3D cross(const Vector3D &vec3d) const;
    Vector3D operator+(const Vector3D &vec3d) const;

    float length() const;
    Vector3D& normalize();

    Vector2D toVec2D() const;
};

#endif // VECTOR3D_H
