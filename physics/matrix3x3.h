#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "vector3d.h"

#include <array>
class Matrix3x3
{
public:
    Matrix3x3();
    Matrix3x3(std::initializer_list<std::initializer_list<double>> init_values);

    std::array<std::array<double, 3>, 3> values;

    Vector3D operator*(const Vector3D &vec3d) const;
};

#endif // MATRIX3X3_H
