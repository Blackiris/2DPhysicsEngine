#include "matrix3x3.h"

#include <stdexcept>

Matrix3x3::Matrix3x3() {}

Matrix3x3::Matrix3x3(std::initializer_list<std::initializer_list<double>> init_values) {
    const int nb_rows = init_values.size();
    if (nb_rows != 3) {
        throw std::invalid_argument("Not an 3x3 matrix");
    }

    for (const auto& l: init_values) {
        if (l.size() != 3) {
            throw std::invalid_argument("Not an 3x3 matrix");
        }
    }

    size_t i = 0;
    for (const auto& column: init_values) {
        size_t j = 0;
        for (const auto& val: column) {
            values[i][j] = val;
            j++;
        }
        i++;
    }
}

Vector3D Matrix3x3::operator*(const Vector3D &vec3d) const {
    const double new_x = values[0][0] * vec3d.x + values[1][0] * vec3d.y + values[2][0] * vec3d.z;
    const double new_y = values[0][1] * vec3d.x + values[1][1] * vec3d.y + values[2][1] * vec3d.z;
    const double new_z = values[0][2] * vec3d.x + values[1][2] * vec3d.y + values[2][2] * vec3d.z;

    return Vector3D(new_x, new_y, new_z);
}
