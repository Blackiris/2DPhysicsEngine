#include "vector3d.h"
#include <cmath>

Vector3D::Vector3D(): x(0), y(0), z(0) {}
Vector3D::Vector3D(const double &x, const double &y, const double &z) : x(x), y(y), z(z) {}
Vector3D::Vector3D(const Vector2D &vec2d): x(vec2d.x), y(vec2d.y), z(0) {}

Vector3D Vector3D::cross(const Vector3D &vec3d) const {
    Vector3D c3(0, 0, 0);
    c3.x = this->y * vec3d.z - this->z * vec3d.y;
    c3.y = this->z * vec3d.x - this->x * vec3d.z;
    c3.z = this->x * vec3d.y - this->y * vec3d.x;
    return c3;
}

Vector3D Vector3D::operator+(const Vector3D &vec3d) const {
    Vector3D c3(0, 0, 0);
    c3.x = this->x + vec3d.x;
    c3.y = this->y + vec3d.y;
    c3.z = this->z + vec3d.z;
    return c3;
}

float Vector3D::length() const {
    return std::sqrt(x*x+y*y+z*z);
}


Vector3D& Vector3D::normalize() {
    const float len = length();
    x /= len;
    y /= len;
    z /= len;

    return *this;
}

Vector2D Vector3D::toVec2D() const {
    return Vector2D(x, y);
}
