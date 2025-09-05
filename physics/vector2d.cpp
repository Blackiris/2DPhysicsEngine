#include "vector2d.h"
#include <cmath>

Vector2D::Vector2D(): x(0), y(0) {}
Vector2D::Vector2D(const double &x, const double &y) : x(x), y(y) {}

float Vector2D::length() const {
    return std::sqrt(x*x+y*y);
}

Vector2D Vector2D::get_unit_vector() const {
    float len = length();
    return Vector2D(x/len, y/len);
}

Vector2D& Vector2D::normalize() {
    float len = length();
    x /= len;
    y /= len;
    return *this;
}

float Vector2D::dot(const Vector2D &vec2d) const {
    return x * vec2d.x + y * vec2d.y;
}

Vector2D Vector2D::operator-() const {
    return Vector2D(-x, -y);
}

Vector2D Vector2D::operator+(const Vector2D &vec2d) const {
    Vector2D c3(0, 0);
    c3.x = this->x + vec2d.x;
    c3.y = this->y + vec2d.y;
    return c3;
}

Vector2D Vector2D::operator-(const Vector2D &vec2d) const {
    Vector2D c3(0, 0);
    c3.x = this->x - vec2d.x;
    c3.y = this->y - vec2d.y;
    return c3;
}

Vector2D Vector2D::operator*(const double &coeff) const {
    Vector2D c3(0, 0);
    c3.x = coeff * this->x;
    c3.y = coeff * this->y;
    return c3;
}

Vector2D Vector2D::operator/(const double &coeff) const {
    Vector2D c3(0, 0);
    c3.x = this->x / coeff;
    c3.y = this->y / coeff;
    return c3;
}

Vector2D& Vector2D::operator+=(const Vector2D& vec2d) {
    x += vec2d.x;
    y += vec2d.y;
    return *this; // retourne l'objet modifié
}

Vector2D& Vector2D::operator-=(const Vector2D& vec2d) {
    x -= vec2d.x;
    y -= vec2d.y;
    return *this; // retourne l'objet modifié
}
