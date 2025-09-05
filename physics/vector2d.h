#ifndef VECTOR2D_H
#define VECTOR2D_H

class Vector2D
{
public:
    Vector2D();
    Vector2D(const double &x, const double &y);
    double x = 0, y = 0;

    float length() const;
    Vector2D get_unit_vector() const;
    Vector2D& normalize();

    float dot(const Vector2D &vec2d) const;
    Vector2D operator-() const;
    Vector2D operator+(const Vector2D &vec2d) const;
    Vector2D operator-(const Vector2D &vec2d) const;
    Vector2D operator*(const double &coeff) const;
    // This friend function does the magic of float * vector
    friend Vector2D operator*(float scalar, const Vector2D& v) {
        return v * scalar;
    }

    Vector2D operator/(const double &coeff) const;

    Vector2D& operator+=(const Vector2D& vec2d);
    Vector2D& operator-=(const Vector2D& vec2d);
};

#endif // VECTOR2D_H
