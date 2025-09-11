#ifndef SHAPE2D_H
#define SHAPE2D_H

class Shape2D
{
public:
    Shape2D();
    virtual ~Shape2D();

    virtual float compute_inertia(const float &mass) const = 0;
};

#endif // SHAPE2D_H
