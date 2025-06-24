#include "staticbody.h"

StaticBody::StaticBody(const Transform2D &location, const Shape2D &shape): PhysicBody(location, shape, 1.0) {}
