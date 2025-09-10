#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

#include "display_backend/backend.h"
#include "physics/rigidbody.h"
#include "physics/staticbody.h"

#include <list>
class GameLogic
{
public:
    GameLogic(Backend &backend);
    int start();

private:
    void display_shapes();
    void display_collisions();

    Backend &backend;
    std::list<RigidBody*> rigids;
    std::list<StaticBody*> statics;
};

#endif // GAME_LOGIC_H
