#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

#include "display_backend/backend.h"
#include "physics/rigidbody.h"
#include "physics/staticbody.h"

#include <memory>
#include <vector>
class GameLogic
{
public:
    GameLogic(Backend &backend);
    int start();

private:
    void display_shapes();
    void display_collisions();
    void init_level();

    Backend &backend;
    std::vector<std::unique_ptr<RigidBody>> rigids;
    std::vector<std::unique_ptr<StaticBody>> statics;
};

#endif // GAME_LOGIC_H
