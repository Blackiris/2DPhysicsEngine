#include "game_logic.h"
#include "physics/circleshape2d.h"
#include "physics/collision_resolver.h"
#include "physics/convex_polygon_shape2d.h"
#include "physics/rigidbody.h"
#include "physics/staticbody.h"

#include <raylib.h>

GameLogic::GameLogic(Backend &backend): backend(backend) {}

struct Level {

};

void GameLogic::init_level() {
    auto circle2D = std::make_shared<CircleShape2D>(30);
    auto circleBody1 = std::make_unique<RigidBody>(Transform2D(Vector2D(70, 100), 0), circle2D, 10, 0.8);
    circleBody1->acc = Vector2D(0, 5);
    auto circleBody2 = std::make_unique<RigidBody>(Transform2D(Vector2D(300, 400), 0), circle2D, 30, 0.8);

    auto squareShape = std::make_shared<ConvexPolygonShape2D>(std::initializer_list<Vector2D>({Vector2D(-50, -50), Vector2D(50, -50), Vector2D(50, 50), Vector2D(-50, 50)}));
    auto squareBody = std::make_unique<RigidBody>(Transform2D(Vector2D(500, 500), 0), squareShape, 50, 0.5);
    auto squareBody2 = std::make_unique<RigidBody>(Transform2D(Vector2D(190, 300), 0), squareShape, 12, 0.8);
    squareBody->acc = Vector2D(0, 2);
    squareBody->rotation_speed = 0.01;
    //squareBody->inertia = 1'333'333;
    squareBody->dynamic_friction = 0.9;
    squareBody->static_friction = 1;


    auto circleStaticBody = std::make_unique<StaticBody>(Transform2D(Vector2D(60, 300), 0), circle2D);

    auto convex_shape_2D = std::make_shared<ConvexPolygonShape2D>(std::initializer_list<Vector2D>({Vector2D(200, 50), Vector2D(-60, 50), Vector2D(-60, -20), Vector2D(200, -20), Vector2D(650, -10)}));
    auto convex_static_body2 = std::make_unique<StaticBody>(Transform2D(Vector2D(100, 700), -0.05), convex_shape_2D);

    rigids.push_back(std::move(circleBody1));
    rigids.push_back(std::move(circleBody2));
    rigids.push_back(std::move(squareBody));
    rigids.push_back(std::move(squareBody2));

    statics.push_back(std::move(circleStaticBody));
    statics.push_back(std::move(convex_static_body2));
}

int GameLogic::start() {

    backend.init(1000, 1000);
    const double physics_nb_sub_steps = 3;
    init_level();

    CollisionResolver collision_resolver{backend};

    while (!backend.should_window_closed()) {
        backend.begin();
        const double delta_time = backend.get_frame_time();
        const double physics_delta_time = delta_time / physics_nb_sub_steps;

        for (int i=0; i<physics_nb_sub_steps; i++) {
           collision_resolver.update_locations(rigids, statics, physics_delta_time);
        }
        display_shapes();

        //DrawText(TextFormat("Score: %02.02f", squareBody.rotation_speed), 200, 80, 20, RED);
        backend.end();
    }
    backend.close();

    return 0;
}

void GameLogic::display_shapes() {
    for (auto& rigid : rigids) {
        backend.display_shape(rigid->location, *rigid->shape);
    }

    for (auto& static_body : statics) {
        backend.display_shape(static_body->location, *static_body->shape);
    }
}

