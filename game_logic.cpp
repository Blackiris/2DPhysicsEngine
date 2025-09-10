#include "game_logic.h"
#include "physics/circleshape2d.h"
#include "physics/collision_resolver.h"
#include "physics/convex_polygon_shape2d.h"
#include "physics/rigidbody.h"
#include "physics/staticbody.h"

#include <raylib.h>

GameLogic::GameLogic(Backend &backend): backend(backend) {}

int GameLogic::start() {

    backend.init(1000, 1000);
    const double physics_nb_sub_steps = 1;

    CircleShape2D circle2D(30);
    RigidBody circleBody1(Transform2D(Vector2D(70, 100), 0), circle2D, 10, 0.8);
    circleBody1.acc = Vector2D(0, 9.8);
    RigidBody circleBody2(Transform2D(Vector2D(190, 400), 0), circle2D, 30, 0.8);

    ConvexPolygonShape2D squareShape({Vector2D(-50, -50), Vector2D(50, -50), Vector2D(50, 50), Vector2D(-50, 50)});
    RigidBody squareBody(Transform2D(Vector2D(500, 200), 0), squareShape, 10, 0.8);
    RigidBody squareBody2(Transform2D(Vector2D(190, 300), 0), squareShape, 12, 0.8);
    squareBody.acc = Vector2D(0, 0);
    squareBody.rotation_speed = 0.01;


    StaticBody circleStaticBody(Transform2D(Vector2D(60, 300), 0), circle2D);

    ConvexPolygonShape2D convex_shape_2D({Vector2D(200, 20), Vector2D(-60, 20), Vector2D(-60, -20), Vector2D(200, -20), Vector2D(450, -10)});
    StaticBody convex_static_body(Transform2D(Vector2D(500, 300), 0.2), convex_shape_2D);

    CollisionResolver collision_resolver{backend};
    rigids = {&circleBody1, &circleBody2, &squareBody, &squareBody2};
    statics = {&circleStaticBody, &convex_static_body};

    while (!backend.should_window_closed()) {
        backend.begin();
        const double delta_time = backend.get_frame_time();
        const double physics_delta_time = delta_time / physics_nb_sub_steps;

        for (int i=0; i<physics_nb_sub_steps; i++) {
           collision_resolver.update_locations(rigids, statics, physics_delta_time);
        }
        display_shapes();
        convex_static_body.location.rotation_rad -= 0.01;
        backend.end();
    }
    backend.close();

    return 0;
}

void GameLogic::display_shapes() {
    for (auto& rigid : rigids) {
        backend.display_shape(rigid->location, rigid->shape);
    }

    for (auto& static_body : statics) {
        backend.display_shape(static_body->location, static_body->shape);
    }
}

