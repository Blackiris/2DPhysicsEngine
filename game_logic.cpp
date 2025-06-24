#include "game_logic.h"
#include "physics/circleshape2d.h"
#include "physics/collision_resolver.h"
#include "physics/convex_polygon_shape2d.h"
#include "physics/rectangleshape2d.h"
#include "physics/rigidbody.h"
#include "physics/staticbody.h"
#include <iostream>

GameLogic::GameLogic(Backend &backend): backend(backend) {}

int GameLogic::start() {

    backend.init(1000, 1000);
    const double physics_nb_sub_steps = 3;

    CircleShape2D circle2D(30);
    RigidBody circleBody(Transform2D(Vector2D(70, 100), 0), circle2D, 10, 0.8);
    circleBody.acc = Vector2D(0, 9.8);

    //RectangleShape2D rectangle2D(100, 10);
    //StaticBody rectangleStaticBody(Transform2D(Vector2D(50, 300), 0), rectangle2D);

    StaticBody circleStaticBody(Transform2D(Vector2D(60, 300), 0), circle2D);



    ConvexPolygonShape2D convex_shape_2D({Vector2D(200, 20), Vector2D(-20, 20), Vector2D(-20, -20), Vector2D(200, -20), Vector2D(450, -50)});
    StaticBody convex_static_body(Transform2D(Vector2D(200, 300), 0), convex_shape_2D);

    CollisionResolver collision_resolver;
    rigids = {&circleBody};
    statics = {&circleStaticBody, &convex_static_body};

    while(!backend.should_window_closed()) {
        backend.begin();
        const double delta_time = backend.get_frame_time();
        const double physics_delta_time = delta_time / physics_nb_sub_steps;

        for (int i=0; i<physics_nb_sub_steps; i++) {
           collision_resolver.update_locations(rigids, statics, physics_delta_time);
        }

        display_shapes();
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

