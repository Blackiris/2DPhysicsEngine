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
    auto circle_shape = std::make_shared<CircleShape2D>(30);
    auto circle_body1 = std::make_unique<RigidBody>(Transform2D(Vector2D(70, 50), 0), circle_shape, 10, 0.8);
    circle_body1->acc = Vector2D(0, 9.8);
    auto circle_body2 = std::make_unique<RigidBody>(Transform2D(Vector2D(300, 350), 0), circle_shape, 30, 0.8);

    auto square_shape = std::make_shared<ConvexPolygonShape2D>(std::initializer_list<Vector2D>({Vector2D(-40, -50), Vector2D(30, -50), Vector2D(60, 60), Vector2D(0, 70), Vector2D(-50, 50)}));
    auto square_body = std::make_unique<RigidBody>(Transform2D(Vector2D(500, 450), 0), square_shape, 50, 0.5);
    auto square_body2 = std::make_unique<RigidBody>(Transform2D(Vector2D(190, 250), 0), square_shape, 12, 0.8);
    square_body->acc = Vector2D(0, 9.8);
    square_body->rotation_speed = 0.01;
    square_body->dynamic_friction = 0.9;
    square_body->static_friction = 1;

    rigids.push_back(std::move(circle_body1));
    rigids.push_back(std::move(circle_body2));
    rigids.push_back(std::move(square_body));
    rigids.push_back(std::move(square_body2));

    auto convex_shape_2D = std::make_shared<ConvexPolygonShape2D>(std::initializer_list<Vector2D>({Vector2D(200, 50), Vector2D(-60, 50), Vector2D(-60, -20), Vector2D(200, -20), Vector2D(550, -10)}));

    rigids.push_back(std::make_unique<StaticBody>(Transform2D(Vector2D(60, 250), 0), circle_shape));
    rigids.push_back(std::make_unique<StaticBody>(Transform2D(Vector2D(350, 650), -0.10), convex_shape_2D));
    rigids.push_back(std::make_unique<StaticBody>(Transform2D(Vector2D(50, 400), 0.20), convex_shape_2D));


    auto rect_shape = std::make_shared<ConvexPolygonShape2D>(std::initializer_list<Vector2D>({Vector2D(-90, -15), Vector2D(90, -15), Vector2D(90, 15), Vector2D(-90, 15)}));
    auto rect_body = std::make_unique<RigidBody>(Transform2D(Vector2D(800, 550), 0), rect_shape, 50, 0.5);
    rect_body->rotation_speed = -0.8;
    rect_body->is_kinematic = true;
    rigids.push_back(std::move(rect_body));
}

void display_help() {
    DrawText("Right click to create box", 20, 20, 20, RED);
    DrawText("Left click to create circle", 20, 50, 20, RED);
}

int GameLogic::start() {

    backend.init(1000, 1000);
    const double physics_nb_sub_steps = 3;
    init_level();

    CollisionResolver collision_resolver{backend};

    while (!backend.should_window_closed()) {
        backend.begin();

        Vector2D mouse_pos = { -100.0f, -100.0f };

        mouse_pos = backend.get_mouse_pos();
        if (backend.is_mouse_button_pressed(MouseButtonName::MOUSE_LEFT)) {
            auto circle2D = std::make_shared<CircleShape2D>(30);
            auto circle_body = std::make_unique<RigidBody>(Transform2D(mouse_pos, 0), circle2D, 10, 0.8);
            circle_body->acc = Vector2D(0, 9.8);
            rigids.push_back(std::move(circle_body));
        }

        if (backend.is_mouse_button_pressed(MouseButtonName::MOUSE_RIGHT)) {
            auto squareShape = std::make_shared<ConvexPolygonShape2D>(std::initializer_list<Vector2D>({Vector2D(-50, -50), Vector2D(50, -50), Vector2D(50, 50), Vector2D(-50, 50)}));
            auto squareBody = std::make_unique<RigidBody>(Transform2D(mouse_pos, 0), squareShape, 50, 0.5);
            squareBody->acc = Vector2D(0, 9.8);
            rigids.push_back(std::move(squareBody));
        }


        const double delta_time = backend.get_frame_time();
        const double physics_delta_time = delta_time / physics_nb_sub_steps;

        for (int i=0; i<physics_nb_sub_steps; i++) {
           collision_resolver.update_locations(rigids, physics_delta_time);
        }
        display_shapes();

        display_help();
        backend.end();
    }
    backend.close();

    return 0;
}

void GameLogic::display_shapes() {
    for (auto& rigid : rigids) {
        backend.display_shape(rigid->location, *rigid->shape);
    }
}

