cmake_minimum_required(VERSION 3.16)

project(2DPhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib 5.5 REQUIRED)

add_executable(2DPhysicsEngine main.cpp
    display_backend/backend.h
    display_backend/raylib_display_backend.h display_backend/raylib_display_backend.cpp
    physics/rigidbody.h physics/rigidbody.cpp


    physics/vector2d.h physics/vector2d.cpp
    physics/shape2d.h physics/shape2d.cpp
    physics/circleshape2d.h physics/circleshape2d.cpp
    physics/transform.h
    physics/staticbody.h physics/staticbody.cpp
    physics/physicbody.h physics/physicbody.cpp
    physics/rectangleshape2d.h physics/rectangleshape2d.cpp
    game_logic.h game_logic.cpp
    physics/collision_resolver.h physics/collision_resolver.cpp
    physics/collision_info.h
    physics/convex_polygon_shape2d.h physics/convex_polygon_shape2d.cpp
    physics/matrix3x3.h physics/matrix3x3.cpp
    physics/vector3d.h physics/vector3d.cpp
)
target_link_libraries(${PROJECT_NAME} raylib)

include(GNUInstallDirs)
install(TARGETS 2DPhysicsEngine
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
