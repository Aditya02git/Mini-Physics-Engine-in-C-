cmake_minimum_required(VERSION 3.10)
project(MiniPhysicsEngine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(mini_physics_engine
    src/main.cpp
    src/RigidBody.cpp
    src/CollisionDetection.cpp
    src/PhysicsWorld.cpp
)
