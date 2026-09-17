// viz_main.cpp
// Real-time visualization of the physics demo scene using raylib.
// Keeps main.cpp untouched (console demo); this is a separate entry point.
//
// Build (MSYS2 MinGW64 terminal, from the project root):
//   g++ -std=c++17 -Iinclude src/viz_main.cpp src/RigidBody.cpp \
//       src/CollisionDetection.cpp src/PhysicsWorld.cpp -o viz.exe \
//       -lraylib -lopengl32 -lgdi32 -lwinmm
// Run:
//   ./viz.exe

#include "raylib.h"
#include "PhysicsWorld.hpp"

// Draw one RigidBody according to its shape type.
static void DrawBody(const RigidBody& body, Color color) {
    Vector3 pos = { body.position.x, body.position.y, body.position.z };

    switch (body.shapeType) {
        case ShapeType::Sphere:
            DrawSphere(pos, body.sphere.radius, color);
            DrawSphereWires(pos, body.sphere.radius, 8, 8, Fade(BLACK, 0.3f));
            break;

        case ShapeType::Box: {
            Vector3 size = {
                body.box.halfExtents.x * 2.0f,
                body.box.halfExtents.y * 2.0f,
                body.box.halfExtents.z * 2.0f
            };
            DrawCube(pos, size.x, size.y, size.z, color);
            DrawCubeWires(pos, size.x, size.y, size.z, Fade(BLACK, 0.3f));
            break;
        }

        case ShapeType::Compound:
            for (const auto& child : body.compound.children) {
                Vector3 childPos = {
                    pos.x + child.localOffset.x,
                    pos.y + child.localOffset.y,
                    pos.z + child.localOffset.z
                };
                DrawSphere(childPos, child.radius, color);
                DrawSphereWires(childPos, child.radius, 8, 8, Fade(BLACK, 0.3f));
            }
            break;
    }
}

int main() {
    const int screenWidth = 1000;
    const int screenHeight = 700;

    InitWindow(screenWidth, screenHeight, "Mini Physics Engine - Visualizer");
    SetTargetFPS(60);

    // --- Same demo scene as main.cpp ---
    PhysicsWorld world;

    RigidBody ground(ShapeType::Box, 0.0f);
    ground.position = Vec3(0.0f, -1.0f, 0.0f);
    ground.box.halfExtents = Vec3(50.0f, 1.0f, 50.0f);
    world.addBody(&ground);

    RigidBody sphere1(ShapeType::Sphere, 1.0f);
    sphere1.position = Vec3(0.0f, 5.0f, 0.0f);
    sphere1.sphere.radius = 0.5f;
    sphere1.restitution = 0.6f;
    world.addBody(&sphere1);

    RigidBody sphere2(ShapeType::Sphere, 1.0f);
    sphere2.position = Vec3(0.3f, 8.0f, 0.0f);
    sphere2.sphere.radius = 0.5f;
    sphere2.restitution = 0.6f;
    world.addBody(&sphere2);

    RigidBody hull(ShapeType::Compound, 4.0f);
    hull.position = Vec3(3.0f, 6.0f, 0.0f);
    hull.restitution = 0.2f;
    hull.compound.children = {
        { Vec3(-0.6f, 0.0f, 0.0f), 0.5f },
        { Vec3( 0.6f, 0.0f, 0.0f), 0.5f }
    };
    world.addBody(&hull);
    // --- end demo scene ---

    Camera3D camera = { 0 };
    camera.position = { 12.0f, 10.0f, 12.0f };
    camera.target   = { 0.0f, 2.0f, 0.0f };
    camera.up       = { 0.0f, 1.0f, 0.0f };
    camera.fovy     = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    const float dt = 1.0f / 60.0f;
    bool paused = false;

    while (!WindowShouldClose()) {
        UpdateCamera(&camera, CAMERA_ORBITAL);

        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (!paused) {
            world.step(dt);
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawGrid(50, 1.0f);

        // Ground (drawn as a flat plane visual, matching its half-extents)
        DrawBody(ground, LIGHTGRAY);

        DrawBody(sphere1, RED);
        DrawBody(sphere2, BLUE);
        DrawBody(hull, ORANGE);

        EndMode3D();

        DrawText("SPACE: pause/resume   |   mouse drag: orbit camera", 10, 10, 18, DARKGRAY);
        if (paused) DrawText("PAUSED", 10, 40, 24, RED);

        DrawText(TextFormat("sphere1.y = %.3f", sphere1.position.y), 10, screenHeight - 90, 18, DARKGRAY);
        DrawText(TextFormat("sphere2.y = %.3f", sphere2.position.y), 10, screenHeight - 68, 18, DARKGRAY);
        DrawText(TextFormat("hull.y    = %.3f", hull.position.y),    10, screenHeight - 46, 18, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}