#include <iostream>
#include <iomanip>
#include "PhysicsWorld.hpp"

int main() {
    PhysicsWorld world;

    // Static ground, modeled as a wide flat box (invMass = 0 -> immovable).
    RigidBody ground(ShapeType::Box, 0.0f);
    ground.position = Vec3(0.0f, -1.0f, 0.0f);
    ground.box.halfExtents = Vec3(50.0f, 1.0f, 50.0f);
    world.addBody(&ground);

    // Two falling spheres, offset so they collide with each other mid-air.
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

    // A compound body: two spheres offset from a shared origin, standing in
    // for a simplified vehicle hull (echoes the compound-collider approach
    // used in the tank/vehicle simulator project instead of a mesh collider).
    RigidBody hull(ShapeType::Compound, 4.0f);
    hull.position = Vec3(3.0f, 6.0f, 0.0f);
    hull.restitution = 0.2f;
    hull.compound.children = {
        { Vec3(-0.6f, 0.0f, 0.0f), 0.5f },
        { Vec3( 0.6f, 0.0f, 0.0f), 0.5f }
    };
    world.addBody(&hull);

    const float dt = 1.0f / 60.0f;
    const int steps = 240; // 4 simulated seconds

    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < steps; ++i) {
        world.step(dt);
        if (i % 30 == 0) {
            std::cout << "t=" << (i * dt) << "s"
                      << "  sphere1.y=" << sphere1.position.y
                      << "  sphere2.y=" << sphere2.position.y
                      << "  hull.y="    << hull.position.y
                      << '\n';
        }
    }

    return 0;
}
