#include "PhysicsWorld.hpp"
#include "CollisionDetection.hpp"
#include <algorithm>

void PhysicsWorld::addBody(RigidBody* body) {
    bodies.push_back(body);
}

void PhysicsWorld::applyGravity() {
    for (auto* body : bodies) {
        if (!body->isStatic()) {
            body->applyForce(gravity * body->mass);
        }
    }
}

void PhysicsWorld::integrateBodies(float dt) {
    for (auto* body : bodies) {
        body->integrate(dt);
    }
}

void PhysicsWorld::resolveCollisions() {
    // Brute-force O(n^2) broad phase - fine for small demo scenes.
    // A production engine would use a spatial hash or BVH here instead.
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            RigidBody& a = *bodies[i];
            RigidBody& b = *bodies[j];

            if (a.isStatic() && b.isStatic()) continue;

            CollisionInfo info = Collision::detect(a, b);
            if (!info.colliding) continue;

            // --- Impulse resolution along the collision normal ---
            Vec3 relativeVelocity = b.velocity - a.velocity;
            float velAlongNormal = relativeVelocity.dot(info.normal);

            // Already separating - nothing to resolve.
            if (velAlongNormal > 0) continue;

            float e = std::min(a.restitution, b.restitution);
            float invMassSum = a.invMass + b.invMass;
            if (invMassSum <= 0.0f) continue;

            float jImpulse = -(1.0f + e) * velAlongNormal / invMassSum;
            Vec3 impulse = info.normal * jImpulse;

            a.applyImpulse(impulse * -1.0f);
            b.applyImpulse(impulse);

            // --- Positional correction (Baumgarte-style slop correction) ---
            // Prevents bodies from slowly sinking into each other due to
            // discrete-timestep integration error.
            const float percent = 0.8f;
            const float slop = 0.01f;
            float correctionMag = std::max(info.penetration - slop, 0.0f) / invMassSum * percent;
            Vec3 correction = info.normal * correctionMag;
            a.position -= correction * a.invMass;
            b.position += correction * b.invMass;
        }
    }
}

void PhysicsWorld::step(float dt) {
    applyGravity();
    integrateBodies(dt);
    resolveCollisions();
}
