# Mini Physics Engine (C++)

A small, dependency-free 3D rigid-body physics engine written in modern C++17.
Built to understand what's actually happening under the hood of the physics
libraries (Rapier) I use in my browser-based vehicle combat simulator
(Three.js + Rapier), and to practice core object-oriented systems design in C++.

## Features

- Custom `Vec3` math library — no external dependencies
- Semi-implicit (symplectic) Euler integration
- Narrow-phase collision detection: Sphere–Sphere, Box–Box (AABB), Sphere–Box
- **Compound collider support** — a body can be built from several sphere
  sub-shapes at local offsets instead of relying on a single mesh or convex
  hull collider, mirroring the compound-collider approach used in my
  vehicle simulator project
- Impulse-based collision resolution with per-body restitution (bounciness)
- Positional correction (Baumgarte-style slop correction) to stop bodies
  slowly sinking into each other from discrete-timestep error
- Simple O(n²) broad phase, with the tradeoff documented below

## Architecture

```
include/
  Vec3.hpp                -- minimal 3D vector math
  Shapes.hpp               -- Sphere, Box, and Compound shape definitions
  RigidBody.hpp             -- mass, velocity, force accumulation, integration
  CollisionDetection.hpp     -- narrow-phase tests + shape-type dispatcher
  PhysicsWorld.hpp            -- simulation loop: gravity, integration, resolution
src/
  *.cpp                      -- corresponding implementations
  main.cpp                    -- demo scene
```

Each `RigidBody` owns a `ShapeType` tag plus the shape data for all shape
kinds; `CollisionDetection::detect()` reads the tags on both bodies and
dispatches to the correct narrow-phase test, so `PhysicsWorld` never needs
to know which shape combination it's resolving.

## Build & Run

Requires CMake 3.10+ and a C++17 compiler.

```bash
mkdir build && cd build
cmake ..
cmake --build .
./mini_physics_engine
```

Or compile directly with g++:

```bash
g++ -std=c++17 -Iinclude src/*.cpp -o mini_physics_engine
./mini_physics_engine
```

## Demo Scene

`main.cpp` drops two spheres onto a static ground plane and a compound
two-sphere "hull" body (standing in for a simplified vehicle chassis), then
prints height over time so you can see gravity, impulse resolution, and the
compound collider all settling out correctly in the console output:

```
t=0.000s  sphere1.y=4.997  sphere2.y=7.997  hull.y=5.997
t=1.000s  sphere1.y=0.779  sphere2.y=2.847  hull.y=0.847
t=2.000s  sphere1.y=0.584  sphere2.y=0.508  hull.y=0.489
t=3.500s  sphere1.y=0.490  sphere2.y=0.490  hull.y=0.489
```

All three bodies come to rest at their expected resting height (radius 0.5
above the ground plane), confirming gravity, integration, and both simple
and compound collision resolution are working correctly.

## Design Notes

- **Why compound colliders?** Convex hull or mesh colliders are expensive to
  generate and can behave unpredictably on irregular shapes like a vehicle
  hull. Building a shape out of a handful of primitives (spheres here) is
  far cheaper to collide against and easier to reason about — the same
  tradeoff made in my Three.js/Rapier vehicle simulator.
- **Why semi-implicit Euler instead of full RK4?** It's numerically stable
  at fixed timesteps and is what most real-time game physics engines
  actually use in practice — extra accuracy rarely matters for
  gameplay-feel physics, and the performance cost of RK4 isn't worth it.

## Known Limitations / Future Work

- Boxes are axis-aligned only — no rotation/orientation yet
- Broad phase is brute-force O(n²) — fine for small scenes; a spatial hash
  or BVH would be needed for larger ones
- No angular velocity or torque — bodies don't spin on impact
- No continuous collision detection — fast-moving small bodies can tunnel
  through thin colliders
- Compound-vs-compound collision is not yet implemented
