#include "raylib.h"
#include "Vec2.h"
#include "Body.h"
#include "PhysicsWorld.h"
#include "InputManager.h"
#include "Renderer.h"
#include "Scene.h"

int main() {
    Scene scene;
    const int screenW = scene.width;
    const int screenH = scene.height;

    InitWindow(screenW, screenH, "Physics Engine - Stable Simulation");
    SetTargetFPS(120);

    PhysicsWorld world;
    InputManager input;
    Renderer renderer;

    scene.loadDefault(world);
    input.recomputeLayout();

    // Fixed timestep parameters
    const double fixedDt = world.dt; // use world.dt consistently
    double accumulator = 0.0;
    double lastTime = GetTime();

    // Main loop
    while (!WindowShouldClose()) {
        double now = GetTime();
        double frameTime = now - lastTime;
        if (frameTime > 0.25) frameTime = 0.25; // avoid spiral of death
        lastTime = now;
        accumulator += frameTime;

        input.recomputeLayout();
        input.handleInput(world);

        // Step physics one or more times at fixedDt
        while (accumulator >= fixedDt) {
            // before stepping, ensure prevPosition is current position (for interpolation)
            for (auto &b : world.bodies) {
                b.prevPosition = b.position;
            }

            world.step(); // world.dt used internally
            accumulator -= fixedDt;
        }

        // compute alpha for interpolation (0..1)
        float alpha = (float)(accumulator / fixedDt);
        // render using interpolated positions
        renderer.drawWorld(world, input, alpha);
    }

    CloseWindow();
    return 0;
}
