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

    InitWindow(screenW, screenH, "Physics Engine - Circle-only + Liquid");
    SetTargetFPS(120);

    PhysicsWorld world(32.0f); // finer hash for particles
    InputManager input;
    Renderer renderer;

    scene.loadDefault(world);
    input.recomputeLayout();

    // Fixed timestep parameters
    const double fixedDt = world.dt; // use world.dt consistently
    double accumulator = 0.0;
    double lastTime = GetTime();

    // For liquid scene: remember spawn params
    bool liquidLoaded = false;

    while (!WindowShouldClose()) {
        double now = GetTime();
        double frameTime = now - lastTime;
        if (frameTime > 0.25) frameTime = 0.25;
        lastTime = now;
        accumulator += frameTime;

        input.recomputeLayout();
        input.handleInput(world);

        // handle scene toggle
        if (input.useLiquidScene && !liquidLoaded) {
            scene.loadLiquid(world, input.spawnSize);
            liquidLoaded = true;
        } else if (!input.useLiquidScene && liquidLoaded) {
            scene.loadDefault(world);
            liquidLoaded = false;
        }

        // Step physics one or more times at fixedDt
        while (accumulator >= fixedDt) {
            for (auto &b : world.bodies) b.prevPosition = b.position;

            world.step();
            accumulator -= fixedDt;
        }

        // compute alpha for interpolation (0..1)
        float alpha = (float)(accumulator / fixedDt);
        renderer.drawWorld(world, input, alpha);
    }

    CloseWindow();
    return 0;
}
