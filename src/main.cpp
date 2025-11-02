#include "raylib.h"
#include "Vec2.h"
#include "Body.h"
#include "PhysicsWorld.h"
#include "InputManager.h"
#include "Renderer.h"

int main() {
    const int screenW = 800;
    const int screenH = 600;

    InitWindow(screenW, screenH, "Physics Engine Step 9 - Renderer + UI");
    SetTargetFPS(120);

    PhysicsWorld world;
    InputManager input;
    Renderer renderer;

    // Ground (static body)
    Body ground({screenW / 2.0f, screenH - 10.0f}, 0.0f);
    ground.isStatic = true;
    ground.collider.type = ShapeType::AABB;
    ground.collider.halfExtents = { (float)screenW / 2.0f, 10.0f };
    ground.color = GRAY;
    world.addBody(ground);

    while (!WindowShouldClose()) {
        input.handleInput(world);
        world.step();
        renderer.drawWorld(world, input);
    }

    CloseWindow();
    return 0;
}
