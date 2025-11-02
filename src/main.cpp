#include "raylib.h"
#include "Vec2.h"
#include "Body.h"
#include "PhysicsWorld.h"
#include "InputManager.h"

int main() {
    const int screenW = 800;
    const int screenH = 600;

    InitWindow(screenW, screenH, "Physics Engine Step 6 - Mouse Interaction (Force/Object)");
    SetTargetFPS(120);

    PhysicsWorld world;
    InputManager input;

    // Create static ground
    Body ground({screenW * 0.5f, screenH - 10.0f}, 0.0f);
    ground.isStatic = true;
    ground.collider.type = ShapeType::AABB;
    ground.collider.halfExtents = { (float)screenW * 0.5f, 10.0f };
    ground.color = GRAY;
    world.addBody(ground);

    while (!WindowShouldClose()) {
        input.handleInput(world);
        world.step();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Mode display
        DrawText("Step 6: Mouse Interaction", 20, 20, 20, DARKGRAY);
        DrawText("[SPACE] Toggle Mode", 20, 45, 18, GRAY);

        if (input.mode == InputMode::Object)
            DrawText("Mode: OBJECT", 20, 70, 20, BLUE);
        else
            DrawText("Mode: FORCE", 20, 70, 20, RED);

        // Draw all bodies
        for (const auto& b : world.bodies) {
            if (b.collider.type == ShapeType::Circle)
                DrawCircleV({b.position.x, b.position.y}, b.collider.radius, b.color);
            else
                DrawRectangleV(
                    {b.position.x - b.collider.halfExtents.x, b.position.y - b.collider.halfExtents.y},
                    {b.collider.halfExtents.x * 2, b.collider.halfExtents.y * 2},
                    b.color
                );
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
