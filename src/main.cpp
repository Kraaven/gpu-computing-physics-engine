#include "raylib.h"
#include "Vec2.h"
#include "Body.h"
#include "PhysicsWorld.h"
#include <iostream>

int main() {
    const int screenW = 800;
    const int screenH = 600;

    InitWindow(screenW, screenH, "Physics Engine Step 5 - Collisions (circle/circle, circle/AABB)");
    SetTargetFPS(120);

    PhysicsWorld world;

    // Create dynamic circles
    Body ball1({200, 100}, 1.0f);
    ball1.collider.type = ShapeType::Circle;
    ball1.collider.radius = 20.0f;
    ball1.color = RED;

    Body ball2({400, 200}, 2.0f);
    ball2.collider.type = ShapeType::Circle;
    ball2.collider.radius = 30.0f;
    ball2.color = BLUE;

    Body ball3({600, 50}, 0.5f);
    ball3.collider.type = ShapeType::Circle;
    ball3.collider.radius = 15.0f;
    ball3.color = GREEN;

    world.addBody(ball1);
    world.addBody(ball2);
    world.addBody(ball3);

    // Create a static ground as AABB centered near bottom
    Body ground({screenW * 0.5f, screenH - 10.0f}, 0.0f);
    ground.isStatic = true;
    ground.collider.type = ShapeType::AABB;
    ground.collider.halfExtents = { (float)screenW * 0.5f, 10.0f }; // full-width ground, 20 px tall
    ground.color = GRAY;
    world.addBody(ground);

    while (!WindowShouldClose()) {
        world.step();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Step 5: Collisions - circle/circle and circle/AABB (static ground)", 20, 20, 18, DARKGRAY);

        // Draw each body
        for (const auto& b : world.bodies) {
            if (b.collider.type == ShapeType::Circle) {
                DrawCircleV({b.position.x, b.position.y}, b.collider.radius, b.color);
            } else {
                // Draw AABB as rectangle (centered)
                DrawRectangleV({b.position.x - b.collider.halfExtents.x, b.position.y - b.collider.halfExtents.y},
                               {b.collider.halfExtents.x * 2.0f, b.collider.halfExtents.y * 2.0f}, b.color);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
