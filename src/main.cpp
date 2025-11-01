#include "raylib.h"
#include "Vec2.h"
#include "Body.h"
#include "PhysicsWorld.h"
#include <iostream>

int main() {
    InitWindow(800, 600, "Physics Engine Step 3 - PhysicsWorld");
    SetTargetFPS(120);

    PhysicsWorld world;

    // Create a few test bodies
    Body ball1({200, 100}, 1.0f);
    ball1.collider.radius = 20.0f;
    ball1.color = RED;

    Body ball2({400, 200}, 2.0f);
    ball2.collider.radius = 30.0f;
    ball2.color = BLUE;

    Body ball3({600, 50}, 0.5f);
    ball3.collider.radius = 15.0f;
    ball3.color = GREEN;

    world.addBody(ball1);
    world.addBody(ball2);
    world.addBody(ball3);

    while (!WindowShouldClose()) {
        world.step();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Step 3: PhysicsWorld - Gravity + Integration", 20, 20, 20, DARKGRAY);

        // Draw each body
        for (const auto& b : world.bodies) {
            DrawCircleV({b.position.x, b.position.y}, b.collider.radius, b.color);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
