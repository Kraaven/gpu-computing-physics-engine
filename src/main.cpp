#include <iostream>
#include "raylib.h"
#include "Vec2.h"
#include "Body.h"

int main() {
    InitWindow(800, 600, "Physics Engine Step 2 - Vec2 + Body Test");
    SetTargetFPS(60);

    Body circle({400, 300}, 2.0f);
    circle.collider.radius = 25.0f;
    circle.color = RED;

    std::cout << "Created body at position " << circle.position
              << " with mass " << circle.mass
              << " and inverse mass " << circle.invMass << std::endl;

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("Step 2: Vec2 + Body structure test", 20, 20, 20, DARKGRAY);
        DrawCircleV({circle.position.x, circle.position.y}, circle.collider.radius, circle.color);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
