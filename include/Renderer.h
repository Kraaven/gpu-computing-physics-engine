#pragma once
#include "raylib.h"
#include "PhysicsWorld.h"
#include "InputManager.h"

class Renderer {
public:
    void drawWorld(const PhysicsWorld& world, const InputManager& input) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // UI info
        DrawText("Physics Sandbox", 20, 20, 24, DARKGRAY);
        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 50, 18, GRAY);
        DrawText(TextFormat("Bodies: %d", (int)world.bodies.size()), 20, 70, 18, GRAY);
        DrawText("[SPACE] Toggle Mode", 20, 100, 18, GRAY);

        if (input.mode == InputMode::Object)
            DrawText("Mode: OBJECT", 20, 130, 20, BLUE);
        else
            DrawText("Mode: FORCE", 20, 130, 20, RED);

        // Draw all bodies
        for (const auto& b : world.bodies) {
            if (b.collider.type == ShapeType::Circle) {
                DrawCircleV({b.position.x, b.position.y}, b.collider.radius, b.color);
            } else if (b.collider.type == ShapeType::AABB) {
                DrawRectangleV(
                    {b.position.x - b.collider.halfExtents.x, b.position.y - b.collider.halfExtents.y},
                    {b.collider.halfExtents.x * 2, b.collider.halfExtents.y * 2},
                    b.color
                );
            }
        }

        // Draw Force Mode influence radius
        if (input.mode == InputMode::Force) {
            Vector2 mouse = GetMousePosition();
            DrawCircleLines(mouse.x, mouse.y, input.influenceRadius, Fade(RED, 0.5f));
        }

        EndDrawing();
    }
};
