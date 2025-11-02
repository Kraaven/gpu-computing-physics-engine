#pragma once
#include "raylib.h"
#include "PhysicsWorld.h"
#include "InputManager.h"
#include <string>

class Renderer {
public:
    // alpha: interpolation factor [0..1] between prevPosition and position
    void drawWorld(const PhysicsWorld& world, const InputManager& input, float alpha) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // UI info
        DrawText("Physics Sandbox", 20, 20, 24, DARKGRAY);
        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 50, 18, GRAY);
        DrawText(TextFormat("Bodies: %d", (int)world.bodies.size()), 20, 70, 18, GRAY);
        DrawText("[SPACE] Toggle Mode  [D] Toggle Debug  [G] Toggle Gravity  [C] Clear Dynamics", 20, 100, 16, GRAY);

        if (input.mode == InputMode::Object)
            DrawText("Mode: OBJECT", 20, 130, 20, BLUE);
        else
            DrawText("Mode: FORCE", 20, 130, 20, RED);

        // Draw all bodies (interpolated)
        for (const auto& b : world.bodies) {
            // interpolated position
            Vec2 renderPos = b.prevPosition * (1.0f - alpha) + b.position * alpha;

            if (b.collider.type == ShapeType::Circle) {
                DrawCircleV({renderPos.x, renderPos.y}, b.collider.radius, b.color);
            } else if (b.collider.type == ShapeType::AABB) {
                DrawRectangleV(
                    {renderPos.x - b.collider.halfExtents.x, renderPos.y - b.collider.halfExtents.y},
                    {b.collider.halfExtents.x * 2, b.collider.halfExtents.y * 2},
                    b.color
                );
            }

            // velocity arrow (small)
            if (!b.isStatic) {
                Vector2 from = {renderPos.x, renderPos.y};
                Vector2 to = {renderPos.x + b.velocity.x * 0.05f, renderPos.y + b.velocity.y * 0.05f};
                DrawLineEx(from, to, 2.0f, GRAY);
            }
        }

        // Draw Force Mode influence radius
        if (input.mode == InputMode::Force) {
            Vector2 mouse = GetMousePosition();
            DrawCircleLines(mouse.x, mouse.y, input.influenceRadius, Fade(RED, 0.5f));
        }

        // Debug: show contacts? (we don't have direct access to contacts here; user toggles for later)
        if (input.showContacts) {
            DrawText("DEBUG: Contacts ON (see console)", 20, 160, 18, MAROON);
        }

        EndDrawing();
    }
};
