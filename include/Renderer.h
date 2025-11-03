#pragma once
#include "raylib.h"
#include "PhysicsWorld.h"
#include "InputManager.h"
#include <string>
#include <cmath>

class Renderer {
public:
    void drawWorld(const PhysicsWorld& world, const InputManager& input, float alpha) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("Physics Sandbox", 20, 20, 24, DARKGRAY);
        DrawText(TextFormat("FPS: %d", GetFPS()), 20, 50, 18, GRAY);
        DrawText(TextFormat("Bodies: %d", (int)world.bodies.size()), 20, 70, 18, GRAY);

        // Draw bodies with interpolation
        for (const auto &b : world.bodies) {
            Vec2 renderPos = b.prevPosition * (1.0f - alpha) + b.position * alpha;
            DrawCircleV({renderPos.x, renderPos.y}, b.collider.radius, b.color);
            if (!b.isStatic) {
                Vector2 from = {renderPos.x, renderPos.y};
                Vector2 to = {renderPos.x + b.velocity.x * 0.05f, renderPos.y + b.velocity.y * 0.05f};
                DrawLineEx(from, to, 2.0f, GRAY);
            }
        }

        // Force influence
        if (input.mode == InputMode::Force) {
            Vector2 mouse = GetMousePosition();
            DrawCircleLines(mouse.x, mouse.y, input.influenceRadius, Fade(RED, 0.5f));
        }

        // UI panel
        drawUIPanel(input);

        if (input.showContacts) {
            DrawText("Contacts: (debug ON)", 20, 100, 16, MAROON);
        }

        EndDrawing();
    }

private:
    void drawUIPanel(const InputManager& input) {
        Rectangle rect = { input.panelRect.x, input.panelRect.y, input.panelRect.w, input.panelRect.h };
        DrawRectangleRounded(rect, 0.06f, 4, Fade(LIGHTGRAY, 0.95f));
        DrawRectangleLinesEx(rect, 2.0f, GRAY);

        float tx = rect.x + input.margin;
        float ty = rect.y + input.margin;

        DrawText("Controls", tx, ty - 6, 20, DARKGRAY);

        drawButton(input.modeObjectBtn.x, input.modeObjectBtn.y, input.modeObjectBtn.w, input.modeObjectBtn.h,
                   "Mode: OBJECT", input.mode == InputMode::Object);
        drawButton(input.modeForceBtn.x, input.modeForceBtn.y, input.modeForceBtn.w, input.modeForceBtn.h,
                   "Mode: FORCE", input.mode == InputMode::Force);

        DrawText("Size", input.sliderRect.x, input.sliderRect.y - 18, 14, DARKGRAY);
        Rectangle sliderBar = { input.sliderRect.x, input.sliderRect.y, input.sliderRect.w, input.sliderRect.h };
        DrawRectangleLinesEx(sliderBar, 2.0f, GRAY);

        const float minS = 1.0f, maxS = 100.0f;

        float t = (input.spawnSize - minS) / (maxS - minS);
        if (t < 0) {
            t = 0;
        }
        if (t > 1) {
            t = 1;
        }
        Rectangle filledBar = { sliderBar.x, sliderBar.y, sliderBar.width * t, sliderBar.height };
        DrawRectangleRounded(filledBar, 0.02f, 4, Fade(SKYBLUE, 0.8f));

        DrawRectangleRounded({ input.sliderKnob.x, input.sliderKnob.y, input.sliderKnob.w, input.sliderKnob.h }, 0.2f, 4, DARKGRAY);
        DrawText(TextFormat("%.1f", input.spawnSize), input.sliderRect.x + input.sliderRect.w + 8, input.sliderRect.y, 14, DARKGRAY);

        drawSmallButton(input.clearBtn.x, input.clearBtn.y, input.clearBtn.w, input.clearBtn.h, "Clear");
        drawSmallToggle(input.gravityBtn.x, input.gravityBtn.y, input.gravityBtn.w, input.gravityBtn.h, "Gravity", input.gravityEnabled);
        drawButton(input.contactsBtn.x, input.contactsBtn.y, input.contactsBtn.w, input.contactsBtn.h,
                   input.showContacts ? "Contacts: ON" : "Contacts: OFF", input.showContacts);

        drawButton(input.sceneToggleBtn.x, input.sceneToggleBtn.y, input.sceneToggleBtn.w, input.sceneToggleBtn.h,
                   input.useLiquidScene ? "Scene: LIQUID" : "Scene: DEFAULT", input.useLiquidScene);

        DrawText("- Left click to spawn (Object mode)", input.infoArea.x, input.infoArea.y, 12, DARKGRAY);
        DrawText("- Right click to delete", input.infoArea.x, input.infoArea.y + 16, 12, DARKGRAY);
        DrawText("- Toggle scenes to switch liquid/default", input.infoArea.x, input.infoArea.y + 32, 12, DARKGRAY);
    }

    void drawButton(float x, float y, float w, float h, const char* text, bool active=false) const {
        Color bg = active ? Fade(DARKGREEN, 0.9f) : Fade(LIGHTGRAY, 0.95f);
        DrawRectangleRounded({x,y,w,h}, 0.08f, 4, bg);
        DrawRectangleLinesEx({x,y,w,h}, 1.5f, GRAY);
        int tw = MeasureText(text, 14);
        DrawText(text, (int)(x + (w - tw) * 0.5f), (int)(y + (h - 14) * 0.5f), 14, WHITE);
    }

    void drawSmallToggle(float x, float y, float w, float h, const char* text, bool on) const {
        Color bg = on ? Fade(SKYBLUE, 0.9f) : Fade(LIGHTGRAY, 0.95f);
        DrawRectangleRounded({x,y,w,h}, 0.06f, 4, bg);
        DrawRectangleLinesEx({x,y,w,h}, 1.0f, GRAY);
        int tw = MeasureText(text, 12);
        DrawText(text, (int)(x + (w - tw) * 0.5f), (int)(y + (h - 12) * 0.5f), 12, WHITE);
    }

    void drawSmallButton(float x, float y, float w, float h, const char* text) const {
        DrawRectangleRounded({x,y,w,h}, 0.06f, 4, Fade(LIGHTGRAY, 0.95f));
        DrawRectangleLinesEx({x,y,w,h}, 1.0f, GRAY);
        int tw = MeasureText(text, 12);
        DrawText(text, (int)(x + (w - tw) * 0.5f), (int)(y + (h - 12) * 0.5f), 12, DARKGRAY);
    }
};