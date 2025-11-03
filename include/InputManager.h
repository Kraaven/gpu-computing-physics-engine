#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"
#include <algorithm>

enum class InputMode { Object, Force };
// SpawnShape removed -- circles only

struct UIRect {
    float x, y, w, h;
    bool contains(const Vector2 &p) const {
        return p.x >= x && p.x <= x + w && p.y >= y && p.y <= y + h;
    }
};

class InputManager {
public:
    InputMode mode = InputMode::Object;

    float spawnSize = 25.0f; // default small for liquid
    float forceStrength = 200000.0f;
    float influenceRadius = 200.0f;

    bool showContacts = false;
    bool gravityEnabled = true;

    // Scene toggle
    bool useLiquidScene = false;

    // UI rectangles
    UIRect panelRect;
    UIRect modeObjectBtn;
    UIRect modeForceBtn;
    UIRect sliderRect;
    UIRect sliderKnob;
    UIRect clearBtn;
    UIRect gravityBtn;
    UIRect contactsBtn;
    UIRect sceneToggleBtn;
    UIRect infoArea;

    // layout / interaction state
    float panelWidth = 220.0f;
    float margin = 12.0f;
    float rowH = 34.0f;
    bool draggingSlider = false;
    bool draggingPanel = false;
    Vector2 dragOffset = {0,0};

    InputManager() {}

    void recomputeLayout() {
        int sw = GetScreenWidth();
        float panelH = 460.0f;
        float x0 = sw - panelWidth - margin;
        float y0 = margin;

        panelRect = {x0, y0, panelWidth, panelH};

        float curY = y0 + margin;
        modeObjectBtn = {x0 + margin, curY, panelWidth - margin*2, rowH};
        curY += rowH + 8;
        modeForceBtn = {x0 + margin, curY, panelWidth - margin*2, rowH};
        curY += rowH + 12;

        sliderRect = {x0 + margin, curY + 6, panelWidth - margin*2 - 40, 16};
        sliderKnob = {sliderRect.x + sliderRect.w * 0.5f - 7, sliderRect.y - 8, 14, sliderRect.h + 16};
        curY += 16 + 20;

        clearBtn = {x0 + margin, curY, (panelWidth - margin*3)*0.5f, rowH};
        gravityBtn = {x0 + margin + clearBtn.w + margin, curY, clearBtn.w, rowH};
        curY += rowH + 12;

        contactsBtn = {x0 + margin, curY, panelWidth - margin*2, rowH};
        curY += rowH + 12;

        sceneToggleBtn = {x0 + margin, curY, panelWidth - margin*2, rowH};
        curY += rowH + 12;

        infoArea = {x0 + margin, y0 + panelH - 140.0f, panelWidth - margin*2, 120.0f};

        // update slider knob position from spawnSize
        const float minSize = 1.0f;
        const float maxSize = 24.0f;
        float t = (spawnSize - minSize) / (maxSize - minSize);
        if (t < 0) {
            t = 0;
        }
        if (t > 1) {
            t = 1;
        }
        float knobX = sliderRect.x + t * (sliderRect.w - sliderKnob.w);
        sliderKnob.x = knobX;
    }

    bool mouseOverUI() const {
        Vector2 m = GetMousePosition();
        return CheckCollisionPointRec(m, { panelRect.x, panelRect.y, panelRect.w, panelRect.h });
    }

    void handleInput(PhysicsWorld &world) {
        recomputeLayout();

        Vector2 mouse = GetMousePosition();
        bool pressed = IsMouseButtonPressed(MOUSE_LEFT_BUTTON);
        bool down = IsMouseButtonDown(MOUSE_LEFT_BUTTON);
        bool released = IsMouseButtonReleased(MOUSE_LEFT_BUTTON);

        if (pressed && CheckCollisionPointRec(mouse, {panelRect.x, panelRect.y, panelRect.w, 30})) {
            draggingPanel = true;
            dragOffset = { mouse.x - panelRect.x, mouse.y - panelRect.y };
            return;
        }
        if (released && draggingPanel) draggingPanel = false;
        if (draggingPanel) {
            Vector2 m = GetMousePosition();
            float newX = m.x - dragOffset.x;
            float newY = m.y - dragOffset.y;
            if (newX < 0) newX = 0;
            if (newY < 0) newY = 0;
            if (newX + panelRect.w > GetScreenWidth()) newX = GetScreenWidth() - panelRect.w;
            if (newY + panelRect.h > GetScreenHeight()) newY = GetScreenHeight() - panelRect.h;
            panelRect.x = newX; panelRect.y = newY;
            recomputeLayout();
            return;
        }

        if (pressed || down || released) {
            if (processUI(mouse, pressed, down, released, world)) return;
        }

        if (mouseOverUI()) return;

        if (mode == InputMode::Object) {
            handleObjectMode(world, Vec2{mouse.x, mouse.y}, pressed);
        } else {
            handleForceMode(world, Vec2{mouse.x, mouse.y});
        }
    }

private:
    bool processUI(const Vector2 &mouse, bool pressed, bool down, bool released, PhysicsWorld &world) {
        if (pressed) {
            if (CheckCollisionPointRec(mouse, {modeObjectBtn.x, modeObjectBtn.y, modeObjectBtn.w, modeObjectBtn.h})) { mode = InputMode::Object; return true; }
            if (CheckCollisionPointRec(mouse, {modeForceBtn.x, modeForceBtn.y, modeForceBtn.w, modeForceBtn.h})) { mode = InputMode::Force; return true; }
            if (CheckCollisionPointRec(mouse, {clearBtn.x, clearBtn.y, clearBtn.w, clearBtn.h})) {
                world.bodies.erase(std::remove_if(world.bodies.begin(), world.bodies.end(),
                    [](const Body &b){ return !b.isStatic; }), world.bodies.end());
                return true;
            }
            if (CheckCollisionPointRec(mouse, {gravityBtn.x, gravityBtn.y, gravityBtn.w, gravityBtn.h})) {
                gravityEnabled = !gravityEnabled;
                world.gravity = gravityEnabled ? Vec2{0.0f, 300.0f} : Vec2::zero();
                return true;
            }
            if (CheckCollisionPointRec(mouse, {contactsBtn.x, contactsBtn.y, contactsBtn.w, contactsBtn.h})) {
                showContacts = !showContacts; return true;
            }
            if (CheckCollisionPointRec(mouse, {sceneToggleBtn.x, sceneToggleBtn.y, sceneToggleBtn.w, sceneToggleBtn.h})) {
                useLiquidScene = !useLiquidScene; return true;
            }
            if (CheckCollisionPointRec(mouse, {sliderRect.x - 8, sliderRect.y - 8, sliderRect.w + 16, sliderRect.h + 16})) {
                draggingSlider = true; updateSliderFromMouse(mouse); return true;
            }
        }
        if (down && draggingSlider) { updateSliderFromMouse(mouse); return true; }
        if (released && draggingSlider) { draggingSlider = false; updateSliderFromMouse(mouse); return true; }
        return false;
    }

    void updateSliderFromMouse(const Vector2 &mouse) {
        float localX = mouse.x - sliderRect.x;
        float t = localX / (sliderRect.w - sliderKnob.w);
        if (t < 0) {
            t = 0;
        }
        if (t > 1) {
            t = 1;
        }
        const float minSize = 1.0f, maxSize = 24.0f;
        spawnSize = minSize + t * (maxSize - minSize);
        sliderKnob.x = sliderRect.x + t * (sliderRect.w - sliderKnob.w);
    }

    void handleObjectMode(PhysicsWorld &world, const Vec2 &mousePos, bool mousePressedL) {
        if (mousePressedL) {
            Body b({mousePos.x, mousePos.y}, 1.0f);
            b.collider.radius = spawnSize;
            b.color = ColorFromHSV(GetRandomValue(0,359), 0.8f, 0.9f);
            world.addBody(b);
            return;
        }

        // right-click delete
        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            Vec2 m(mousePos.x, mousePos.y);
            for (int i = (int)world.bodies.size() - 1; i >= 0; --i) {
                Body &b = world.bodies[i];
                if (b.isStatic) continue;
                Vec2 d = m - b.position;
                if (d.length2() <= b.collider.radius * b.collider.radius) {
                    world.bodies.erase(world.bodies.begin() + i);
                    break;
                }
            }
        }
    }

    void handleForceMode(PhysicsWorld &world, const Vec2 &mousePos) {
        bool attract = IsMouseButtonDown(MOUSE_RIGHT_BUTTON);
        bool repel = IsMouseButtonDown(MOUSE_LEFT_BUTTON);
        if (!attract && !repel) return;

        for (auto &b : world.bodies) {
            if (b.isStatic) continue;
            Vec2 dir = b.position - mousePos;
            float dist2 = dir.length2();
            if (dist2 > influenceRadius * influenceRadius) continue;
            float dist = std::sqrt(std::max(dist2, 1.0f));
            Vec2 n = dir / dist;
            if (attract) n = n * -1.0f;
            float strength = forceStrength / (dist + 100.0f);
            b.velocity += n * (strength * world.dt);

            float maxV = world.maxVelocity;
            float vsq = b.velocity.length2();
            if (vsq > maxV * maxV) {
                float vlen = std::sqrt(vsq);
                b.velocity = b.velocity * (maxV / vlen);
            }
        }
    }
};