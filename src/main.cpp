#include <iostream>
#include <vector>
#include <omp.h>
#include "raylib.h"

// Quick test: small vector sum using OpenMP reduction
long long test_openmp_sum(int N = 1000000) {
    std::vector<int> arr(N, 1);
    long long sum = 0;
    #pragma omp parallel for reduction(+:sum)
    for (int i = 0; i < N; ++i) sum += arr[i];
    return sum;
}

int main() {
    // Print OpenMP info
    #ifdef _OPENMP
    std::cout << "OpenMP is enabled. Max threads: " << omp_get_max_threads() << std::endl;
    #else
    std::cout << "OpenMP NOT enabled." << std::endl;
    #endif

    long long s = test_openmp_sum(1000000);
    std::cout << "OpenMP test sum: " << s << std::endl;

    // Initialize RayLib window
    const int screenW = 1024;
    const int screenH = 720;
    InitWindow(screenW, screenH, "Physics Engine Starter (C++ / OpenMP / RayLib)");
    SetTargetFPS(60);

    // Simple loop (we won't do physics here yet)
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("Physics Engine Starter (C++ / OpenMP / RayLib)", 20, 20, 20, DARKGRAY);
        DrawText(TextFormat("OpenMP threads: %d", omp_get_max_threads()), 20, 50, 18, LIGHTGRAY);
        DrawText("Press ESC or close window to quit", 20, 80, 18, LIGHTGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
