#include "includes/astar_class.h"
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

const std::string OUTPUT_DIR   = "../output";
const std::string OUTPUT_FILE  = "astar_algoritm_in_cpp.avi";
const std::string OUTPUT_IMAGE = "astar_cpp_path.png";

// ---------------------------------------------------------------------------
// Input helpers
// ---------------------------------------------------------------------------

// Read a value of type T from stdin, re-prompting on bad input or failed validation.
// valid_fn: return true if the value is acceptable.
// hint: shown when validation fails.
template <typename T>
T get_input(const std::string& prompt,
            std::function<bool(T)> valid_fn = nullptr,
            const std::string& hint = "") {
    while (true) {
        std::cout << prompt;
        std::string line;
        std::getline(std::cin, line);
        std::istringstream ss(line);
        T val;
        if (!(ss >> val)) {
            std::cout << "  Please enter a valid value. " << hint << "\n";
            continue;
        }
        if (valid_fn && !valid_fn(val)) {
            std::cout << "  Invalid value. " << hint << "\n";
            continue;
        }
        return val;
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    std::cout << std::string(50, '=') << "\n";
    std::cout << "          A* Path Planner\n";
    std::cout << std::string(50, '=') << "\n";
    std::cout << "Map size: 300 cols x 200 rows\n\n";

    // --- Robot Parameters ---
    std::cout << "--- Robot Parameters ---\n";
    int radius = get_input<int>(
        "  Robot radius       (int, >= 0) : ",
        [](int v){ return v >= 0; });

    int clearance = get_input<int>(
        "  Obstacle clearance (int, >= 0) : ",
        [](int v){ return v >= 0; });

    double row_min = 1.0 + radius + clearance;
    double row_max = 200.0 - radius - clearance;
    double col_min = 1.0 + radius + clearance;
    double col_max = 300.0 - radius - clearance;

    if (row_min > row_max || col_min > col_max) {
        std::cout << "\n  Error: radius + clearance (" << radius + clearance
                  << ") is too large for the map.\n";
        return 1;
    }

    std::cout << "\n  Valid position range: row [" << row_min << " - " << row_max
              << "], col [" << col_min << " - " << col_max << "]\n";

    // Temporary instance used only for IsValid / IsObstacle checks
    Astar checker({row_min, col_min, 0}, {row_max, col_max}, clearance, radius);
    auto is_free = [&](double row, double col) {
        return checker.IsValid(row, col) && !checker.IsObstacle(row, col);
    };

    // --- Start Position ---
    std::cout << "\n--- Start Position ---\n";
    double start_row, start_col;
    while (true) {
        start_row = get_input<double>(
            "  Start row   (float, " + std::to_string(row_min) + " - " + std::to_string(row_max) + ") : ",
            [&](double v){ return v >= row_min && v <= row_max; },
            "Must be between " + std::to_string(row_min) + " and " + std::to_string(row_max) + ".");

        start_col = get_input<double>(
            "  Start col   (float, " + std::to_string(col_min) + " - " + std::to_string(col_max) + ") : ",
            [&](double v){ return v >= col_min && v <= col_max; },
            "Must be between " + std::to_string(col_min) + " and " + std::to_string(col_max) + ".");

        if (!is_free(start_row, start_col)) {
            std::cout << "  Start position is inside an obstacle. Please choose another.\n";
            continue;
        }
        break;
    }
    int start_angle = get_input<int>(
        "  Start angle (int, 0/30/60/.../330)    : ",
        [](int v){ return v >= 0 && v < 360 && v % 30 == 0; },
        "Must be a multiple of 30 between 0 and 330.");

    // --- Goal Position ---
    std::cout << "\n--- Goal Position ---\n";
    double goal_row, goal_col;
    while (true) {
        goal_row = get_input<double>(
            "  Goal row    (float, " + std::to_string(row_min) + " - " + std::to_string(row_max) + ") : ",
            [&](double v){ return v >= row_min && v <= row_max; },
            "Must be between " + std::to_string(row_min) + " and " + std::to_string(row_max) + ".");

        goal_col = get_input<double>(
            "  Goal col    (float, " + std::to_string(col_min) + " - " + std::to_string(col_max) + ") : ",
            [&](double v){ return v >= col_min && v <= col_max; },
            "Must be between " + std::to_string(col_min) + " and " + std::to_string(col_max) + ".");

        if (!is_free(goal_row, goal_col)) {
            std::cout << "  Goal position is inside an obstacle. Please choose another.\n";
            continue;
        }
        break;
    }

    Node  start = {start_row, start_col, start_angle};
    auto  goal  = std::make_pair(goal_row, goal_col);

    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "  Start : row=" << start_row << ", col=" << start_col
              << ", angle=" << start_angle << " deg\n";
    std::cout << "  Goal  : row=" << goal_row  << ", col=" << goal_col  << "\n";
    std::cout << "  Robot : radius=" << radius  << ", clearance=" << clearance << "\n";
    std::cout << std::string(50, '=') << "\n";
    std::cout << "Running A* ...\n";

    Astar astar(start, goal, clearance, radius);
    auto [explored, backtrack, cost] = astar.Run();

    if (backtrack.empty()) {
        std::cout << "No path found.\n";
        return 1;
    }

    std::cout << "\nPath cost     : " << cost             << "\n";
    std::cout << "Path length   : " << backtrack.size() << " nodes\n";
    std::cout << "Nodes explored: " << explored.size()  << "\n";

    fs::create_directories(OUTPUT_DIR);
    std::string video_path = OUTPUT_DIR + "/" + OUTPUT_FILE;
    std::string image_path = OUTPUT_DIR + "/" + OUTPUT_IMAGE;

    std::cout << "\nGenerating video -> " << video_path << "\n";
    astar.Animate(explored, backtrack, video_path, image_path);
    std::cout << "Final image saved -> " << image_path << "\n";
    std::cout << "Done.\n";

    return 0;
}
