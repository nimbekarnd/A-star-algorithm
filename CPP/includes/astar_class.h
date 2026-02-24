#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <tuple>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <string>
#include <algorithm>
#include <functional>

// ---------------------------------------------------------------------------
// Node: (row, col, angle_degrees)
// ---------------------------------------------------------------------------
using Node = std::tuple<double, double, int>;

// Hash for Node — required to use Node as an unordered_map key
struct NodeHash
{
    std::size_t operator()(const Node &n) const
    {
        std::size_t h1 = std::hash<double>{}(std::get<0>(n));
        std::size_t h2 = std::hash<double>{}(std::get<1>(n));
        std::size_t h3 = std::hash<int>{}(std::get<2>(n));
        return h1 ^ (h2 << 32) ^ (h3 << 16);
    }
};

// Priority queue entry: (f_cost, node) — min-heap on f_cost
using PQEntry = std::pair<double, Node>;

// Sentinel node used to mark the start of the path in pathMap
inline const Node SENTINEL = {-1.0, -1.0, -1};

inline double edgeFn(double x1, double y1,
                     double x2, double y2,
                     double col, double row)
{
    // Signed area / cross product
    return (x2 - x1) * (row - y1) - (y2 - y1) * (col - x1);
}

inline double triOrient(double x1, double y1,
                        double x2, double y2,
                        double x3, double y3)
{
    return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
}

// ---------------------------------------------------------------------------
// Astar class
// ---------------------------------------------------------------------------
class Astar
{
public:
    // Constructor
    // start     : (row, col, angle_degrees) — angle must be a multiple of 30
    // goal      : (row, col)
    // clearance : buffer around obstacles in pixels
    // radius    : robot radius in pixels
    Astar(Node start, std::pair<double, double> goal, int clearance, int radius);

    // Returns true if (currRow, currCol) is inside the traversable map area.
    // Valid region: [1+radius+clearance, numRows-radius-clearance]
    //           x  [1+radius+clearance, numCols-radius-clearance]
    bool IsValid(double currRow, double currCol);

    // Returns true if (row, col) is inside any obstacle (including inflation
    // by radius + clearance).
    // Obstacles: circle, ellipse, two triangles, rhombus, square, rod.
    bool IsObstacle(double row, double col);

    // Action: turn heading +30° (CCW), move one step, return true if valid.
    // Side-effect: sets move_i = sin(new_heading), move_j = cos(new_heading)
    bool ActionMove30Up(double currRow, double currCol, int currAngle);

    // Action: turn heading +60° (CCW), move one step, return true if valid.
    bool ActionMove60Up(double currRow, double currCol, int currAngle);

    // Action: turn heading -30° (CW), move one step, return true if valid.
    bool ActionMove30Down(double currRow, double currCol, int currAngle);

    // Action: turn heading -60° (CW), move one step, return true if valid.
    bool ActionMove60Down(double currRow, double currCol, int currAngle);

    // Action: move straight in the current heading, return true if valid.
    bool ActionMoveStraight(double currRow, double currCol, int currAngle);

    // Returns true if (currRow, currCol) is within 1.5 units of the goal.
    // When true, stores the matching position in goalRow / goalCol.
    bool CheckIfGoal(double currRow, double currCol);

    // Runs A* from start to goal.
    // Returns: (explored_states, backtrack_path, path_cost)
    //   explored_states : every node popped from the priority queue (in order)
    //   backtrack_path  : optimal path from start to goal (empty if no path)
    //   path_cost       : g-cost of the goal node (inf if no path)
    std::tuple<std::vector<Node>, std::vector<Node>, double> Run();

    // Renders the result to a video file and (optionally) a PNG image.
    //   explored    : orange  (255, 150,   0) — every 75 frames
    //   free space  : green   (154, 250,   0) — every 75 frames
    //   path        : blue    (  0,   0, 255) — every frame + imshow
    void Animate(const std::vector<Node> &explored,
                 const std::vector<Node> &backtrack,
                 const std::string &videoPath,
                 const std::string &imagePath = "");

private:
    Node start;
    std::pair<double, double> goal;
    int clearance;
    int radius;
    int numRows;    // 200
    int numCols;    // 300
    double move_i;  // sin(heading) — set as side-effect by action methods
    double move_j;  // cos(heading) — set as side-effect by action methods
    double goalRow; // row of the node that reached the goal
    double goalCol; // col of the node that reached the goal

    // obstacle location and shape parameters (for IsObstacle)
    double circle_row = 150.0, circle_col = 225.0, circle_radius = 25.0;
    double ellipse_row = 100.0, ellipse_col = 150.0, ellipse_a = 40.0, ellipse_b = 20.0;
    double tri1_x1 = 50.0, tri1_y1 = 50.0, tri1_x2 = 75.0, tri1_y2 = 75.0, tri1_x3 = 50.0, tri1_y3 = 75.0;
    double tri2_x1 = 100.0, tri2_y1 = 50.0, tri2_x2 = 125.0, tri2_y2 = 75.0, tri2_x3 = 100.0, tri2_y3 = 75.0;
    double rhombus_cx = 200.0, rhombus_cy = 50.0, rhombus_d1 = 40.0, rhombus_d2 = 20.0;
    double square_row = 50.0, square_col = 250.0, square_size = 25.0;
    double rod_row = 150.0, rod_col = 50.0, rod_width = 10.0, rod_length = 50.0;
};
