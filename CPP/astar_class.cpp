#include "includes/astar_class.h"
#include <iostream>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Astar::Astar(Node start, std::pair<double, double> goal, int clearance, int radius)
    : start(start), goal(goal), clearance(clearance), radius(radius),
      numRows(200), numCols(300), move_i(0.0), move_j(0.0),
      goalRow(0.0), goalCol(0.0)
{}

// ---------------------------------------------------------------------------
// IsValid
// ---------------------------------------------------------------------------
bool Astar::IsValid(double currRow, double currCol) {
    // TODO: return true if currRow and currCol are within the valid region:
    //       [1+radius+clearance, numRows-radius-clearance]
    //   x   [1+radius+clearance, numCols-radius-clearance]

    double maxRow = currRow+radius+clearance;
    double minRow = numRows-radius-clearance;
    double maxCol = currCol+radius+clearance;
    double minCol = numCols-radius-clearance;
    if (currRow < maxRow && currRow > minRow && currCol < maxCol && currCol > minCol) {
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------
// IsObstacle
// ---------------------------------------------------------------------------
bool Astar::IsObstacle(double row, double col) {
    double sum_of_c_and_r  = clearance + radius;
    double sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r;

    // Get the distance from the starting position to the edge of the circle obstacle, subtracting (clearance + radius)
    double dist1 = sqrt((row-circle_row)*(row-circle_row) + (col-circle_col)*(col-circle_col)) - (circle_radius + sum_of_c_and_r);

    // Get the distance from the starting position to the edge of the ellipse obstacle, subtracting (clearance + radius)
    double dx = col - ellipse_col;
    double dy = row - ellipse_row;

    double dist2 =
        (dx*dx)/(ellipse_a*ellipse_a) + (dy*dy)/(ellipse_b*ellipse_b) - 1.0;

    // Triangle 1: collision check via convex polygon half-space tests
    double s1 = (triOrient(tri1_x1, tri1_y1,
                           tri1_x2, tri1_y2,
                           tri1_x3, tri1_y3) > 0)
                    ? -1.0
                    : 1.0;

    double t1_1 = s1 * edgeFn(tri1_x1, tri1_y1, tri1_x2, tri1_y2, col, row);
    double t1_2 = s1 * edgeFn(tri1_x2, tri1_y2, tri1_x3, tri1_y3, col, row);
    double t1_3 = s1 * edgeFn(tri1_x3, tri1_y3, tri1_x1, tri1_y1, col, row);

    double dist3 = std::max({t1_1, t1_2, t1_3}); // <= 0 → inside triangle 1

    // Triangle 2: collision check via convex polygon half-space tests
    double s2 = (triOrient(tri2_x1, tri2_y1,
                           tri2_x2, tri2_y2,
                           tri2_x3, tri2_y3) > 0)
                    ? -1.0
                    : 1.0;

    double t2_1 = s2 * edgeFn(tri2_x1, tri2_y1, tri2_x2, tri2_y2, col, row);
    double t2_2 = s2 * edgeFn(tri2_x2, tri2_y2, tri2_x3, tri2_y3, col, row);
    double t2_3 = s2 * edgeFn(tri2_x3, tri2_y3, tri2_x1, tri2_y1, col, row);

    double dist4 = std::max({t2_1, t2_2, t2_3}); // <= 0 → inside triangle 2

    // Rhombus: convex obstacle collision check (diagonal-aligned)
    // check rhombus (half-plane test; dist5/dist6 = 0 if inside else 1)
    double x1 = (10.0 - sqrt_of_c_and_r), y1 = 225.0;
    double x2 = 25.0, y2 = (200.0 - sqrt_of_c_and_r);
    double x3 = (40.0 + sqrt_of_c_and_r), y3 = 225.0;
    double x4 = 25.0, y4 = (250.0 + sqrt_of_c_and_r);

    double first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1));
    double second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2));
    double third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3));
    double fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4));

    double dist5 = 1.0;
    double dist6 = 1.0;

    if (first >= 0.0 && second >= 0.0 && third >= 0.0 && fourth >= 0.0)
    {
        dist5 = 0.0;
        dist6 = 0.0;
    }

    // Square: collision check using axis-aligned bounding constraints (AABB)
    double x1 = (150.0 - sqrt_of_c_and_r), y1 = 50.0;
    double x2 = (120.0 - sqrt_of_c_and_r), y2 = 75.0;
    double x3 = 150.0, y3 = (100.0 + sqrt_of_c_and_r);
    double x4 = (185.0 + sum_of_c_and_r), y4 = (75.0 + (sum_of_c_and_r * 0.5148));

    double first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1));
    double second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2));
    double third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3));
    double fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4));

    double dist7 = 1.0;
    double dist8 = 1.0;

    if (first <= 0.0 && second <= 0.0 && third <= 0.0 && fourth <= 0.0)
    {
        dist7 = 0.0;
        dist8 = 0.0;
    }

    // Rod: collision check for elongated rectangular obstacle (AABB)
    double first = ((col - 95.0) * (8.66 + sqrt_of_c_and_r)) - ((5.0 + sqrt_of_c_and_r) * (row - 30.0 + sqrt_of_c_and_r));

    double second = ((col - 95.0) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (row - 30.0 + sqrt_of_c_and_r));

    double third = ((col - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (row - 67.5));

    double fourth = ((col - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (row - 76.15 - sqrt_of_c_and_r));

    double dist9 = 1.0;
    double dist10 = 1.0;

    if (first <= 0.0 && second >= 0.0 && third >= 0.0 && fourth >= 0.0)
    {
        dist9 = 0.0;
        dist10 = 0.0;
    }

    if (dist1 <= 0.0 || dist2 <= 0.0 ||
        dist3 == 0.0 || dist4 == 0.0 ||
        dist5 == 0.0 || dist6 == 0.0 ||
        dist7 == 0.0 || dist8 == 0.0 ||
        dist9 == 0.0 || dist10 == 0.0)
    {
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Action methods
// Each method:
//   1. Computes the new heading angle
//   2. Sets move_i = sin(rad), move_j = cos(rad)
//   3. Returns IsValid(nextRow, nextCol) && !IsObstacle(nextRow, nextCol)
// ---------------------------------------------------------------------------
bool Astar::ActionMove30Up(double currRow, double currCol, int currAngle) {
    // TODO: int theta = (currAngle + 30) % 360;
    //       double rad = theta * M_PI / 180.0;
    //       move_i = std::sin(rad);  move_j = std::cos(rad);
    //       return IsValid(currRow + move_i, currCol + move_j)
    //           && !IsObstacle(currRow + move_i, currCol + move_j);
    return false;
}

bool Astar::ActionMove60Up(double currRow, double currCol, int currAngle) {
    // TODO
    return false;
}

bool Astar::ActionMove30Down(double currRow, double currCol, int currAngle) {
    // TODO
    return false;
}

bool Astar::ActionMove60Down(double currRow, double currCol, int currAngle) {
    // TODO
    return false;
}

bool Astar::ActionMoveStraight(double currRow, double currCol, int currAngle) {
    // TODO: int theta = currAngle;
    //       double rad = theta * M_PI / 180.0;
    //       move_i = std::sin(rad);  move_j = std::cos(rad);
    //       return IsValid(currRow + move_i, currCol + move_j)
    //           && !IsObstacle(currRow + move_i, currCol + move_j);
    return false;
}

// ---------------------------------------------------------------------------
// CheckIfGoal
// ---------------------------------------------------------------------------
bool Astar::CheckIfGoal(double currRow, double currCol) {
    // TODO: double check = (currRow-goal.first)*(currRow-goal.first)
    //                    + (currCol-goal.second)*(currCol-goal.second)
    //                    - 1.5*1.5;
    //       if (check <= 0) { goalRow = currRow; goalCol = currCol;
    //                         std::cout << "goal reached\n"; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Run  (A* search)
// ---------------------------------------------------------------------------
std::tuple<std::vector<Node>, std::vector<Node>, double> Astar::Run() {
    std::unordered_map<Node, double, NodeHash> distMap;
    std::unordered_map<Node, Node,   NodeHash> pathMap;
    std::unordered_map<Node, bool,   NodeHash> visited;

    // TODO: for row in [1..numRows] step 0.5
    //         for col in [1..numCols] step 0.5
    //           for angle in {0,30,60,...,330}
    //             Node n = {row, col, angle};
    //             distMap[n] = inf;  pathMap[n] = SENTINEL;  visited[n] = false;

    std::vector<Node> explored_states;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    // TODO: distMap[start] = 0.0;
    //       pq.push({0.0, start});

    while (!pq.empty()) {
        auto [f_cost, currNode] = pq.top();
        pq.pop();

        // TODO: if (visited[currNode]) continue;
        // TODO: visited[currNode] = true;
        // TODO: explored_states.push_back(currNode);

        auto [currRow, currCol, currAngle] = currNode;

        // TODO: if (CheckIfGoal(currRow, currCol)) break;

        int a1 = (currAngle + 30) % 360;
        int a2 = (currAngle + 60) % 360;
        int a3 = ((currAngle - 30) % 360 + 360) % 360;
        int a4 = ((currAngle - 60) % 360 + 360) % 360;

        // List of (action, next_angle, step_cost)
        // For each move:
        //   if action(currRow, currCol, currAngle):
        //     double nextRow  = currRow + std::round(move_i);
        //     double nextCol  = currCol + std::round(move_j);
        //     Node   nextNode = {nextRow, nextCol, next_angle};
        //     double g_next   = distMap[currNode] + step_cost;
        //     if (!visited[nextNode] && distMap[nextNode] > g_next) {
        //         double h_next = std::sqrt(std::pow(nextRow - goal.first,  2)
        //                                + std::pow(nextCol - goal.second, 2));
        //         distMap[nextNode] = g_next;
        //         pathMap[nextNode] = currNode;
        //         pq.push({g_next + h_next, nextNode});
        //     }
        //
        // TODO: ActionMove30Up   (a1, 1.12)
        // TODO: ActionMove60Up   (a2, 1.12)
        // TODO: ActionMove30Down (a3, 1.12)
        // TODO: ActionMove60Down (a4, 1.12)
        // TODO: ActionMoveStraight (currAngle, 1.0)
    }

    // TODO: check near-goal nodes for any finite distMap entry (no-path check)
    //       find bestAngle at (goalRow, goalCol) with minimum distMap value
    //       backtrack through pathMap until SENTINEL, reverse the list
    //       return {explored_states, backtrack_states, distMap[{goalRow,goalCol,bestAngle}]}

    return {{}, {}, std::numeric_limits<double>::infinity()};
}

// ---------------------------------------------------------------------------
// Animate
// ---------------------------------------------------------------------------
void Astar::Animate(const std::vector<Node>& explored,
                    const std::vector<Node>& backtrack,
                    const std::string& videoPath,
                    const std::string& imagePath) {
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    cv::VideoWriter out(videoPath, fourcc, 20.0, cv::Size(numCols, numRows));
    cv::Mat image = cv::Mat::zeros(numRows, numCols, CV_8UC3);

    // TODO: draw explored states in orange (B=255, G=150, R=0)
    //   for each state in explored:
    //     int r = numRows - (int)std::get<0>(state);
    //     int c = (int)std::get<1>(state) - 1;
    //     image.at<cv::Vec3b>(r, c) = {255, 150, 0};
    //     if (count % 75 == 0) out.write(image);
    //     count++;

    // TODO: draw free (unvisited, non-obstacle) cells in green (B=154, G=250, R=0)
    //   for row in [1..numRows], col in [1..numCols]:
    //     if pixel is still black && IsValid(row,col) && !IsObstacle(row,col):
    //       image.at<cv::Vec3b>(numRows-row, col-1) = {154, 250, 0};
    //       if (count % 75 == 0) out.write(image);

    // TODO: draw backtrack path in blue (B=0, G=0, R=255)
    //   for each state in backtrack:
    //     image.at<cv::Vec3b>(...) = {0, 0, 255};
    //     out.write(image);
    //     cv::imshow("result", image);
    //     cv::waitKey(5);

    if (!imagePath.empty())
        cv::imwrite(imagePath, image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    out.release();
}
