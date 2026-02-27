#include "includes/astar_class.h"
#include <iostream>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Astar::Astar(Node start, std::pair<double, double> goal, int clearance, int radius)
    : start(start), goal(goal), clearance(clearance), radius(radius),
      numRows(200), numCols(300), move_i(0.0), move_j(0.0),
      goalRow(0.0), goalCol(0.0)
{
}

// ---------------------------------------------------------------------------
// IsValid
// ---------------------------------------------------------------------------
bool Astar::IsValid(double currRow, double currCol)
{
    double rowMin = 1.0 + radius + clearance;
    double rowMax = numRows - radius - clearance;
    double colMin = 1.0 + radius + clearance;
    double colMax = numCols - radius - clearance;
    return currRow >= rowMin && currRow <= rowMax && currCol >= colMin && currCol <= colMax;
}

// ---------------------------------------------------------------------------
// IsObstacle
// ---------------------------------------------------------------------------
bool Astar::IsObstacle(double row, double col)
{
    double sum_of_c_and_r = clearance + radius;
    double sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r;

    // Get the distance from the starting position to the edge of the circle obstacle, subtracting (clearance + radius)
    double dist1 = sqrt((row - circle_row) * (row - circle_row) + (col - circle_col) * (col - circle_col)) - (circle_radius + sum_of_c_and_r);

    // Ellipse: inflate semi-axes by sum_of_c_and_r (matches Python)
    double dx = col - ellipse_col;
    double dy = row - ellipse_row;
    double a_inf = ellipse_a + sum_of_c_and_r;
    double b_inf = ellipse_b + sum_of_c_and_r;
    double dist2 = (dy * dy) / (b_inf * b_inf) + (dx * dx) / (a_inf * a_inf) - 1.0;

    // Triangle 1: vertices inflated to match Python
    double t1_x1 = 120.0 - 2.62 * sum_of_c_and_r, t1_y1 = 20.0 - 1.205 * sum_of_c_and_r;
    double t1_x2 = 150.0 - sqrt_of_c_and_r,        t1_y2 = 50.0;
    double t1_x3 = 185.0 + sum_of_c_and_r,          t1_y3 = 25.0 - sum_of_c_and_r * 0.9247;
    double t1_e1 = ((col - t1_y1) * (t1_x2 - t1_x1)) - ((t1_y2 - t1_y1) * (row - t1_x1));
    double t1_e2 = ((col - t1_y2) * (t1_x3 - t1_x2)) - ((t1_y3 - t1_y2) * (row - t1_x2));
    double t1_e3 = ((col - t1_y3) * (t1_x1 - t1_x3)) - ((t1_y1 - t1_y3) * (row - t1_x3));
    double dist3 = 1.0;
    if (t1_e1 <= 0.0 && t1_e2 <= 0.0 && t1_e3 <= 0.0) dist3 = 0.0;

    // Triangle 2: vertices inflated to match Python
    double t2_x1 = 150.0 - sqrt_of_c_and_r,  t2_y1 = 50.0;
    double t2_x2 = 185.0 + sum_of_c_and_r,    t2_y2 = 25.0 - sum_of_c_and_r * 0.9247;
    double t2_x3 = 185.0 + sum_of_c_and_r,    t2_y3 = 75.0 + sum_of_c_and_r * 0.5148;
    double t2_e1 = ((col - t2_y1) * (t2_x2 - t2_x1)) - ((t2_y2 - t2_y1) * (row - t2_x1));
    double t2_e2 = ((col - t2_y2) * (t2_x3 - t2_x2)) - ((t2_y3 - t2_y2) * (row - t2_x2));
    double t2_e3 = ((col - t2_y3) * (t2_x1 - t2_x3)) - ((t2_y1 - t2_y3) * (row - t2_x3));
    double dist4 = 1.0;
    if (t2_e1 >= 0.0 && t2_e2 >= 0.0 && t2_e3 >= 0.0) dist4 = 0.0;

    // Rhombus: convex obstacle collision check (diagonal-aligned)
    // check rhombus (half-plane test; dist5/dist6 = 0 if inside else 1)
    double r_x1 = (10.0 - sqrt_of_c_and_r);
    double r_y1 = 225.0;
    double r_x2 = 25.0;
    double r_y2 = (200.0 - sqrt_of_c_and_r);
    double r_x3 = (40.0 + sqrt_of_c_and_r); 
    double r_y3 = 225.0;
    double r_x4 = 25.0; 
    double r_y4 = (250.0 + sqrt_of_c_and_r);

    double first = ((col - r_y1) * (r_x2 - r_x1)) - ((r_y2 - r_y1) * (row - r_x1));
    double second = ((col - r_y2) * (r_x3 - r_x2)) - ((r_y3 - r_y2) * (row - r_x2));
    double third = ((col - r_y3) * (r_x4 - r_x3)) - ((r_y4 - r_y3) * (row - r_x3));
    double fourth = ((col - r_y4) * (r_x1 - r_x4)) - ((r_y1 - r_y4) * (row - r_x4));

    double dist5 = 1.0;
    double dist6 = 1.0;

    if (first >= 0.0 && second >= 0.0 && third >= 0.0 && fourth >= 0.0)
    {
        dist5 = 0.0;
        dist6 = 0.0;
    }

    // Square: collision check using axis-aligned bounding constraints (AABB)
    double s_x1 = (150.0 - sqrt_of_c_and_r), s_y1 = 50.0;
    double s_x2 = (120.0 - sqrt_of_c_and_r), s_y2 = 75.0;
    double s_x3 = 150.0, s_y3 = (100.0 + sqrt_of_c_and_r);
    double s_x4 = (185.0 + sum_of_c_and_r), s_y4 = (75.0 + (sum_of_c_and_r * 0.5148));

    double sq_first  = ((col - s_y1) * (s_x2 - s_x1)) - ((s_y2 - s_y1) * (row - s_x1));
    double sq_second = ((col - s_y2) * (s_x3 - s_x2)) - ((s_y3 - s_y2) * (row - s_x2));
    double sq_third  = ((col - s_y3) * (s_x4 - s_x3)) - ((s_y4 - s_y3) * (row - s_x3));
    double sq_fourth = ((col - s_y4) * (s_x1 - s_x4)) - ((s_y1 - s_y4) * (row - s_x4));

    double dist7 = 1.0;
    double dist8 = 1.0;

    if (sq_first <= 0.0 && sq_second <= 0.0 && sq_third <= 0.0 && sq_fourth <= 0.0)
    {
        dist7 = 0.0;
        dist8 = 0.0;
    }

    // Rod: collision check for elongated rectangular obstacle (AABB)
    double rod_first  = ((col - 95.0) * (8.66 + sqrt_of_c_and_r)) - ((5.0 + sqrt_of_c_and_r) * (row - 30.0 + sqrt_of_c_and_r));
    double rod_second = ((col - 95.0) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (row - 30.0 + sqrt_of_c_and_r));
    double rod_third  = ((col - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (row - 67.5));
    double rod_fourth = ((col - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (row - 76.15 - sqrt_of_c_and_r));

    double dist9 = 1.0;
    double dist10 = 1.0;

    if (rod_first <= 0.0 && rod_second >= 0.0 && rod_third >= 0.0 && rod_fourth >= 0.0)
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
bool Astar::ActionMove30Up(double currRow, double currCol, int currAngle)
{
    int theta = (currAngle + 30) % 360;
    double rad = theta * M_PI / 180.0;
    move_i = std::sin(rad);
    move_j = std::cos(rad);
    return IsValid(currRow + move_i, currCol + move_j) && !IsObstacle(currRow + move_i, currCol + move_j);
}

bool Astar::ActionMove60Up(double currRow, double currCol, int currAngle)
{
    int theta = (currAngle + 60) % 360;
    double rad = theta * M_PI / 180.0;
    move_i = std::sin(rad);
    move_j = std::cos(rad);
    return IsValid(currRow + move_i, currCol + move_j) && !IsObstacle(currRow + move_i, currCol + move_j);
}

bool Astar::ActionMove30Down(double currRow, double currCol, int currAngle)
{
    int theta = (currAngle - 30) % 360;
    double rad = theta * M_PI / 180.0;
    move_i = std::sin(rad);
    move_j = std::cos(rad);
    return IsValid(currRow + move_i, currCol + move_j) && !IsObstacle(currRow + move_i, currCol + move_j);
}

bool Astar::ActionMove60Down(double currRow, double currCol, int currAngle)
{
    int theta = (currAngle - 60) % 360;
    double rad = theta * M_PI / 180.0;
    move_i = std::sin(rad);
    move_j = std::cos(rad);
    return IsValid(currRow + move_i, currCol + move_j) && !IsObstacle(currRow + move_i, currCol + move_j);
}

bool Astar::ActionMoveStraight(double currRow, double currCol, int currAngle)
{
    int theta = currAngle;
    double rad = theta * M_PI / 180.0;
    move_i = std::sin(rad);
    move_j = std::cos(rad);
    return IsValid(currRow + move_i, currCol + move_j) && !IsObstacle(currRow + move_i, currCol + move_j);
}

// ---------------------------------------------------------------------------
// CheckIfGoal
// ---------------------------------------------------------------------------
bool Astar::CheckIfGoal(double currRow, double currCol)
{
    double check = (currRow - goal.first) * (currRow - goal.first) + (currCol - goal.second) * (currCol - goal.second) - 1.5 * 1.5;
    if (check <= 0)
    {
        goalRow = currRow;
        goalCol = currCol;
        std::cout << "goal reached\n";
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Run  (A* search)
// ---------------------------------------------------------------------------
std::tuple<std::vector<Node>, std::vector<Node>, double> Astar::Run()
{
    std::unordered_map<Node, double, NodeHash> distMap;
    std::unordered_map<Node, Node, NodeHash> pathMap;
    std::unordered_map<Node, bool, NodeHash> visited;

    for (double row = 1.0; row <= numRows; row += 0.5)
    {
        for (double col = 1.0; col <= numCols; col += 0.5)
        {
            for (int angle = 0; angle < 360; angle += 30)
            {
                Node n = {row, col, angle};
                distMap[n] = std::numeric_limits<double>::infinity();
                pathMap[n] = {-1.0, -1.0, -1}; // SENTINEL
                visited[n] = false;
            }
        }
    }

    std::vector<Node> explored_states;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    distMap[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty())
    {
        auto [f_cost, currNode] = pq.top();
        pq.pop();

        if (visited[currNode])
            continue;
        visited[currNode] = true;
        explored_states.push_back(currNode);

        auto [currRow, currCol, currAngle] = currNode;

        if (CheckIfGoal(currRow, currCol))
            break;

        int action1 = (currAngle + 30) % 360;
        int action2 = (currAngle + 60) % 360;
        int action3 = ((currAngle - 30) % 360 + 360) % 360;
        int action4 = ((currAngle - 60) % 360 + 360) % 360;

        for (const auto &[action_valid, next_angle, step_cost] : std::vector<std::tuple<bool, int, double>>{
                 {ActionMove30Up(currRow, currCol, currAngle), action1, 1.12},
                 {ActionMove60Up(currRow, currCol, currAngle), action2, 1.12},
                 {ActionMove30Down(currRow, currCol, currAngle), action3, 1.12},
                 {ActionMove60Down(currRow, currCol, currAngle), action4, 1.12},
                 {ActionMoveStraight(currRow, currCol, currAngle), currAngle, 1.0}})
        {
            if (action_valid)
            {
                double nextRow = currRow + std::round(move_i);
                double nextCol = currCol + std::round(move_j);
                Node nextNode = {nextRow, nextCol, next_angle};
                double g_next = distMap[currNode] + step_cost;

                if (!visited[nextNode] && distMap[nextNode] > g_next)
                {
                    double h_next = std::sqrt(std::pow(nextRow - goal.first, 2) +
                                              std::pow(nextCol - goal.second, 2));
                    distMap[nextNode] = g_next;
                    pathMap[nextNode] = currNode;
                    pq.push({g_next + h_next, nextNode});
                }
            }
        }

        std::vector<std::tuple<bool, int, double>> actions = {
            {ActionMove30Up(currRow, currCol, currAngle), action1, 1.12},
            {ActionMove60Up(currRow, currCol, currAngle), action2, 1.12},
            {ActionMove30Down(currRow, currCol, currAngle), action3, 1.12},
            {ActionMove60Down(currRow, currCol, currAngle), action4, 1.12},
            {ActionMoveStraight(currRow, currCol, currAngle), currAngle, 1.0}};
    }

    // No-path check: find the angle at goalRow/goalCol with the lowest g-cost
    int bestAngle = -1;
    double bestDist = std::numeric_limits<double>::infinity();
    for (int angle = 0; angle < 360; angle += 30)
    {
        Node n = {goalRow, goalCol, angle};
        auto it = distMap.find(n);
        if (it != distMap.end() && it->second < bestDist)
        {
            bestDist = it->second;
            bestAngle = angle;
        }
    }

    if (bestAngle == -1)
    {
        std::cout << "No path found\n";
        return {explored_states, {}, std::numeric_limits<double>::infinity()};
    }

    // Backtrack from goal to start through pathMap, then reverse
    std::vector<Node> backtrack_states;
    Node curr = {goalRow, goalCol, bestAngle};
    while (curr != SENTINEL)
    {
        backtrack_states.push_back(curr);
        curr = pathMap[curr];
    }
    std::reverse(backtrack_states.begin(), backtrack_states.end());

    return {explored_states, backtrack_states, bestDist};
}

// ---------------------------------------------------------------------------
// Animate
// ---------------------------------------------------------------------------
void Astar::Animate(const std::vector<Node> &explored,
                    const std::vector<Node> &backtrack,
                    const std::string &videoPath,
                    const std::string &imagePath)
{
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    cv::VideoWriter out(videoPath, fourcc, 20.0, cv::Size(numCols, numRows));
    cv::Mat image = cv::Mat::zeros(numRows, numCols, CV_8UC3);

    // Draw explored states in orange (BGR: 255, 150, 0)
    int count = 0;
    for (const auto &state : explored)
    {
        int r = numRows - (int)std::get<0>(state);
        int c = (int)std::get<1>(state) - 1;
        image.at<cv::Vec3b>(r, c) = {255, 150, 0};
        if (count % 75 == 0)
            out.write(image);
        count++;
    }

    // Draw free (unvisited, non-obstacle) cells in green (BGR: 154, 250, 0)
    for (int row = 1; row <= numRows; row++)
    {
        for (int col = 1; col <= numCols; col++)
        {
            if (image.at<cv::Vec3b>(numRows - row, col - 1) == cv::Vec3b{0, 0, 0} && IsValid(row, col) && !IsObstacle(row, col))
            {
                image.at<cv::Vec3b>(numRows - row, col - 1) = {154, 250, 0};
                if (count % 75 == 0)
                    out.write(image);
                count++;
            }
        }
    }

    // Draw backtrack path in blue (BGR: 0, 0, 255)
    for (const auto &state : backtrack)
    {
        int r = numRows - (int)std::get<0>(state);
        int c = (int)std::get<1>(state) - 1;
        image.at<cv::Vec3b>(r, c) = {0, 0, 255};
        out.write(image);
        cv::imshow("result", image);
        cv::waitKey(5);
    }

    if (!imagePath.empty())
        cv::imwrite(imagePath, image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    out.release();
}
