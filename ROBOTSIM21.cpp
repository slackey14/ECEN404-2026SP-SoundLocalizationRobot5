#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <ctime>

//
// ====================== NODE STRUCT ======================
// Represents one cell (or node) on the grid for A* pathfinding.
//
struct Node {
    int x, y;            // Coordinates of the node in the grid
    double g;            // Cost from start node to this node
    double h;            // Heuristic estimate to goal node
    double f;            // Total estimated cost (f = g + h)
    Node* parent;        // Pointer to the node from which this one was reached

    // Constructor initializes coordinates and optional costs/parent
    Node(int x_, int y_, double g_=0, double h_=0, Node* parent_=nullptr)
        : x(x_), y(y_), g(g_), h(h_), f(g_+h_), parent(parent_) {}
};

//
// ====================== PRIORITY QUEUE COMPARATOR ======================
// Used by std::priority_queue to always expand the node with the smallest f-cost.
//
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;   // Lower f = higher priority
    }
};

//
// ====================== HEURISTIC FUNCTION ======================
// Manhattan distance heuristic (since movement is 4-directional).
//
double heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

//
// ====================== PATH RECONSTRUCTION ======================
// Traces back from goal node to start node using the parent pointers.
//
std::vector<Node*> reconstruct_path(Node* goal) {
    std::vector<Node*> path;
    for (Node* current = goal; current != nullptr; current = current->parent)
        path.push_back(current);
    std::reverse(path.begin(), path.end());   // Reverse so path goes from start→goal
    return path;
}

//
// ====================== GRID EXPANSION ======================
// Dynamically increases grid size outward to simulate "unknown" world expansion.
//
void expandGrid(std::vector<std::vector<int>>& grid, int expandBy) {
    int oldRows = grid.size();
    int oldCols = grid[0].size();
    int newRows = oldRows + expandBy;
    int newCols = oldCols + expandBy;

    // Add new rows at the bottom
    grid.resize(newRows, std::vector<int>(oldCols, 0));

    // Extend all rows with new columns on the right
    for (int i = 0; i < newRows; i++)
        grid[i].resize(newCols, 0);

    std::cout << " Grid expanded to " << newRows << " x " << newCols << "\n";
}

//
// ====================== RANDOM OBSTACLE GENERATION ======================
// Randomly fills a portion of cells with obstacles based on given density (0.0–1.0).
//
void addRandomObstacles(std::vector<std::vector<int>>& grid, double density = 0.1) {
    int rows = grid.size(), cols = grid[0].size();
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Only place obstacle if cell is empty and random chance < density
            if (grid[i][j] == 0 && (rand() % 100) < (density * 100))
                grid[i][j] = 1;  // 1 represents an obstacle
        }
    }
}

//
// ====================== A* SEARCH ALGORITHM ======================
// Finds the shortest path from start to goal avoiding obstacles.
//
std::vector<Node*> a_star(std::vector<std::vector<int>>& grid, Node* start, Node* goal) {
    int rows = grid.size(), cols = grid[0].size();

    // Reject invalid goals that are outside grid boundaries
    if (goal->x >= rows || goal->y >= cols) return {};

    // Priority queue for open nodes, lowest f-cost first
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open;

    // 2D closed list to track visited nodes
    std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));

    // Initialize start node
    start->h = heuristic(start->x, start->y, goal->x, goal->y);
    start->f = start->g + start->h;
    open.push(start);

    // Movement in 4 cardinal directions (no diagonals)
    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    // -------------------- MAIN A* LOOP --------------------
    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        // Skip if node was already processed
        if (closed[current->x][current->y]) continue;
        closed[current->x][current->y] = true;

        // Goal reached → return full path
        if (current->x == goal->x && current->y == goal->y)
            return reconstruct_path(current);

        // Explore all 4 neighboring cells
        for (int i = 0; i < 4; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            // Skip invalid positions (off-map or blocked)
            if (nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue;
            if (grid[nx][ny] == 1 || closed[nx][ny]) continue;

            // Compute tentative g, h, and f costs for the neighbor
            double g_new = current->g + 1.0;   // uniform cost for grid step
            double h_new = heuristic(nx, ny, goal->x, goal->y);

            // Create and enqueue neighbor node
            Node* neighbor = new Node(nx, ny, g_new, h_new, current);
            open.push(neighbor);
        }
    }

    // If queue empties without reaching goal, no path was found
    return {};
}

//
// ====================== MAIN SIMULATION ======================
// Simulates a robot navigating a 2D map with expanding exploration.
//
int main() {
    srand(time(0));  // Seed random number generator

    int initialSize = 10;  // Start with small grid
    std::vector<std::vector<int>> grid(initialSize, std::vector<int>(initialSize, 0));

    // Place random obstacles
    addRandomObstacles(grid, 0.1);

    // Define start position (center of initial grid)
    Node* currentPos = new Node(initialSize / 2, initialSize / 2);

    // Define faraway goal (forces grid expansion)
    Node* goal = new Node(50, 50);

    int steps = 0;  // Counter for how many moves taken

    // -------------------- MAIN NAVIGATION LOOP --------------------
    while (!(currentPos->x == goal->x && currentPos->y == goal->y)) {
        steps++;

        // If goal lies outside current grid, expand it
        while (goal->x >= grid.size() - 2 || goal->y >= grid[0].size() - 2) {
            expandGrid(grid, 10);
            addRandomObstacles(grid, 0.05);  // Add light new obstacle density
        }

        // Attempt to compute new A* path to goal
        auto path = a_star(grid, new Node(currentPos->x, currentPos->y), goal);

        // If no path found, expand and try again (simulate exploration)
        if (path.empty()) {
            expandGrid(grid, 10);
            addRandomObstacles(grid, 0.05);
            std::cout << " Exploring further...\n";

            if (steps > 300) {  // Safety limit
                std::cout << " Gave up after 300 expansions.\n";
                break;
            }
            continue;
        }

        // Move one step along the path (next node after current)
        if (path.size() > 1)
            currentPos = path[1];

        std::cout << " Step " << steps << ": Robot moved to (" 
                  << currentPos->x << "," << currentPos->y << ")\n";

        // Occasionally simulate detection of a new obstacle by sensors
        if (rand() % 5 == 0) {
            int ox = std::max(0, currentPos->x + (rand() % 3 - 1));  // random nearby cell
            int oy = std::max(0, currentPos->y + (rand() % 3 - 1));
            if (ox < grid.size() && oy < grid[0].size())
                grid[ox][oy] = 1;
            std::cout << " Sensor detected new obstacle at (" << ox << "," << oy << ")\n";
        }

        // Safety stop if it gets stuck too long
        if (steps > 500) {
            std::cout << " Gave up after 500 steps.\n";
            break;
        }
    }

    // -------------------- FINAL RESULT --------------------
    if (currentPos->x == goal->x && currentPos->y == goal->y)
        std::cout << "  Reached goal at (" << goal->x << "," << goal->y 
                  << ") after " << steps << " steps.\n";
    else
        std::cout << "  Could not reach goal.\n";

    return 0;
}
