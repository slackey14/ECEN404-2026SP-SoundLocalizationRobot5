#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y;
    double g, h, f;
    Node* parent;

    Node(int x_, int y_, double g_ = 0, double h_ = 0, Node* parent_ = nullptr)
        : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(parent_) {}
};

struct CompareF {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;
    }
};

double heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

bool isValid(int x, int y, const std::vector<std::vector<int>>& grid) {
    return y >= 0 && x >= 0 && y < grid.size() && x < grid[0].size() && grid[y][x] == 0;
}

std::vector<std::pair<int,int>> reconstructPath(Node* node) {
    std::vector<std::pair<int,int>> path;
    while (node != nullptr) {
        path.push_back({node->x, node->y});
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<int,int>> aStar(
    std::vector<std::vector<int>>& grid,
    std::pair<int,int> start,
    std::pair<int,int> goal
) {
    int rows = grid.size();
    int cols = grid[0].size();

    std::priority_queue<Node*, std::vector<Node*>, CompareF> openList;
    std::vector<std::vector<bool>> closedList(rows, std::vector<bool>(cols, false));

    Node* startNode = new Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second));
    openList.push(startNode);

    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        if (current->x == goal.first && current->y == goal.second) {
            std::vector<std::pair<int,int>> path = reconstructPath(current);
            delete current;
            while(!openList.empty()) {
                delete openList.top();
                openList.pop();
            }
            return path;
        }

        closedList[current->y][current->x] = true;

        for (int i = 0; i < 4; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (!isValid(nx, ny, grid) || closedList[ny][nx])
                continue;

            double gNew = current->g + 1;
            double hNew = heuristic(nx, ny, goal.first, goal.second);

            Node* neighbor = new Node(nx, ny, gNew, hNew, current);
            openList.push(neighbor);
        }
    }

    return {};
}

// Print grid top-to-bottom with path overlay
void printGridWithPath(std::vector<std::vector<int>> grid, const std::vector<std::pair<int,int>>& path,
                       std::pair<int,int> start, std::pair<int,int> goal) {
    for (auto& p : path) {
        int x = p.first;
        int y = p.second;
        if (grid[y][x] == 0)
            grid[y][x] = 2; // mark path
    }

    std::cout << "\nGrid (0=free, 1=obstacle, *=path, S=start, G=goal):\n";

    // print from top (y=0) to bottom (y=max)
    for (int y = 0; y < grid.size(); y++) {
        for (int x = 0; x < grid[y].size(); x++) {
            if (x == start.first && y == start.second)
                std::cout << "S ";
            else if (x == goal.first && y == goal.second)
                std::cout << "G ";
            else if (grid[y][x] == 1)
                std::cout << "■ ";
            else if (grid[y][x] == 2)
                std::cout << "* ";
            else
                std::cout << ". ";
        }
        std::cout << "\n";
    }
}

int main() {
    // grid[y][x]: 0 = free space, 1 = obstacle
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 1},
        {1, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 1, 0, 1}
    };

    std::pair<int,int> start = {0, 0}; // x, y
    std::pair<int,int> goal = {3, 4};  // x, y

    std::vector<std::pair<int,int>> path = aStar(grid, start, goal);

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (auto& p : path)
            std::cout << "(" << p.first << "," << p.second << ") ";
        std::cout << "\n";

        printGridWithPath(grid, path, start, goal);
    } else {
        std::cout << "No path found.\n";
    }

    return 0;
}
