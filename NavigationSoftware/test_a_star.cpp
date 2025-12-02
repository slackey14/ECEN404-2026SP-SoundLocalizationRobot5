#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <climits>
using namespace std;

// =======================================================
// LOAD ASCII MAP (#/. with S and G anywhere)
// =======================================================
vector<vector<int>> loadMap(const string& filename,
                            int& sx, int& sy,
                            int& gx, int& gy)
{
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "ERROR: Cannot open " << filename << "\n";
        return {};
    }

    vector<string> lines;
    string line;

    while (getline(file, line)) {
        if (line.empty()) continue;
        lines.push_back(line);
    }

    if (lines.empty()) {
        cerr << "ERROR: Map file is empty\n";
        return {};
    }

    int rows = lines.size();
    int cols = lines[0].size();

    vector<vector<int>> grid(rows, vector<int>(cols, 0));

    sx = sy = gx = gy = -1;

    for (int r = 0; r < rows; r++) {
        if ((int)lines[r].size() != cols) {
            cerr << "ERROR: Inconsistent row length.\n";
            return {};
        }

        for (int c = 0; c < cols; c++) {
            char ch = lines[r][c];

            if (ch == '#')
                grid[r][c] = 1;       // obstacle
            else
                grid[r][c] = 0;       // free

            if (ch == 'S') { sx = r; sy = c; grid[r][c] = 0; }
            if (ch == 'G') { gx = r; gy = c; grid[r][c] = 0; }
        }
    }

    if (sx < 0 || sy < 0 || gx < 0 || gy < 0) {
        cerr << "ERROR: Missing S or G in map.\n";
        return {};
    }

    return grid;
}

// Manhattan heuristic
int H(int x1,int y1,int x2,int y2) {
    return abs(x1-x2) + abs(y1-y2);
}

// =======================================================
// A* SEARCH (dynamic size)
// =======================================================
struct Node {
    int x = 0, y = 0;
    int g = INT_MAX;
    int h = 0;
    int f = INT_MAX;
    int parentX = -1;
    int parentY = -1;
    bool closed = false;
};

vector<pair<int,int>> a_star(const vector<vector<int>>& grid,
                             int sx,int sy,int gx,int gy)
{
    int R = grid.size();
    int C = grid[0].size();

    vector<vector<Node>> map(R, vector<Node>(C));

    auto cmp = [&](const Node &a, const Node &b){ return a.f > b.f; };
    priority_queue<Node, vector<Node>, decltype(cmp)> open(cmp);

    // init start node
    map[sx][sy].x = sx;
    map[sx][sy].y = sy;
    map[sx][sy].g = 0;
    map[sx][sy].h = H(sx,sy,gx,gy);
    map[sx][sy].f = map[sx][sy].h;
    open.push(map[sx][sy]);

    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = { 0, 0,-1, 1};

    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        int x = cur.x;
        int y = cur.y;

        if (map[x][y].closed) continue;
        map[x][y].closed = true;

        // reached goal → reconstruct path
        if (x == gx && y == gy) {
            vector<pair<int,int>> path;
            while (!(x == sx && y == sy)) {
                path.push_back({x,y});
                Node &p = map[x][y];
                x = p.parentX;
                y = p.parentY;
            }
            path.push_back({sx,sy});
            reverse(path.begin(), path.end());
            return path;
        }

        // explore neighbors
        for (int k = 0; k < 4; k++) {
            int nx = x + dx[k];
            int ny = y + dy[k];

            if (nx < 0 || ny < 0 || nx >= R || ny >= C) continue;
            if (grid[nx][ny] == 1) continue;     // obstacle
            if (map[nx][ny].closed) continue;

            int newG = map[x][y].g + 1;
            if (newG < map[nx][ny].g) {
                map[nx][ny].x = nx;
                map[nx][ny].y = ny;
                map[nx][ny].g = newG;
                map[nx][ny].h = H(nx,ny,gx,gy);
                map[nx][ny].f = newG + map[nx][ny].h;
                map[nx][ny].parentX = x;
                map[nx][ny].parentY = y;

                open.push(map[nx][ny]);
            }
        }
    }

    return {}; // no path
}

// =======================================================
// PRINT MAP WITH PATH OVERLAID
// =======================================================
void printMapWithPath(const vector<vector<int>>& grid,
                      const vector<pair<int,int>>& path,
                      int sx, int sy,
                      int gx, int gy)
{
    int R = grid.size();
    int C = grid[0].size();

    // Build display buffer
    vector<vector<char>> disp(R, vector<char>(C, '.'));

    // Base: obstacles and free space
    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            if (grid[r][c] == 1)
                disp[r][c] = '#';
            else
                disp[r][c] = '.';
        }
    }

    // Mark start and goal
    disp[sx][sy] = 'S';
    disp[gx][gy] = 'G';

    // Overlay path (except start and goal)
    for (auto &p : path) {
        int x = p.first;
        int y = p.second;
        if ((x == sx && y == sy) || (x == gx && y == gy)) continue;
        if (disp[x][y] == '.') disp[x][y] = 'P';
    }

    // Print
    cout << "\n=== MAP WITH PATH ===\n";
    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            cout << disp[r][c];
        }
        cout << "\n";
    }
    cout << "=====================\n\n";
}

// =======================================================
// MAIN
// =======================================================
int main() {
    int sx, sy, gx, gy;

    auto grid = loadMap("Ushape.txt", sx, sy, gx, gy);
    if (grid.empty()) return 1;

    auto path = a_star(grid, sx, sy, gx, gy);

    if (path.empty()) {
        cout << "NO PATH FOUND\n";
        return 0;
    }

    cout << "Found path! Length = " << path.size()-1 << "\n";
    for (auto &p : path)
        cout << "(" << p.first << "," << p.second << ")\n";

    printMapWithPath(grid, path, sx, sy, gx, gy);

    return 0;
}
