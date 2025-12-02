// ==================== LIBRARIES ====================
// Core Arduino functions + timing/GPIO
#include <Arduino.h>

// STL containers used for map + path planning
#include <vector>      // 2D grid map
#include <queue>       // priority queue for A*
#include <cmath>       // cos(), sin(), abs()
#include <algorithm>   // std::min, std::max, std::reverse
#include <cstdlib>     // random(), rand()
#include <utility>   // for std::pair

using std::vector;

// ==================== CONFIG / CONSTANTS ====================

// Max size the grid is allowed to grow to (prevents runaway memory use)
const int MAX_GRID = 60;

// Amount to grow the occupancy grid by each time expansion is required
const int EXPAND_BY = 10;

// Threshold at which we declare an obstacle (22.86 cm ≈ 9 inches)
const float OBST_THRESH = 22.86f;


// ==================== ULTRASONIC SENSOR PIN DEFINITIONS ====================
// (SIM version uses these as identifiers only — real mode will use pulseIn())

#define FRONT_TRIG 12
#define FRONT_ECHO 13

#define LEFT_TRIG  16
#define LEFT_ECHO  17

#define RIGHT_TRIG 18
#define RIGHT_ECHO 19

#define BACK_TRIG  21
#define BACK_ECHO  20


// ==================== MOTOR DRIVER PINS (TB6612FNG) ====================
// Each wheel has IN1, IN2 (direction) + PWM (speed)

#define M1_IN1 4
#define M1_IN2 5
#define M1_PWM 7

#define M2_IN1 2
#define M2_IN2 15
#define M2_PWM 6

#define M3_IN1 32
#define M3_IN2 33
#define M3_PWM 8

#define M4_IN1 39
#define M4_IN2 40
#define M4_PWM 9


// ==================== STATUS LED ====================
#define STATUS_LED 45   // Blinks when obstacles detected

// ==================== MOTOR DIRECTION INDICATOR LEDS ====================
// These LEDs visually show drivetrain direction in simulation / debug

#define FL_FWD_LED 10
#define FL_BWD_LED 11
#define FR_FWD_LED 35
#define FR_BWD_LED 36
#define RL_FWD_LED 37
#define RL_BWD_LED 38
#define RR_FWD_LED 41
#define RR_BWD_LED 42


// ==================== ULTRASONIC SENSOR SIMULATION ====================
// This version fakes distance readings by generating AC-like distances,
// occasionally injecting a "close object" reading. Real mode replaces this.

float getDistanceCM(int trig, int echo) {
  // Ultra-short HIGH pulse (as if triggering a sensor)
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Fake distance: between 5cm and ~120cm
  int simulatedCM = random(5, 120);

  // Simulate echo-pulse duration — *this was the crash cause before*
  unsigned long echoTime = (unsigned long)(simulatedCM * 58.2);

  // Fake echo pulse HIGH for that duration, then LOW again
  // ⚠️ This is a simulation hack — real mode uses pulseIn()
  pinMode(echo, OUTPUT);
  digitalWrite(echo, HIGH);
  delayMicroseconds(echoTime);
  digitalWrite(echo, LOW);
  pinMode(echo, INPUT);

  return simulatedCM;
}


// ==================== MOTOR & LED DRIVER HELPERS ====================

// Set one motor's direction + PWM output
void driveMotor(int in1, int in2, int pwm, bool forward, int speed) {
  digitalWrite(in1, forward);
  digitalWrite(in2, !forward);
  analogWrite(pwm, speed);
}
//PLACE HOLDER MOTOR CONTROL LOGIC
// High-level mecanum control logic for directional movement
void driveMecanum(String direction, int speed) {

  if (direction == "forward") {
    // All wheels forward
    driveMotor(M1_IN1,M1_IN2,M1_PWM,true, speed);
    driveMotor(M2_IN1,M2_IN2,M2_PWM,true, speed);
    driveMotor(M3_IN1,M3_IN2,M3_PWM,true, speed);
    driveMotor(M4_IN1,M4_IN2,M4_PWM,true, speed);

  } else if (direction == "backward") {
    // All wheels reverse
    driveMotor(M1_IN1,M1_IN2,M1_PWM,false, speed);
    driveMotor(M2_IN1,M2_IN2,M2_PWM,false, speed);
    driveMotor(M3_IN1,M3_IN2,M3_PWM,false, speed);
    driveMotor(M4_IN1,M4_IN2,M4_PWM,false, speed);

  } else if (direction == "right") {
    // Mecanum wheel strafe right
    driveMotor(M1_IN1,M1_IN2,M1_PWM,true,  speed);
    driveMotor(M2_IN1,M2_IN2,M2_PWM,false, speed);
    driveMotor(M3_IN1,M3_IN2,M3_PWM,false, speed);
    driveMotor(M4_IN1,M4_IN2,M4_PWM,true,  speed);

  } else if (direction == "left") {
    // Mecanum wheel strafe left
    driveMotor(M1_IN1,M1_IN2,M1_PWM,false, speed);
    driveMotor(M2_IN1,M2_IN2,M2_PWM,true,  speed);
    driveMotor(M3_IN1,M3_IN2,M3_PWM,true,  speed);
    driveMotor(M4_IN1,M4_IN2,M4_PWM,false, speed);

  } else {
    // Stop
    analogWrite(M1_PWM,0);
    analogWrite(M2_PWM,0);
    analogWrite(M3_PWM,0);
    analogWrite(M4_PWM,0);
  }
}

// TEMPORARILY REPLACING MOTORS FOR USE IN SIMULATION
// ==================== LED DIRECTION VISUALIZATION ====================

void visualizeMove(String direction) {
  // Reset all LEDs to off
  int leds[] = {FL_FWD_LED,FR_FWD_LED,RL_FWD_LED,RR_FWD_LED,
                FL_BWD_LED,FR_BWD_LED,RL_BWD_LED,RR_BWD_LED};
  for(int i=0;i<8;i++){
    pinMode(leds[i],OUTPUT);
    digitalWrite(leds[i],LOW);
  }

  // Light directional LEDs depending on motion vector
  if (direction == "forward") {
    digitalWrite(FL_FWD_LED,HIGH);
    digitalWrite(FR_FWD_LED,HIGH);
    digitalWrite(RL_FWD_LED,HIGH);
    digitalWrite(RR_FWD_LED,HIGH);
  }
  else if (direction == "backward") {
    digitalWrite(FL_BWD_LED,HIGH);
    digitalWrite(FR_BWD_LED,HIGH);
    digitalWrite(RL_BWD_LED,HIGH);
    digitalWrite(RR_BWD_LED,HIGH);
  }
  else if (direction == "right") {
    digitalWrite(FL_FWD_LED,HIGH);
    digitalWrite(RR_FWD_LED,HIGH);
    digitalWrite(FR_BWD_LED,HIGH);
    digitalWrite(RL_BWD_LED,HIGH);
  }
  else if (direction == "left") {
    digitalWrite(FR_FWD_LED,HIGH);
    digitalWrite(RL_FWD_LED,HIGH);
    digitalWrite(FL_BWD_LED,HIGH);
    digitalWrite(RR_BWD_LED,HIGH);
  }
}


// ==================== DYNAMIC OCCUPANCY GRID EXTENSION ====================
// Called when robot approaches an unknown boundary.

void shiftGrid(vector<vector<int>>& grid, int shiftX, int shiftY); // forward decl

void expandGrid(vector<vector<int>>& grid, int expandBy) {

  int oldRows = grid.size();
  int oldCols = grid[0].size();

  int newRows = min(oldRows + expandBy, MAX_GRID);
  int newCols = min(oldCols + expandBy, MAX_GRID);

  if (newRows == oldRows && newCols == oldCols) {
    Serial.println("❌ Map limit reached — no more expansion");
    return;
  }

  // First expand grid size
  grid.resize(newRows);
  for (int r = 0; r < newRows; r++) {
    grid[r].resize(newCols, 0);
  }

  // ✅ FULL EXPANSION SHIFT FIX:
  // Shift grid so robot remains centered relative to new space
  int shift = expandBy / 2;

  shiftGrid(grid, shift, shift);

  // Notify
  Serial.printf("🧩 Grid expanded to %dx%d and shifted by %d\n",
                newRows, newCols, shift);
}

// ==================== A* PATH PLANNING STRUCTURES ====================

// Each grid cell holds A* metadata (parent, cost, closed flag)
struct Cell {
  bool closed = false;       // already processed
  float g = INFINITY;        // cost from start
  float h = 0.0f;            // heuristic distance to goal
  float f = INFINITY;        // total cost g + h
  int parentX = -1;          // parent cell for backtracking path
  int parentY = -1;
};

// Manhattan distance heuristic for grid-based motion
inline float heuristic(int x1,int y1,int x2,int y2){
  return abs(x1-x2)+abs(y1-y2);
}


// ==================== A* SEARCH (no dynamic pointers) ====================
// Returns a list of (x,y) grid coordinates from start to goal.
// Empty vector means "no path found".

vector<std::pair<int,int>>
a_star_grid(const vector<vector<int>>& grid, int sx,int sy,int gx,int gy)
{
  int rows = grid.size();
  int cols = grid[0].size();

  // Reject invalid start/goal indices
  if (gx<0||gy<0||gx>=rows||gy>=cols) return {};
  if (sx<0||sy<0||sx>=rows||sy>=cols) return {};
  if (grid[gx][gy]==1) return {};  // Goal is blocked

  // Per-cell state map
  vector<vector<Cell>> map(rows, vector<Cell>(cols));

  // Priority queue uses (f, x, y)
  auto cmp = [&](const std::tuple<float,int,int>& a,
                 const std::tuple<float,int,int>& b){
    return std::get<0>(a) > std::get<0>(b);
  };

  std::priority_queue<
      std::tuple<float,int,int>,
      vector<std::tuple<float,int,int>>,
      decltype(cmp)
  > open(cmp);

  // Initialize start cell
  map[sx][sy].g = 0.0f;
  map[sx][sy].h = heuristic(sx,sy,gx,gy);
  map[sx][sy].f = map[sx][sy].h;
  open.emplace(map[sx][sy].f, sx, sy);

  // 4-connected grid motion
  const int dx[4] = {-1, 1, 0, 0};
  const int dy[4] = { 0, 0,-1, 1};

  // Main A* loop
  while (!open.empty()) {
    auto [cf, cx, cy] = open.top(); open.pop();
    if (map[cx][cy].closed) continue;
    map[cx][cy].closed = true;

    // Goal reached: reconstruct path
    if (cx==gx && cy==gy) {
      vector<std::pair<int,int>> path;
      int px=gx, py=gy;
      while (!(px==sx && py==sy)) {
        path.emplace_back(px,py);
        int nx = map[px][py].parentX;
        int ny = map[px][py].parentY;
        if (nx<0||ny<0) break;
        px=nx; py=ny;
      }
      path.emplace_back(sx,sy);
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Explore neighbors
    for (int k=0;k<4;k++){
      int nx = cx + dx[k];
      int ny = cy + dy[k];

      // Bounds + obstacle + closed checks
      if (nx<0||ny<0||nx>=rows||ny>= (int)grid[nx].size()) continue;
      if (grid[nx][ny]==1) continue;
      if (map[nx][ny].closed) continue;

      float tentativeG = map[cx][cy].g + 1.0f;

      // First-time visit or better path found
      if (tentativeG < map[nx][ny].g) {
        map[nx][ny].g = tentativeG;
        map[nx][ny].h = heuristic(nx,ny,gx,gy);
        map[nx][ny].f = map[nx][ny].g + map[nx][ny].h;
        map[nx][ny].parentX = cx;
        map[nx][ny].parentY = cy;
        open.emplace(map[nx][ny].f, nx, ny);
      }
    }
  }

  return {}; // No path found
}

// ===============================================================
// FIXED FINAL MAP DISPLAY
// Prints ONLY the bounding box around real activity on the grid:
//   - Path (P)
//   - Obstacles (#)
//   - Start (S)
//   - Goal (G)
// This prevents phantom obstacles from appearing far from the path.
// ===============================================================
void printMap(
  const vector<vector<int>>& grid,
  const vector<std::pair<int,int>>& path,
  int startX, int startY,
  int goalX, int goalY
) {
  int rows = grid.size();
  int cols = grid[0].size();

  // Build display buffer
  vector<vector<char>> disp(rows, vector<char>(cols, '.'));

  // Fill obstacles
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      if (grid[r][c] == 1) disp[r][c] = '#';
    }
  }

  // Mark path
  for (auto& p : path) {
    int x = p.first;
    int y = p.second;
    if (x >= 0 && x < rows && y >= 0 && y < cols)
      disp[x][y] = 'P';
  }

  // Start and goal override
  disp[startX][startY] = 'S';
  disp[goalX][goalY]   = 'G';

  // =====================================================
  // FIND ACTIVE BOUNDING BOX
  // =====================================================
  int minR = rows, maxR = -1;
  int minC = cols, maxC = -1;

  auto consider = [&](int r, int c) {
    minR = min(minR, r);
    maxR = max(maxR, r);
    minC = min(minC, c);
    maxC = max(maxC, c);
  };

  // Path bounds
  for (auto& p : path) consider(p.first, p.second);

  // Start/Goal
  consider(startX, startY);
  consider(goalX, goalY);

  // Obstacle bounds
  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++)
      if (disp[r][c] == '#')
        consider(r, c);

  // Safety fallback (empty map)
  if (maxR == -1) {
    Serial.println("🧩 Map empty — nothing to print.");
    return;
  }

  // =====================================================
  // PRINT ONLY THE RELEVANT BOX
  // =====================================================
  Serial.println("\n========== FINAL MAP ==========");

  for (int r = minR; r <= maxR; r++) {
    for (int c = minC; c <= maxC; c++) {
      Serial.print(disp[r][c]);
      Serial.print(' ');
    }
    Serial.println();
  }

  Serial.println("================================\n");
}

// ===========================================================
// SHIFT THE ENTIRE GRID CONTENTS BY (shiftX, shiftY)
// Used to keep robot centered during dynamic expansion.
// ===========================================================

void shiftGrid(vector<vector<int>>& grid, int shiftX, int shiftY) {

  int rows = grid.size();
  int cols = grid[0].size();

  vector<vector<int>> newGrid(rows, vector<int>(cols, 0));

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {

      int nr = r + shiftX;
      int nc = c + shiftY;

      if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
        newGrid[nr][nc] = grid[r][c];
      }
    }
  }

  grid = newGrid;
}
// ==================== ROBOT NAVIGATION ====================

void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());  // unpredictable noise for sim obstacles

  // Initialize indicator LEDs
  pinMode(FL_FWD_LED,OUTPUT); pinMode(FL_BWD_LED,OUTPUT);
  pinMode(FR_FWD_LED,OUTPUT); pinMode(FR_BWD_LED,OUTPUT);
  pinMode(RL_FWD_LED,OUTPUT); pinMode(RL_BWD_LED,OUTPUT);
  pinMode(RR_FWD_LED,OUTPUT); pinMode(RR_BWD_LED,OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  // ============ INITIALIZE GRID ============

  int N = 30; // initial map size
  vector<vector<int>> grid(N, vector<int>(N,0));

  // Starting pos: middle of map
  int startX = N/2;
  int startY = N/2;



// Convert sound localization data → grid offset
float direction_deg = 180.0f;      // example sound vector
float distance_in   = 126.0f;      // inches to victim estimate
float node_size_in  = 9.0f;        // cell size in inches

float distance_nodes = distance_in / node_size_in;

// Interpret direction_deg as:
// 0° = forward, 90° = right, 180° = back, 270° = left (clockwise)
float rad = radians(direction_deg);

// forward component (along +row)
float forward = cos(rad);   // 0°: +1 (forward), 180°: -1 (back)

// right component (clockwise from forward)
float right = sin(rad);     // 90°: +1 (right), 270°: -1 (left)

// Grid axes: +row = forward, +col = LEFT
int gdx = round(distance_nodes * forward);   // rows
int gdy = round(distance_nodes * -right);   // cols (minus because +col = left)

  int goalX_initial = startX + gdx;
  int goalY_initial = startY + gdy;

  // Shift grid if goal lies outside map (i.e., negative index)
  int shiftX = (std::min(startX, goalX_initial) < 0)
               ? -std::min(startX, goalX_initial) : 0;
  int shiftY = (std::min(startY, goalY_initial) < 0)
               ? -std::min(startY, goalY_initial) : 0;
  shiftGrid(grid, shiftX, shiftY);
  // Apply shift
  int curX  = startX + shiftX;
  int curY  = startY + shiftY;
  vector<std::pair<int,int>> visited;
  visited.push_back({curX, curY});   // mark starting cell

  int goalX = goalX_initial + shiftX;
  int goalY = goalY_initial + shiftY;

  int origStartX = curX;
  int origStartY = curY;

  // Expand grid to ensure goal fits after shifting
  int needR = std::min(std::max(goalX+3, N), MAX_GRID);
  int needC = std::min(std::max(goalY+3, N), MAX_GRID);
  grid.resize(needR);
  for (int r=0;r<needR;r++) grid[r].resize(needC,0);

  Serial.printf("📐 Goal shifted to (%d,%d), Start (%d,%d)\n",
                goalX, goalY, curX, curY);
  Serial.printf("📍 Robot initial position (%d,%d), Goal initial position (%d,%d)\n",
                startX, startY, goalX_initial, goalY_initial);

  // ============ NAVIGATION LOOP ============

  int steps = 0;
  while (!(curX==goalX && curY==goalY)) {

    // If robot or goal is near grid edge → expand world
    if ((goalX >= (int)grid.size() - 2) ||
        (goalY >= (int)grid[0].size() - 2))
    {
      if ((int)grid.size() < MAX_GRID ||
          (int)grid[0].size() < MAX_GRID)
      {
        // Store old positions BEFORE expansion
        int oldCurX = curX;
        int oldCurY = curY;

        int oldGoalX = goalX;
        int oldGoalY = goalY;

        // Expand + shift the grid
        expandGrid(grid, EXPAND_BY);

        int shift = EXPAND_BY / 2;

        // ✅ Shift robot
        curX  = oldCurX + shift;
        curY  = oldCurY + shift;

        // ✅ Shift goal
        goalX = oldGoalX + shift;
        goalY = oldGoalY + shift;

        // ✅ SHIFT ENTIRE PATH HISTORY (THE MISSING FIX)
        for (auto &p : visited) {
          p.first  += shift;
          p.second += shift;
        }

        Serial.printf("🧩 Grid expanded by %d\n", EXPAND_BY);
        Serial.printf("📍 Robot shifted to (%d,%d), Goal shifted to (%d,%d)\n",
                      curX, curY, goalX, goalY);
      }
    }


    // ========== SENSOR + MAPPING ==========

    float dF = getDistanceCM(FRONT_TRIG, FRONT_ECHO);
    float dB = getDistanceCM(BACK_TRIG,  BACK_ECHO);
    float dL = getDistanceCM(LEFT_TRIG,  LEFT_ECHO);
    float dR = getDistanceCM(RIGHT_TRIG, RIGHT_ECHO);

    Serial.printf("📡 Sensors F=%.1f L=%.1f R=%.1f B=%.1f\n",
                  dF,dL,dR,dB);

    // Mark occupied cells based on threshold
    bool hit=false;

    // Front obstacle → +row (forward in this convention)
    if (dF <= OBST_THRESH && curX+1 < (int)grid.size()) {
      grid[curX+1][curY]=1;
      hit=true;
      Serial.println("🚧 FRONT");
    }
    // Back obstacle → -row
    if (dB <= OBST_THRESH && curX-1 >= 0) {
      grid[curX-1][curY]=1;
      hit=true;
      Serial.println("🚧 BACK");
    }
    // Left obstacle → +col (left of robot facing down)
    if (dL <= OBST_THRESH && curY+1 < (int)grid[curX].size()) {
      grid[curX][curY+1]=1;
      hit=true;
      Serial.println("🚧 LEFT");
    }
    // Right obstacle → -col
    if (dR <= OBST_THRESH && curY-1 >= 0) {
      grid[curX][curY-1]=1;
      hit=true;
      Serial.println("🚧 RIGHT");
    }

    // Flash status LED if mapping event happened
    if (hit) {
      digitalWrite(STATUS_LED, HIGH);
      delay(60);
      digitalWrite(STATUS_LED, LOW);
    }

    // ========== A* PATH PLANNING ==========

    auto path = a_star_grid(grid, curX, curY, goalX, goalY);

    if (path.empty()) {
      // Try one emergency expansion, otherwise abort mission
      if ((int)grid.size() < MAX_GRID ||
          (int)grid[0].size() < MAX_GRID)
      {
        expandGrid(grid, EXPAND_BY);
        continue;
      }
      Serial.println("❌ Could not reach goal!");
      break;
    }

    // Path must have at least (current,next)
    if ((int)path.size() < 2) break;

    int nx = path[1].first;
    int ny = path[1].second;

    // ========== EXECUTE ONE STEP ==========

    int ddx = nx - curX;
    int ddy = ny - curY;
    String dir="stop";

    // Robot-facing-down convention:
    // +row = forward, -row = backward
    // +col = LEFT of robot, -col = RIGHT of robot
    if (ddx == 1)       dir = "forward";   // move down one row
    else if (ddx == -1) dir = "backward";  // move up one row
    else if (ddy == 1)  dir = "left";      // +col = left of robot
    else if (ddy == -1) dir = "right";     // -col = right of robot

    // Print movement direction
    if (dir=="forward")      Serial.println("⬆️ Moving FORWARD");
    else if (dir=="backward")Serial.println("⬇️ Moving BACKWARD");
    else if (dir=="right")   Serial.println("➡️ Moving RIGHT");
    else if (dir=="left")    Serial.println("⬅️ Moving LEFT");
    else                     Serial.println("⛔ STOPPED");

    // Visual and motor command
    visualizeMove(dir);
    driveMecanum(dir,150);
    delay(500);
    visualizeMove("stop");
    driveMecanum("stop",0);

    // Commit movement state
    curX = nx;
    curY = ny;
    steps++;
    visited.push_back({curX, curY});

    Serial.printf("Step %d at (%d,%d)\n", steps, curX, curY);

    if (steps > 300) {
      Serial.println("⏹️ Step cap reached");
      break;
    }
  }
  // ============================================================
  // PRINT FINAL MAP (path, obstacles, start, goal)
  // ============================================================

  // Recompute full final path from original start → goal
  printMap(grid, visited, origStartX, origStartY, goalX, goalY);


  // Final mission result
  if (curX==goalX && curY==goalY) {
    Serial.println("✅ REACHED GOAL!");
    digitalWrite(STATUS_LED, HIGH);
  } else {
    Serial.println("❌ Could not reach goal!");
  }
}

void loop() {}
