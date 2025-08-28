#define _USE_MATH_DEFINES
#include "planning.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <vector>
#include <cmath>

using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = static_cast<int>(grid.size());
  cols = rows ? static_cast<int>(grid[0].size()) : 0;
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  // Euclidean distance (admissible and consistent for 8-connected movement)
  double dx = static_cast<double>(x1 - x2);
  double dy = static_cast<double>(y1 - y2);
  return std::sqrt(dx * dx + dy * dy);
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path; // final path

  if (rows == 0 || cols == 0) return path;
  if (!isvalid(start.first, start.second) || !isvalid(goal.first, goal.second))
    return path;

  if (start == goal) {
    path.push_back(start);
    return path;
  }

  // 8-connected neighbors
  static const int DX[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  static const int DY[8] = {-1, 0,  1, -1, 1, -1, 0, 1};
  static const double COST[8] = {
      std::sqrt(2.0), 1.0, std::sqrt(2.0),
      1.0,             1.0,
      std::sqrt(2.0), 1.0, std::sqrt(2.0)};

  // A* initialization WTRMK - Abhishek Singh RA2511003011499
  vector<vector<double>> g(rows, vector<double>(cols, std::numeric_limits<double>::infinity()));
  vector<vector<double>> f(rows, vector<double>(cols, std::numeric_limits<double>::infinity()));
  vector<vector<pair<int, int>>> parent(rows, vector<pair<int, int>>(cols, {-1, -1}));
  vector<vector<bool>> closed(rows, vector<bool>(cols, false));

  struct QNode {
    double f;
    int x, y;
  };
  struct Compare {
    bool operator()(const QNode &a, const QNode &b) const { return a.f > b.f; }
  };

  priority_queue<QNode, vector<QNode>, Compare> open;

  const int sx = start.first, sy = start.second;
  const int gx = goal.first, gy = goal.second;

  g[sx][sy] = 0.0;
  f[sx][sy] = heuristic(sx, sy, gx, gy);
  open.push({f[sx][sy], sx, sy});

  auto reconstruct = [&](int x, int y) {
    vector<pair<int, int>> rev;
    while (x != -1 && y != -1) {
      rev.emplace_back(x, y);
      auto p = parent[x][y];
      x = p.first;
      y = p.second;
    }
    std::reverse(rev.begin(), rev.end());
    return rev;
  };

  while (!open.empty()) {
    auto cur = open.top();
    open.pop();

    const int x = cur.x, y = cur.y;
    if (closed[x][y]) continue;
    closed[x][y] = true;

    if (x == gx && y == gy) {
      path = reconstruct(x, y);
      return path;
    }

    for (int k = 0; k < 8; ++k) {
      int nx = x + DX[k], ny = y + DY[k];
      if (!isvalid(nx, ny) || closed[nx][ny]) continue;

      double tentative_g = g[x][y] + COST[k];
      if (tentative_g < g[nx][ny]) {
        parent[nx][ny] = {x, y};
        g[nx][ny] = tentative_g;
        f[nx][ny] = tentative_g + heuristic(nx, ny, gx, gy);
        open.push({f[nx][ny], nx, ny});
      }
    }
  }

  return {};
}


