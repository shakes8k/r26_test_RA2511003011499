#define _USE_MATH_DEFINES
#include "odometry.h"
#include <cmath>
#include <ctime>
#include <iterator>
#include <numeric>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}
// Compute odometry commands
MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res{0.0, 0.0};
  if (path.size() < 2 || linear_vel <= 0.0) return res;

  double total_dist = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_dist += distance(path[i-1].first, path[i-1].second,
                           path[i].first,   path[i].second);
  }
  // total angle turned
  auto normalize180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };

  double total_turn = 0.0;
  if (path.size() >= 3) {
    // heading of first segment
    double prev_heading = angle(path[0].first, path[0].second,
                                path[1].first, path[1].second);
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      double curr_heading = angle(path[i].first, path[i].second,
                                  path[i+1].first, path[i+1].second);
      double d = normalize180(curr_heading - prev_heading);
      total_turn += std::abs(d);
      prev_heading = curr_heading;
    }
  }
  //  initial turn to face the first segment,
  total_turn += std::abs(normalize180(angle(path[0].first, path[0].second, path[1].first, path[1].second)));

  // 3) Convert dt to time with cst lv
  res.time_sec  = total_dist / linear_vel;  
  res.angle_deg = total_turn;               

  return res;
}










/* Other Solutions I tried to fix total angular change -- mixed up solutions, consider reading Readme.MD */







// TRIAL PART 1 -- AUTO ROTATION HEURISTIC

/* /* ----------------  using AUTO rotation heuristic ----------------
 * Time is still distance / linear_vel (assumes 1 mtr/cell by default here).
 * For rotation:
 *   - Build segment headings in degrees.
 *   - Compute two candidates:
 *       1) Shortest-turn sum (include initial turn from 0°, and a final park
 *          to the nearest multiple of 45°).
 *       2) CCW-accumulated sum (non-negative deltas), with the same initial
 *          and final handling.
 *   - Detect a zig-zag pattern (A–B–A–B), and choose CCW if it’s present
 *     at least twice; otherwise choose shortest-turn, snapped to a multiple of 45°.
 * 
 * // CODE - WTRMK - RA2511003011499 - Abhishek Singh
 
MotionCommand Odometry::computeCommands(vector<pair<int,int>> &path) {
  MotionCommand res{0.0, 0.0};
  if (path.size() < 2 || linear_vel <= 0.0) return res;

  double total_dist_cells = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_dist_cells += distance(path[i-1].first, path[i-1].second,
                                 path[i].first,   path[i].second);
  }
  const double cellsize_m = 1.0; // default assumption for challenger main.cpp
  const double total_dist_m = total_dist_cells * cellsize_m;

  vector<double> headings;
  headings.reserve(path.size() - 1);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    headings.push_back(angle(path[i].first, path[i].second,
                             path[i+1].first, path[i+1].second));
  }

  auto to0_360 = [](double a){
    while (a < 0.0)   a += 360.0;
    while (a >= 360.) a -= 360.0;
    return a;
  };
  auto norm180 = [](double a){
    while (a > 180.0)  a -= 360.0;
    while (a <= -180.) a += 360.0;
    return a;
  };
  auto park45 = [](double yaw){
    double k = std::round(yaw / 45.0);
    return k * 45.0;
  };

  vector<double> heads360; heads360.reserve(headings.size());
  for (double h : headings) heads360.push_back(to0_360(h));

  const double initial_yaw = 0.0; // assume start facing East

  double shortest = std::abs(norm180(headings.front() - initial_yaw));
  for (size_t i = 1; i < headings.size(); ++i) {
    shortest += std::abs(norm180(headings[i] - headings[i-1]));
  }
  double shortest_final = std::abs(norm180(park45(headings.back()) - headings.back()));
  double shortest_total = shortest + shortest_final;
-
  double ccw = to0_360(heads360.front() - to0_360(initial_yaw));
  for (size_t i = 1; i < heads360.size(); ++i) {
    ccw += to0_360(heads360[i] - heads360[i-1]);
  }
  double ccw_final = to0_360(to0_360(park45(headings.back())) - heads360.back());
  double ccw_total = ccw + ccw_final;

  int switches = 0;
  for (size_t i = 2; i < headings.size(); ++i) {
    double d1 = norm180(headings[i-1] - headings[i-2]);
    double d2 = norm180(headings[i]   - headings[i-1]);
    if (d1 * d2 < 0.0 && std::abs(std::abs(d1) - 45.0) < 1e-6 && std::abs(std::abs(d2) - 45.0) < 1e-6) {
      switches++;
    }
  }

// Strong zigzag for swtiches >=2
  double total_turn;
  if (switches >= 2) {
    total_turn = ccw_total;
  } else {
    total_turn = std::round(shortest_total / 45.0) * 45.0;
  }

  res.time_sec  = total_dist_m / linear_vel;
  res.angle_deg = total_turn;
  return res;
}





// TRIAL PART 2 -- MIXED UP END RESULT CASES 








/* ---------------- Parameterized version (explicit control) ----------------
 * Keeps your original knobs for when you want to force a specific convention.


MotionCommand Odometry::computeCommands(vector<pair<int,int>> &path,
                                        bool include_initial_turn,
                                        bool include_final_turn,
                                        double initial_yaw_deg,
                                        double desired_final_yaw_deg,
                                        double cellsize_m) {
  MotionCommand res{0.0, 0.0};
  if (path.size() < 2 || linear_vel <= 0.0) return res;

  auto normalize180 = [](double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  };

  double total_dist_cells = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_dist_cells += distance(path[i-1].first, path[i-1].second,
                                 path[i].first,   path[i].second);
  }
  const double total_dist_m = total_dist_cells * cellsize_m;

  vector<double> headings;
  headings.reserve(path.size() - 1);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    headings.push_back(angle(path[i].first, path[i].second,
                             path[i+1].first, path[i+1].second));
  }

  double total_turn = 0.0;
  if (include_initial_turn) {
    total_turn += std::abs(normalize180(headings.front() - initial_yaw_deg));
  }
  for (size_t i = 1; i < headings.size(); ++i) {
    total_turn += std::abs(normalize180(headings[i] - headings[i-1]));
  }
  if (include_final_turn) {
    total_turn += std::abs(normalize180(desired_final_yaw_deg - headings.back()));
  }

  res.time_sec  = total_dist_m / linear_vel;
  res.angle_deg = total_turn;
  return res; */
