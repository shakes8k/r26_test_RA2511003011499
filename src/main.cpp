#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> 
#include "gridmap.h"
#include "odometry.h"
#include "planning.h"
#include "ublox_reader.h"

using namespace std;

// Helper to convert angle to unit direction
pair<double, double> directionFromAngle(double angle_deg) {
  double rad = angle_deg * M_PI / 180.0;
  return {cos(rad), sin(rad)};
}

int main(int argc, char *argv[]) {

  if (argc < 3) {
    cerr << "usage: " << argv[0] << " <gps_data_file> <output_file" << endl;
    return 1;
  }

  string gps_data = argv[1];
  string odom_commands = argv[2];

  auto result = readUbloxFile(gps_data);
  if(static_cast<int>(result.first.lat)==0 && static_cast<int>(result.first.lon)==0 && static_cast<int>(result.second.lat)==0 && static_cast<int>(result.second.lon)==0)
  {
    cout<<"Error: Invalid GPS Coordinates"<<endl;
    return 1;
  }
  cout << "Start -> Lat: " << result.first.lat << " Lon: " << result.first.lon
       << endl;
  cout << "Goal  -> Lat: " << result.second.lat << " Lon: " << result.second.lon
       << endl;

  // initialize Gridmapper with start as origin
  GPS origin = {result.first.lat, result.first.lon};
  double cellsize = 1.0; 
  int rows = 10, cols = 10;
  Gridmapper grid(origin, cellsize, rows, cols);

  // convert start and goal GPS to grid coordinates
  pair<int, int> start = grid.gpstogrid(result.first);
  pair<int, int> goal = grid.gpstogrid(result.second);

  cout << "Start (grid) -> (" << start.first << "," << start.second << ")"
       << endl;
  cout << "Goal  (grid) -> (" << goal.first << "," << goal.second << ")"
       << endl;

  Planner planner(grid.getGrid());
  auto path = planner.pathplanning(start, goal);

  cout << "Planned Path:" << endl;
  for (auto &p : path) {
    cout << "(" << p.first << "," << p.second << ") ";
  }
  cout << endl;

  // Odometry results for terminal + seprt
  cout << "\nOdometry Commands" << endl;
  double wheel_radius = 0.05; 
  double rpm = 120;           //  WTRMK - RA2511003011499 - Abhishek Singh
  Odometry odo(wheel_radius, rpm);
  auto commands = odo.computeCommands(path);

  ofstream result_file(odom_commands);

  if (!result_file.is_open()) {
    cerr << "Error: cannot open file " << odom_commands << endl;
    return 1;
  }

  cout << fixed << setprecision(4);
  cout << "Time (s): " << commands.time_sec << "\n";
  cout << "Total rotation (deg): " << commands.angle_deg << "\n";


  result_file << commands.time_sec << endl << commands.angle_deg << endl;

  result_file.close();

  return 0;
}
