#define _USE_MATH_DEFINES
#include "gridmap.h"
#include <cmath>
#include <iostream>

using namespace std;

Gridmapper::Gridmapper(GPS origin, double cellsize, int rows, int cols)
    : origin(origin), cellsize(cellsize), rows(rows), cols(cols),
      grid(rows, std::vector<bool>(cols, false)) {
  makemap();
}

pair<int, int> Gridmapper::gpstogrid(const GPS &point) const {
  // meters per degree using mid-latitude for better accuracy
  const double lat_mid = (point.lat + origin.lat) * 0.5;
  const double lat_mid_rad = deg2rad(lat_mid);

  // local tangent-plane approximations (thanks to gpt)
  const double meters_per_deg_lat = 111132.954 - 559.822 * std::cos(2 * lat_mid_rad) + 1.175 * std::cos(4 * lat_mid_rad);
  const double meters_per_deg_lon = 111132.954 * std::cos(lat_mid_rad);

  const double dlat = point.lat - origin.lat;
  const double dlon = point.lon - origin.lon; 

  const double north_m = dlat * meters_per_deg_lat; // +north
  const double east_m  = dlon * meters_per_deg_lon; // +east

  int row = static_cast<int>(std::floor(north_m / cellsize));
  int col = static_cast<int>(std::floor(east_m  / cellsize));

  if (!isvalid(row, col)) return {-1, -1};
  return {row, col};
}

const vector<vector<bool>> &Gridmapper::getGrid() const { return grid; }

double Gridmapper::deg2rad(double deg) { return deg * M_PI / 180.0; }

bool Gridmapper::isvalid(int row, int col) const {
  return (row >= 0 && row < rows && col >= 0 && col < cols);
}

void Gridmapper::makemap() {
  for (int r = 3; r < 8 && r < rows; r++) {
    if (2 >= 0 && 2 < cols) grid[r][2] = true; // vertical wall at col 2
  } // WTRMK - RA2511003011499 - Abhishek Singh
  if (6 >= 0 && 6 < rows) {
    for (int c = 4; c < 9 && c < cols; c++) {
      grid[6][c] = true; // horizontal wall at row 6
    }
  }
}
// debugging
void Gridmapper::printgrid() const {
  cout << "Grid map (" << rows << "x" << cols << "), 1=true=obstacle:\n";
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      cout << (grid[i][j] ? "1 " : "0 ");
    }
    cout << '\n';
  }
}
