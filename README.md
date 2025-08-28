# RA26_Test
# Solution
## Understanding
This project takes raw GPS data from a u-blox receiver in UBX hex format and converts it into usable navigation commands for a rover. The GPS data is first decoded to extract latitude, longitude, and height, then mapped into a local grid around the start position. Using this grid, an A* path planner computes a collision free path from the start gps point to the goal gps point. The path is then translated into odometry commands based on the rover’s wheel radius and RPM: total traversal time (distance ÷ velocity) and total angular rotation. 
The final outputs required were:
* The time it would take to traverse the planned path (based on wheel radius and RPM).
* The total angular rotation the rover performs while following the path.

## Thought Process
I broke the problem into clear stages, mirroring the actual navigation pipeline of a rover:
* Decode UBX GPS data → convert from raw hex to latitude, longitude, and height
* Grid mapping → transform GPS coordinates into a local grid representation with obstacles
* Path planning → use the A* algorithm on the grid to find a path from start cell to goal cell
* Odometry → convert the path into rover motion commands, specifically computing time and angle   based on wheel radius and RPM

** During development I realized that time always matched (distance ÷ velocity), but rotation didnt, by analyzing the expected outputs, I saw a pattern in some cases the grader used shortest turns plus a final parking rotation while in others they used accumulated counter-clockwise turns. This explained why one testcase gave 225°, another 720°, and another 540° — all multiples of 45° but from different conventions
So the solution needed to be flexible enough to handle both smooth paths and zig-zag paths that inflate rotation totals, yet my own solutions did not make up the mark but I shared it in <odometry.cpp> comments, on how I approached the problem. I have also given a descriptive information regarding it below:

## Implementations
* UBX Decoder:
1.  Implemented decodeUBX() to extract NAV-POSLLH fields. Handled both full UBX frames and shorter “headerless” inputs. Converted lat/lon from 1e-7 degrees to degrees, and height from mm to meters
2. Each field was unpacked using memcpy into the classId struct, then converted into human-friendly units
3. Finally, a helper function readUbloxFile() was used to read two lines from a file (start and goal positions) and convert them into GPS structures

* Grid Mapper 
1. The start GPS coordinate was taken as the origin (0,0) of the grid map.
2. For demonstration and testing, obstacles were hard-coded (walls), the grid was initialized as a 2D boolean array

* Path Planner 
1. Each grid cell (row, col) Neighbors: 8 directions were allowed (up, down, left, right, and 4 diagonals).
2. **Euclidean distance** from the current gps coordinate to the goal point was admissible and consistent for this 8- connected grid 

* Odometry
1. Calculated path length from successive coordinates, then converted to meters (× cellsize)
2. Linear velocity from wheel radius and RPM: v = 2 * pi * R * (RPM/60)
3. For rotation implemented both shortest turn and CCW accumulated strategies but neither worked perfectly. available in <odometry.cpp> comments


# Problems
1. Provided test cases did not have a full UBX frame (which were sync bytes) but that was not a major issue 
2. The main persistent problem in this project was the rotation mismatch to which I have discussed above, and gave a few solutions in odometry.cpp comments yet resoluted to over shortest cumulative rotational angular change only.

# Thoughts
It was a really fun project/test. Thank you

if possible i will create a makefile (the original one would still work but just in case ) for this project alongwith the code but the compilation would still work with (since i am using cpp 17)
1.  >g++ -std=c++17 -O2 -Isrc -Ilib //including all the cpp files here// -o rover.exe #example

2. rover.exe test/testcase{x}.txt outputresults{x}.txt #example

3. About the total rotation part, I believe solving it through the shortest path might be the best approach:

(Given testcase below assumes that the ROVER is at a 45 degree turn on its original coordinate)

-> ![image](https://github.com/user-attachments/assets/314e5dd8-536b-48d1-a423-3d4391174e08)
