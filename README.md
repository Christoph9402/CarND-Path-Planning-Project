# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
   The objective of the Path Planning project is to develop a code that creates trajectories that a vehicle follows along a track inside a simulator. The track has a length of 4.32 miles and must be completed without incidents such as accidents or leaving the lane. In addition, the speed limit of 50 mph must not be exceeded. The maximum acceleration must be less than 10 m/s^2 and jerk less than 10 m/s^3. The car should stay within one lane and only leave this lane if a lane change makes sense. This lane change should be performed when a car in front of the ego-vehicle is driving too slowly and another lane is free for the ego-vehicle to change into.
Without an implemented code, the vehicle is initially parked on the middle lane of the track and does not drive off. To fix this I used the FaQ video by Udacity as a guide. To generate the trajectories an integer variable "lane" had to be initialized with the value 1. This indicates the lane in which the ego vehicle is located. The left lane is lane=0, the middle lane=1 and the right lane lane = 2. I also defined a variable "max_speed" which I set to 0. Next the data for the localization of the vehicle are read out - the x- and y-position of the vehicle and the frenet coordinates s and d. Also the angle of the vehicle and the speed. "previous_path_x" and "previous_path_y" contain the data of the previous path. Furthermore, the last s and d values of the previous path are read. "Sensor fusion" contains data about all vehicles with the same direction of travel.
An integer variable "previous_size" is created with the length of previous_path_x as value. Then it is checked whether this newly created integer variable is greater than zero, which indicates that a previous path with waypoints exists. If this is the case, the variable car_s gets the value of the last value from the previous path. After this if loop a boolean variable "TooClose" is defined and set to False. Three vectors - VehiclesLane0, VeiclesLane1 and VehiclesLane2 - were created. These vectors will later be used to split up the vehicles of the “sensor_fusion” list, so that it is easy to get the number of vehicles on each lane.  
Afterwards every entry of the sensor_fusion list is processed in a for loop. It is checked what d value the i vehicle has. Because these are frenet coordinates a constant d means that the lane remains constant. According to the project requirements, one lane is 4 meters wide. It is checked if the vehicle i is in the same lane as the ego vehicle, using an if condition. If this is the case, the velocity data of the vehicle in front is read and the speed is calculated. In a subsequent if statement it is checked if the distance of the vehicle in front is less than 30 meters and if the vehicle is in front of the ego vehicle. If this is the case the boolean variable "TooClose" is set to true.  
After all vehicles in the sensor_fusion list have been checked, the list is searched again with a for loop and the vehicles are assigned to the tracks. However, only vehicles that are 40 meters in front of the ego vehicle or 10 meters behind the vehicle are added to the list. As distance the difference of the s values is used to neglect the distances in the direction. If one of the conditions applies to the ith vehicle, it is added to the respective vector.  
After passing this loop it is specified what happens if a vehicle in front is too close in the same lane, i.e. if TooClose is true. if this is the case Different situations are checked using if operations. In the first case the vehicle is on the left lane and the middle lane is empty. This is verified by checking if the vector containing the vehicles in the middle lane is empty. If this is the case, the integer variable is increased by 1 - thus the ego vehicle moves one lane to the right.  
If the ego vehicle speed is less than 49.5mph, the speed is increased by 0.35 because the overtaking manoeuvre is performed. In the second case it is checked if the vehicle is in the left lane, the middle lane is occupied and the right lane is free. In this case, the lane variable is increased by 2 - the ego vehicle moves into the right lane for overtaking. I have decided to reduce the speed in this case to avoid exceeding the maximum jerk. In the third case the ego vehicle is located in the middle lane and the left lane is free, while the right lane is not free. In this case the vehicle should change to the left lane, lane is reduced by 1. The speed is increased again if it is below 49.5. Similarly, in the next case, if the ego vehicle is driving in the middle lane and the right lane is free and occupies the left lane, the speed is increased again. In this case, the vehicle changes to the right. If the ego-vehicle is in the middle lane and both the left and right lanes are free, it does not matter which lane the ego-vehicle changes to. In this case I have decided that a lane change to the left should be performed. Analog to the case that the ego-drive is in the left lane, the action is also performed when it is in the right lane, however, changing lanes to the left. If all lanes are occupied by other vehicles, the lane is not changed, so the ego-vehicle has to wait until one of the lanes becomes free for overtaking. The speed is also reduced if the vehicle in front has a higher speed. If the own vehicle is slower than the vehicle in front, the speed increases. This way the same speed as the vehicle in front is tried to be reached.  
After the if statement that describes what happens if TooClose is true, an else if statement checks if the ego vehicle is driving slower than the allowed 50 mph. I have chosen 49.5 as suggested in the FaQ so that there is a buffer and the maximum speed is never exceeded. After the if statement that describes what happens if TooClose is true, an else if statement checks if the ego vehicle is driving slower than the allowed 50 km/h. I have chosen 49.5 as suggested in the FaQ so that there is a buffer and the maximum speed is never exceeded. Afterwards the trajectory is created. First 2 vectors points_path_x and points_path_y are created. These vectors contain the points that later form the trajectory. After the definition of ref_x,ref_y and ref_yaw it is checked whether the variable "previous_size" that was defined at the beginning is smaller than 2. In this case the previous location of the ego vehicle is calculated from the x- and y- positions as well as the yaw_rate. The x and y points of this position are appended to the vector "points_path_x" and "points_path_y". If the if statement does not apply, ref_x is first given the value of the element from the vector previous_path_x at the position of previous size-1. This is also done for the y-values. Two double variables "previous_ref_x" and "previous_ref_y" are created. They get the value from the respective previous_path vector at the position previous_size-2. All points are assigned to the respective vectors "previous_path_x" and "previous_path_y".  
Then 3 vectors are created - next_waypoint0, next_waypoint1 and next_waypoint2. For these vectors the frenet coordinates are converted into X-Y coordinates. When the function is called, car_s+30 (for next_waypoint0), car_s+60 (for next_waypoint1) and car_s+90 (for next_waypoint2) are taken in place of the s-value. For the d-value (2+4*lane) is used, since the current lane of the vehicle is always taken into account and the trajectory can be easily adapted if a lane change is required. The transformed x and y points are then attached to the vectors points_path_x and points_path_y.  
In a following for loop every point in the "points_path_x" and "points_path_y" vectors is adjusted so that the reference angle of the vehicle is 0 at every position. Afterwards a spline is created using the header file "spline.h" (from https://kluge.in-chemnitz.de/opensource/spline/) . With the help of this file it should be guaranteed that the trajectory is a smooth one. The x and y values of points_path_x and points_path_y are added to this spline. Afterwards, two vectors next_x_vals and next_y_vals are each assigned the values of previous_path_x and previous_path_y to ensure that all points of the previous path are used at the beginning. Afterwards, two vectors next_x_vals and next_y_vals are each assigned the values of previous_path_x and previous_path_y to ensure that all points of the previous path are used at the beginning. Afterwards a target_x is defined. I have set this to 30m as suggested in the FaQ. The corresponding y-value to the target_x value is read out using s(target_x). To calculate the distance between the vehicle and a target point, the hypothenuse is calculated with target_x and target_y values. In a for loop a N is calculated, meaning how many points the spline has to be divided into. This distance is important because the vehicle passes one point every 0.02 seconds. If the distance between the points is too large, the vehicle will drive too fast. Afterwards the double values x-point and x-point are calculated as shown in the FaQ by converting them back to the global coordinate system and adding the values ref_y and ref_x. Finally, "x_point" and "y_point" are appended to the vectors "next_x_vals" and "next_y_vals". These then make up the trajectory.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
