# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Method and Implementation

#### Trajectory Generation

The project includes some code to generate a spline that will not collide with vehicles ahead and will attempt to maintain a maximum set speed. Here I use [this](https://kluge.in-chemnitz.de/opensource/spline/) spline generation code to create a smooth path. In this case, the speed is set to 49.5MPH to stay below the 50MPH limit. Since it builds off of previous trajectories, we are able to maintain smooth trajectories between computation cycles. The spline uses 3 reference points spaced 30 meters apart from the last point in the last trajectory to create a smooth path that follows the center of the lane.
There are a few shortcomings with this method.
1. Matching the speed of the vehicle ahead is crude. It decrements by 0.224 MPH (or 0.1 m/s) if a vehicle is within 30 meters ahead of the ego. It will then increment the speed when it is further than 30 meters away. When converted to acceleration (assuming one increment or decrement by this amount per computation cycle of 20ms), this gets a +/-5 m/s^2 rate which is safely below the +/-10 m/s^2 limit. The shortcoming of this is that maintaining distance from a vehicle ahead is not smooth. The vehicle accelerates and decelerates between 30 meters of distance. A better implementation would be to match the speed and use a PID controller to maintain a more constant distance.
2. We don't have much control over the jerk in the trajectory and assume the road is within these constraints. If the road conditions were more diverse, we would see trajectories that we would have to reduce speed for in order to stay within the jerk constraints.

#### Path Planning

Once the basic vehicle following logic was implemented, the next step was to add safe and smooth lane changes to the logic. Here I implement a very basic dual-state state machine. The vehicle can either:
- keep lane

or

- change lane

The logic is as follows:
The vehicle will keep the current lane so long as it is driving the maximum speed (a condition that is always met unless there is a car ahead). If the speed is less than the target speed (49.5MPH), I use sensor fusion data to see if the other lanes are open. If a vehicle exists in a window that is at most 10m behind the ego and at most 30m ahead of the ego, that lane is considered blocked and not safe for a lane change. Otherwise, it is marked as safe. Then, a lane to change to is picked. It must be unblocked and only one lane change of direction since changing two lanes at once often fails the jerk constraint and could cause unsafe driving situations. It will transition to the lane change state until the ego current position and last trajectory position is within a certain bound in the center of the lane. This prevents rapid successive lane changes that could be too fast and fail the jerk constraint.

This planning algorithm is very simple in that it always prioritizes speed and never has to make turns. Lane change logic is simple but does not allow for much expansion if other maneuvers are needed. For example, a maneuver that requires the ego to slow down to get around a slow moving car would be difficult to add to this implementation, but could very simply be added to a set of cost functions and a larger state machine.
