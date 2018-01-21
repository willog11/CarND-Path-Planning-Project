# Udacity Self-Driving Car Engineer Nanodegree
## Path Planning Project
### Introduction

The goal of this project was to implement a path planning algorithm that utilizes given way points and nearby vehicles (simulating sensor fusion).  The algorithm should enable the ego vehicle to drive around the track safely, in that the ego vehicle:

- Shall not collide with any other vehicle
- Shall not go off the track (3 far right lanes)
- Shall be able to overtake when the adjacent lane is free and the current lane has slower moving traffic
- Shall not exert any extreme accelerations or jerks in both lateral and longitudinal directions 

### Implementation

The algorithm that was chosen includes using sub modules:

1. Construct interpolated waypoints of nearby area  using Frenet co-ordinate system and spline interpolation
2. Determine ego car parameters and construct vehicle object
3. Generate predictions of each vehicles and determining the next possible states of the ego vehicle
4. Calculating the cost of each potential next state and selecting the state with the lowest cost
5. Creating and using the trajectory of the lowest state so that the ego vehicle proceeds in the allocated direction and speed


#### Construct interpolated waypoints of nearby area  using Frenet co-ordinate system and spline interpolation

The Spline.h API was used as directed in the recommendations at the start of the project. This made it easy to create a projected path of the ego vehicle. Likewise the points on how to move the ego vehicle in the walk-through were followed to get the project started.

#### Utilizing the sensor fusion data to determine nearby vehicles and their predicted n+i locations.

A class called Vehicle was created to be able to create vehicle objects of all vehicles in the far 3 right lanes including the ego vehicle. This class was created to handle the kinematics of each vehicle, predicted locations, FSM for the path planner and generation of trajectories. 

The implementation used is similar to previous lessons however it includes required updates to handle latency and prevention of extreme accelerations and jerks.

#### Generate predictions of each vehicles and determining the next possible states of the ego vehicle

Now that each vehicle has been encapsulated in an object, predictions can be made on where they will be over time using their current location, speed and acceleration. 

When the next locations are known it is easy to determine what are the next possible states of the ego vehicle are. These can either be "Keep Lane", "Prepare Lane Change Left", "Prepare Lane Change Right", "Lane Change Left" and "Lane Change Right".

#### Calculating the cost of each potential next state and selecting the state with the lowest cost

Once the potential next states are known, a cost of each movement needs to be calculated. To do this the following three cost functions were implemented:

1. Inefficiency Cost -  Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
2. Goal Distance Cost -  Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory. Cost of being out of goal lane (middle lane) also becomes larger as vehicle approaches goal distance.
3. Lane Change Cost -  Cost increases when the ego vehicle wants to change lanes.

Finally the next potential state with the lowest cost is chosen.

#### Creating and using the trajectory of the lowest state so that the ego vehicle proceeds in the allocated direction and speed

The final step utilizes step one, in that the trajectory (lane location and speed) are assigned to the ego vehicle so that it moves in the correction speed and direction as calculated.

### Conclusion

The overall implementation certainly meets the criteria. However this is an area where one can spend a lot of time tweaking the algorithm to improve the system. This implementation uses the major blocks that were discussed in the previous lessons so it was a huge help in understanding the lessons further.

Some potential improvements to the the current design would be:

- To include d, d_dot and dd_dot in the vehicle class so that the lateral acceleration and jerk can be calculated with higher accuracy.
- Tweak the cost functions further to give a better path planner. However this is not so easy for my setup as I am using Docker environment so I need to continually commit my updates into git and pull and rebuild on docker each time I need to test something. This is a major inconvenience when using windows 7 machines as it  slows down development and test processes.
- Add more cost functions to further tweak the path planning decision making process.

Overall though I was able to get over 8 miles incident free driving which meets the requirements.



# CarND-Path-Planning-Project
## Self-Driving Car Engineer Nanodegree Program
  
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

