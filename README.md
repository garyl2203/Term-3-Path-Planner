# Path Planning Project - Term 3

### Objective

The aim is to get a car to drive and stay in a lane , with cars overtaking slower cars. Of course we would like to minimise the jerks and limit max accelrations. We will also try to avoid collisions with other vehicles or objects. 

The speed limit is 50 mph, passing other vehicles sharing the highway. We are provided sensor fusion data as well as telemetry data. It also provides the previous path that was used by the car in the previous step.

The simulato provides telemtry data as well as sensor fusion data. It also provides the previous path of the car.


### Path Planning

1. We generate five points in Frenet cooridnates, as we want the car to go along a path.
The first two points are our last two points in the previous path. The nexy three points in our target lane are spaced 30 metres apart, as explained in the project walkhrough.

2. Next, we generate a spline using the provided spline.h. We use spline instead of a polynomial (in MPC) as it is a peicewise function and will go through all of the points instead of the polynomial. There are advantages and disadvantages to both. 

3. For this, we use spline as we donnt want the car to stray away from our predicted path as it may hit other objects. Afterwards, we add pints on the spline, (our path points) to next_x_vals and next_y_vals.

4. Set a ref_velocity that is slightly lower than the speed limit.

5. When adding points to our path, keep a distance of .02 x ref velocity / 2.24 between them.


We try to find a path that minnimizes the jerk. This is similar to a human deciding on where to drive on the road ahead, or 'the brains' of the vehicle.
Next, we use sensor fusion data to navigate and reduce speed or accelrate when necessary.

We create 5 points in Frenet coordinates. The first two points are taken as  the last two points in the previous path, and the next 3 points in our target lane which is spaced 30 metres apart. Fortunately we already have a peice of code (spline.h) provided which will create a spline through these points by solving a quintic polynomial.

### Staying In Lane

Specifically, we set d = 2 + 4 x d since it is given that a lane is 4m wide and we'd like to stay in the center of the lane because that seems safest. We would like to stay within our limits, perhaps slightly lower.

For changing lanes, we start at the inital lane of 1, of there are 3 lanes, then the lanes will be 0,1,2. 
For vehicle speed, a loop is made to continously check the current speed, and if there is a vehicle in front.

### Avoid Collision

In order to find whether there is any obstacle in the road, we should be using this sensor fusion data and understand the environment around us. Based on our environment, determine whether to continue moving in the same speed, or change lane &/or reduce speed. 

From the path planning lectures, we can do this by implementing a cost function.
The function called lane_shift uses a simple cost based evaluation of various vehicles that are in front and adjacent lanes. We decide when the car will shift lane based on how expensive it is to change to the other lane. If cars are far away, we can simply put low costs to allow car to change lanes safely. But if our car detects a car is getting closer(ahead of us or behind us), we can implement a high cost of action, which will force the car to either shift lane to over take car, or stay in lane so we dont crash.

Depending up on the cost, our car either changes the lane to left or right or remains in the same lane and reduces its speed.


### Known Issues

On rare occasions, when the car is changing to the most right side lane, during the curved highway sections, the car may go out of lane on the right side, briefly for a few seconds. 

Also very rarely, when our car is surrounded by multiple cars on the left, right and on the front, when the car sees a chance to change lane, a sudden accelaration jerk maybe expereinced by very briefly and the warning sign is barely noticed.

Also, though not an 'issue', the car may seem to be changing lanes multiple times, when a car is in front of our lane, and when the car is in front of another lane. This is due to the program algorithm, if our car sees a chance to switch lane, given there are no cars behind us(or beside us), it will switch, and since those conditions are met in our lane, and the next lane, given the cars in front are roughly the same speed within each other, the car may seem indecisive. Once another car slows down or speeds up, our Mario car is able to overtake 1 or the other vehicle and drive normally.




# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

