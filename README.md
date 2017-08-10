# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: ./images/Simulator.png "Sim"

# Write Up
In this project we create a trajectory planner and behavioral planner to safely navigate a car around a test track.

![alt text][image1]


## Video
A video of the car successfully navigating the full length of the track (4.23 miles) can be found below:


## General Overview
My implementation was as follows:

###1)Lane scoring: 
First I rank each lane based on the number of cars in the lanes and their distance from my cars current position in "s" Frenet coordinates. The farther away cars are and the less cars in the lane, the higher the rank of the lane. The scores I chose by hand. The result was an estimate of what lane we should target.

###2)Finite State Machine (FSM):
My FSM has 3 states, Keep Lane and Change Lane Left or Right. For each state I compute a cost and then choose the lowest cost state as the state of the vehicle. My costs are based on predictions of collisions, slowdowns from the target speed and costs associated with changing lanes.

a) For the collisions cost, I extract the other cars in the FSM State's target lane and then extrapolate their trajectory over a fixed time horizon based on their current speed. I also extrapolate our cars position based on the current speed. If any part of the predicted trajectory of our car or the other cars are within a certain threshold then I assign a higher cost. The threshold was hand tuned as well as the gain applied to the cost value.

b) For the slowdown cost, I apply a gain to the difference between the current speed and the target 50MPH speed limit. This was designed to force us to change lanes if our speed was getting too low. The gain was again tuned by hand.

c) For the lane change cost, I set another hand tuned gain so that there was a high cost to changing lanes. I did not want the vehicle constantly changing lanes, it should only change in certain situations.

d) Once I had computed the costs, I ranked these costs, took the lowest and used the associated FSM state as the desired action. 

###3)Set Target Lane and Speed
From this I set a target speed and target lane. For the most part this was setting the target speed to around the speed limit of 50 MPH and setting the target lane to the desired lane. I did have to add some contingencies for cars stopping suddenly (an emergency brake) and allowing lane changes to complete before starting another lane change.

###4)Path Generation
Ther target lane and speed was fed into the path generation logic provided in the Udacity Term 3 Project 1 Walkthrough Youtube video. This path generation logic merges previous and current path using splines and ensures a constant spacing between waypoints sent to the simulator. The target velocity we send is set as a reference and the velocity is slowly incremented or decremented using a fixed acceleration value that was tuned to be below the 10 m/s/s limit. The target lane is converted to a "d" Frenet coordinate and set as a final point in a spline that forms the trajectory. This spline is sampled such that waypoints are evenly spaced.

## Challenges
The most challenging part of this project was merging the current and previous paths to minimize jerk and acceleration spikes. I spent nearly 3 weeks trying different methods with mixed results. The Udacity Term 3 Project 1 Walkthrough was a fantastic explanation that helped resolve many of those issues.

## Further Improvements
The system works most of the time, but lacks robustness. The fixed time horizon we plan trajectories over limits our ability to react quickly to dynamic situations. Also tuning all the gains by hand is not an efficient method for setting the costs, generating more complex cost functions would be a better solution.

## Acknowledgements
This was an extremely difficult project for me, but I am grateful to the Udacity team and my fellow students for their support.

## References
[1] Udacity Self Driving Car Forums
[2] Udacity #pathplanning Slack Channel 
[3] https://medium.com/@mohankarthik/path-planning-in-highways-for-an-autonomous-vehicle-242b91e6387d
[4] https://discussions.udacity.com/t/latency-handling/322156/26

------------------------------------------
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

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
