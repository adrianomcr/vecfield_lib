# vecfield_lib - vecfield_ros

This ROS package has some control classes. It also has an drone simulator and a n example of how to use the codes. The results of the simulator can be visualized on matlab.

---------------------


### Requirements

The matlab code that displays the simulation resuls has some function from the matlab_lib. Follow the instruction in <https://github.com/adrianomcr/matlab_lib> to install it.

The Eigen library should be also installed.


### Installation


Place the package in your `catkin_ws` and compile with ROS:

```bash
$ catkin build
```

Source you workspace.


### Turtlesim

There is an example controller to make the turtle, from the turtlesim_node, follow a curve. Just run:

```bash
$ rosrun turtlesim turtlesim_node
$ rosrun vecfield_ros control_turtle
```

### Drone simulation - sdandalone

This package has a simple model for a quadcopter (see the code `drone_class.cpp`). The inputs are the velocities of each of the propellers. There is also a AcroRate controller (see the code `dacrorate_class.cpp`) that allows the drone to be controlled in the AcroRate mode, thus, respond to total thrust and angular rate commands.

The code `drone_sim.cpp` uses the drone model class and the controller classes to perform a simulation. To run the sim, use this:

```bash
$ rosrun vecfield_ros drone_sim
```

The program will write the results in the `/tmp` folder. On matlab, run the script `see_logs.m` to see the results, which include an animation.


Currently, the drone simulator does not depend on any specific ROS tool, just the compiler. The code can be easily adapted to control drones in other simulators that communicate with ROS.


### Drone simulation - Coppeliasim

This simulation considers the CoppeliaSim simulator integrated with ROS. You can get the simulator at <https://www.coppeliarobotics.com/coppeliaSim> and setup the ROS communication following the instructions in <https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm>. However, I recommend the instructions [here](coppelia/README.md).

With the ROS core running, open the coppelia scene `coppelia/drone_scene.ttt` and play the simulator. If your setup is ok, you should see the topics `/drone/gt` and `/drone/input/rateThrust` if you type `rostopic list`.

Now, run the controller on a terminal:

```bash
$ rosrun vecfield_ros control_drone
```





