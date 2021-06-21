# vecfield_lib - vecfield_ros

This ROS package has some control classes. It also has an drone simulator and a n example of how to use the codes. The results of the simulator can be visualized on matlab.

---------------------


### Requirements

The matlab code that displays the simulation resuls has some function from the matlab_lib. Follow the instruction in <https://github.com/adrianomcr/matlab_lib> to install it.



### Drone simulation

This package has a simple model for a quadcopter (see the code `drone_class.cpp`). The inputs are the velocities of each of the propellers. There is also a AcroRate controller (see the code `dacrorate_class.cpp`) that allows the drone to be controlled in the AcroRate mode, thus, respond to total thrust and angular rate commands.

The code `drone_sim.cpp` uses the drone model class and the controller classes to perform a simulation. Place the package in your `catkin_ws` and compile with ROS:

```bash
$ catkin build
```

Source you workspace and run:

```bash
$ rosrun vecfield_ros drone_sim
```

The program will write the results in the `/tmp` folder. On matlab, run the script `see_logs.m` to see the results, which include an animation.


#### Note

Currently, the simulator does not depend on any specific ROS tool, just the compiler. Other codes can use these classes to control other robots in ROS.
