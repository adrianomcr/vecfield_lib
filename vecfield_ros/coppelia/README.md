
### CoppeliaSim + ROS

Install coppeliasim:

```bash
$ wget -P /tmp https://coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu16_04.tar.xz
$ cd /tmp && tar -xvf CoppeliaSim_Edu_V4_1_0_Ubuntu16_04.tar.xz
$ mv CoppeliaSim_Edu_V4_1_0_Ubuntu16_04 ~/
$ echo 'export COPPELIASIM_ROOT_DIR="$HOME/CoppeliaSim_Edu_V4_1_0_Ubuntu16_04"' >> ~/.bashrc && source ~/.bashrc
$ echo 'alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"' >> ~/.bashrc && source ~/.bashrc
```

Test if the installation, type `coppeliasim` on a terminal.

Get the package for ROS communication:
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/CoppeliaRobotics/simExtROSInterface --branch coppeliasim-v4.0.0
```

Fix a python requirement that is broken in CoppeliaSim 4.1.0:

```bash
$ cd $COPPELIASIM_ROOT_DIR/programming
$ rm -rf libPlugin
$ git clone https://github.com/CoppeliaRobotics/libPlugin.git
$ cd libPlugin
$ git checkout 1e5167079b84ca002a6197414d51c40eda583d01
```

Install some requirements:

```bash
$ sudo apt-get install -y python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs		
```

Compile with `catkin build`:

```bash
$ cd ~/catkin_ws
$ catkin clean -y && catkin build
```

If compilation works, copy the generated library to the CoppeliaSim folder:

```bash
$ cp ~/catkin_ws/devel/lib/libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR
```

To run the simulator with ROS communication you should run `roscore` before. Then when you run `coppelia` you should see the following line on the terminal:

```
Plugin 'RosInterface': loading...
Plugin 'RosInterface': warning: replaced variable 'simROS'
Plugin 'RosInterface': load succeeded.
```