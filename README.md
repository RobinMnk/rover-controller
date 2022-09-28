# Custom-built ROS2 Foxy Rover Control

A controller for a Rover implemented using the ROS2 Foxy framework. It is designed from-scratch using no library or framework for the steering or navigation as all services are custom created. In particular, the *custom-built components* include:

- A **Controller** that allows for driving smooth curves as opposed to driving straight lines only and rotating in place, both manually and autonomously
- A **Navigator** for calculating movement orders to a given target from a live position
- A complete **Lifecycle Management Framework** that services every task from mission handling to traversing a route to logging
- A **Localization concept** that relies on GPS but is also capable of employing any mixture of a **vehicle model simulation**, and a custom **Noise Handling System** which uses a kalman filter and a custom **regression model** for computing the orientation of the rover

## Packages

There are 4 packages:

- The `motor_ctrl` package contains nodes for controlling the movement of the robot. As such it includes:
  - The node `keyboard_input` for receiving and processing keyboard input to enable manual steering. The node converts pressed keys into a unified forward/rotation vector which is passed to the motor controller via the `mov` topic (for more info see *Rover Movement*).
  - The `smoothcontroller` node translates incoming movement requests from the `mov` topic into wheel speeds and controls the motor. It also publishes current motor information to the `motorstate` topic.
- In `motor_interfaces` the necessary messages concerning the motor are defined.
- The `rover_utils` package is used to access the sensors which the robot provides. It includes
  - The `ultrasonic` node uses the built-in ultrasonic sensor to publish information about the distance that is free in front of the rover to the `distance` topic. If active, the `smoothcontroller` uses this information to try to prevent the robot from colliding with obstacles head-on.
- The most important package is the `navigation` package.

## Setup

#### First Time Installation

To install the required packages and dependencies after you have setup ROS2 Foxy according to the [Installation Manual](https://docs.ros.org/en/foxy/Installation.html), you need to run the following commands once for your system

```
sudo apt install gpsd gpsd-clients
pip3 install gps3 filterpy utm
```

#### At System Startup

At **every system startup**, you need to activate the gpsd socket with the following command in order for the GPS module to work

```
sudo gpsd /dev/ttyACM0 –F /var/run/gpsd.sock
```

*Note: should your GPS module be connected as a device other than ttyACM0 change that in the above command!*

#### In every new Terminal

There are some environment variables which need to be set **in every new terminal** in order for the system to work properly:

- The `ROUTE_LOGGING_HOME` directory will contain all log files and plots from the runs.
- The `SERVER_HOME` environment variable should point to the directory which will contain the incoming `mission.json`. It will also be used to document the milestones corresponding to each mission.

Finally, navigate to the `rover_control` directory and run the following command **in every new terminal** 

````
source install/setup.bash
````

*Note that you can use the provided setup script `rover_setup.sh` to run in every new terminal window. Feel free to adapt the install directory of this project and the location of the log directories. So assuming the project is installed at `~/rover` you can just run `source rover/rover_setup.sh` in any new terminal for the complete setup.*

## Running the System

Launch the system using the provided launch file

```
ros2 launch navigation fullsystem.launch.py
```

Wait until all nodes are up and running and for the localization to be setup, this can take a short while. You should see a message like

```
[localization-1] [INFO] [1628695025.270994909] [Localization]: Localization ready 
[localization-1] [INFO] [1628695025.270994909] [Localization]: Localization initialized at
lat: 48.39682719872158
lon: 11.729478757285506
```

Then once a `mission.json` is found in the `SERVER_HOME` directory the corresponding mission is executed. The start of a new mission is indicated with this message:

```
[mission_ctrl-7] --------------------------------
[mission_ctrl-7] ------- STARTING MISSION -------
[mission_ctrl-7] --------------------------------
```

The format of the `mission.json` should be as follows:

````
{
	"mission_id": <id: string>,
	"route": [
		{
			"latitude": <latitude of stop 1: double>,
			"longitude": <longitude of stop 1: double>
		}
	]
}
````



## Run Logging

Once a mission for an incoming `mission.json` file is started, a new directory for the mission is created at `$SERVER_HOME/missions/mission_{mission_id}` and the JSON file is moved there. All milestones for the waypoints will be stored in subdirectories.

For debugging and visualization of the Rovers Runs a log file and plot will be stored in a directory named with the current date in the `ROUTE_LOGGING_HOME` directory.

## Rover Movement

The rover is controlled via messages in the `mov` topic. A movement command is a 2D vector with the `float32` fields `forward` and `rotation` as specified in the `Mov` message within the `motor_interfaces` package. The length of this vector specifies the speed and should lie in the interval [0, 1].

The `keyboard_input` node maintains such a 2D vector for controlling the rover. Pressing one the keys `w` for forward or `s` for backward sets the vector along the x-axis in the corresponding direction with a length of the current speed. Pressing `a` for left or `d` for right will rotate the current vector by a fixed angle to the corresponding side, but never exceeding a 90° turn. A vector that is rotated to one side will be reset and immediately turned to the other side if the opposite steering key is pressed (as opposed to slowly rotating it back to normal).

The `smoothcontroller` translates this 2D movement direction vector to actual left and right motor speeds using the forward component as a base and then adding/subtracting the rotation component on the left/right side. To account for the inverted steering direction when driving backwards, this adding/subtracting of the rotation component is reversed when going backwards. The conversion of a `(forward, rotation)` vector thus looks like this:

````python
motorspeed_left  = forward - sign(forward) * rotation
motorspeed_right = forward + sign(forward) * rotation
````

The conversion function is however discontinuous at points on the rotation axis, i.e., driving forward and steering all the way left results in rotating left on the spot whereas driving backwards and steering left results in rotating on the spot to the right. In both cases the corresponding final movement order is `(0, 1)`. To differentiate these cases the `smoothcontroller` remembers the last direction in which it drove, either forward or backwards and inverts the motor speeds accordingly if necessary.

