# clever_tools

Useful ROS nodes and stuff for CLEVER drone kit, that is not included to the [Raspberry Pi image](https://github.com/CopterExpress/clever/releases).

## Installation

Clone repository to the catkin workspace sources:

```bash
cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clever_tools.git
```

Execute catkin_make in catkin workspace directory and source environment setup files:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

If you don't want to install all packages in catkin workspace, you can install specified package:

```bash
cd ~/catkin_ws
catkin_make --pkg <package_name>
source devel/setup.bash
```

## clever_tools package

### Nodes

#### interactive.py

Control the drone using [interactive markers](http://wiki.ros.org/interactive_markers) in Rviz!

```bash
rosrun clever_tools interactive.py
```

## clever_flight_routines package

### Nodes

#### create_route.py

Create flying route by moving drone in space and recording its coordinates by triggering RC pitch stick (channel 2).

```bash
usage: rosrun clever_tools create_route.py [-h] [-f FRAME_ID] [filename]

positional arguments:
  filename              Filename of route csv table. Default is route.csv.

optional arguments:
  -h, --help            show help message and exit
  -f FRAME_ID, --frame_id FRAME_ID
                        Coordinates will be recorded relative to frame_id
                        parameter. Default is map.
```

#### fly_route.py

Fly route from the csv table with x y z values.

```bash
usage: rosrun clever_tools fly_route.py [-h] [-r] [-z HEIGHT] [-f FRAME_ID] [-s SPEED] [filename]

positional arguments:
  filename              Filename of the route csv table

optional arguments:
  -h, --help            show this help message and exit
  -r, --repeat          Program will repeat the route infinite if you use this
                        arg.
  -z HEIGHT, --height HEIGHT
                        Height of flight. Default is 1 m. If height is 'nan'
                        then the drone will takeoff to 1 m and fly on the
                        route point heights.
  -f FRAME_ID, --frame_id FRAME_ID
                        Coordinates will be used relative to frame_id
                        parameter. Default is map.
  -s SPEED, --speed SPEED
                        Copter speed. Default is 1 m/s.
```

#### test_flight.py

Just example node of flight: takeoff to 1m, fly forward 1m, and land.

### Python module

#### Usage example

```bash
#!/usr/bin/env python
import rospy
from clever_flight_routines import get_telemetry, takeoff, reach_point, land

rospy.init_node('clever_flight_example')

# copter parameters

speed = 1
z = 1
frame = 'map'
x0 = get_telemetry(frame_id = frame).x
y0 = get_telemetry(frame_id = frame).y

# flight program

takeoff(z)                                                  # takeoff
reach_point(x=x0, y=y0+1, z=z, speed=speed, frame_id=frame) # flight 1m toward
land()
```

#### Documentation

Documentation can be found [here](clever_flight_routines/README.md)

