# clever_tools

Useful ROS nodes and stuff for CLEVER drone kit, that is not included to the [Raspberry Pi image](https://github.com/CopterExpress/clever/releases).

## Installation

Clone repository to the catkin workspace sources:

```bash
cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clever_tools.git
```

Execute catkin_make in catkin workspace directory:

```bash
cd ~/catkin_ws
catkin_make clever_tools
```

## Nodes

### interactive.py

Control the drone using [interactive markers](http://wiki.ros.org/interactive_markers) in Rviz!

```bash
rosrun clever_tools interactive.py
```

### create_route.py

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

### fly_route.py

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

### test_flight.py

Just example node of flight: takeoff to 1m, fly forward 1m, and land.

## Python modules

### flight_routines

#### Usage example

```
import rospy
from flight_routines import *

rospy.init_node('flight routines node')

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


#### Globals

```bash
TOLERANCE = 0.2             # m
SPEED = 1.0                 # m/s
TAKEOFF_SPEED = 1.0         # m/s
TAKEOFF_HEIGHT = 1.0        # m
LOCAL_FRAME_ID = 'map'
```

#### Service proxies

Available proxies to services:
* `navigate` to service `/navigate`
* `navigate_global` to service `/navigate_global`
* `set_position` to service `/set_position`
* `set_velocity` to service `/set_velocity`
* `set_attitude` to service `/set_attitude`
* `set_rates` to service `/set_rates`
* `get_telemetry` to service `get_telemetry`
* `land` to service `/land`
* `arming` to service `/mavros/cmd/arming`
* `set_mode` to service `/mavros/set_mode`

#### Functions

```bash
get_distance(x1, y1, z1, x2, y2, z2)
```

Return distance betwen 2 points: `(x1,y1,z1)` and `(x2,y2,z2)`.

```bash
takeoff(height=TAKEOFF_HEIGHT, speed=TAKEOFF_SPEED, tolerance=TOLERANCE, frame_id=LOCAL_FRAME_ID)
```

Takeoff to specified height.  
Arguments:  
* `height` - takeoff height in m.  
* `speed` - copter vertical speed in m/s.  
* `tolerance` - tolerance of reaching height in m.  
* `frame_id` - copter will takeoff relative to this frame id.

```bash
reach_point(x, y, z, yaw=float('nan'), speed=SPEED, tolerance=TOLERANCE, frame_id=LOCAL_FRAME_ID)
```

Reach specified point. Copter needs to be armed. Heading will be the same as initial if `yaw`=`float('nan')`.  
Arguments:  
* `x`, `y`, `z` - coordinates in m.  
* `yaw` - copter heading in radians.  
* `speed` - copter speed in m/s.  
* `tolerance` - tolerance of reaching point in m.  
* `frame_id` - copter will fly to point relative to this frame id.

```bash
create_route(filename, add_trigger, stop_trigger, frame_id=LOCAL_FRAME_ID)
```

Create route file in .csv format with sequence of points with `x`, `y`, `z` coordinates.  
Arguments:  
* `filename` - file name of the route.  
* `add_trigger` and `stop_trigger` - external controlled `threading.Event` variables for add point and end creating operations.  
* `frame_id` - coordinates will be recorded relative to this frame.

```bash
read_route(filename)
```

Read route file. Return array of points. Each point is a dictionary: `{'x': float(x),'y': float(y),'z': float(z)}`.

```bash
def fly_route(route, flight_function=reach_point, z = float('nan'), delay = 0.1, speed=SPEED, frame_id=LOCAL_FRAME_ID)
```

Fly the route.  
Arguments:  
* `route` - array of points, each point is a dictionary: `{'x': float(x),'y': float(y),'z': float(z)}`.
* `flight_function` - function that is used to fly to point. Variants are: `navigate`, `set_position` or `reach_point`.
* `z` - specified height of flight in m. If `z` is `float('nan')` then the height will be `['z']` value in each point. 
* `delay` - delay between flying to points in s. Useful with `navigate` or `set_position` flight sunctions.  
* `speed` - speed of flight in m/s.  
* `frame_id` - copter will fly to point relative to this frame.


