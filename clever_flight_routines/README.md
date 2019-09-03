# clever_flight_routines

ROS package and python module which simplifies the usage of [simple_offboard](https://clever.copterexpress.com/ru/simple_offboard.html)

## Python module API description

### Globals

```bash
TOLERANCE = 0.2             # m
SPEED = 1.0                 # m/s
TAKEOFF_SPEED = 1.0         # m/s
TAKEOFF_HEIGHT = 1.0        # m
LOCAL_FRAME_ID = 'map'
```

### Service proxies

Most services described [here](https://clever.copterexpress.com/ru/simple_offboard.html)

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

### Functions

#### get_distance

Return distance betwen 2 points: `(x1,y1,z1)` and `(x2,y2,z2)`.

Arguments:

* x1, y1, z1, x2, y2, z2 - coordinates in the same metric system

#### takeoff

Takeoff to specified height. 

Arguments:  
* `height` - takeoff height in m. Default is TAKEOFF_HEIGHT. 
* `speed` - copter vertical speed in m/s. Default is TAKEOFF_SPEED.
* `tolerance` - tolerance of reaching height in m. Default is TOLERANCE. 
* `frame_id` - copter will takeoff relative to this frame id. Default is LOCAL_FRAME_ID.

#### reach_point

Reach specified point. Copter needs to be armed. Heading will be the same as initial if `yaw`=`float('nan')`.

Arguments:  
* `x`, `y`, `z` - coordinates in m.  
* `yaw` - copter heading in radians. Default is float('nan').  
* `speed` - copter speed in m/s. Default is SPEED.
* `tolerance` - tolerance of reaching point in m. Default is TOLERANCE.
* `frame_id` - copter will fly to point relative to this frame id. Default id LOCAL_FRAME_ID.

#### create_route

Create route file in .csv format with sequence of points with `x`, `y`, `z` coordinates.

Arguments:  
* `filename` - file name of the route.  
* `add_trigger` and `stop_trigger` - external controlled `threading.Event` variables for add point and end creating operations.  
* `frame_id` - coordinates will be recorded relative to this frame. Default is LOCAL_FRAME_ID.

#### read_route

Read route file. Return array of points. Each point is a dictionary: `{'x': float(x),'y': float(y),'z': float(z)}`.

#### fly_route

Fly the route.

Arguments:  
* `route` - array of points, each point is a dictionary: `{'x': float(x),'y': float(y),'z': float(z)}`.
* `flight_function` - function that is used to fly to point. Variants are: `navigate`, `set_position` or `reach_point`. Default is `reach_point`.
* `z` - specified height of flight in m. If `z` is `float('nan')` then the height will be `['z']` value in each point. Default is `float('nan')`. 
* `delay` - delay between flying to points in s. Useful with `navigate` or `set_position` flight sunctions. Default is 0.1 s.  
* `speed` - speed of flight in m/s. Default is SPEED.
* `frame_id` - copter will fly to point relative to this frame. Default is LOCAL_FRAME_ID.
