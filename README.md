# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

Project details can be found from [Udacity CarND-MPC-Project](https://github.com/udacity/CarND-MPC-Project)

## Main Goal
Implement a MPC control to drive a car in simulator

### Inputs from Simulator
* Car global position (x, y)
* Car global view points for guiding the car directions
* Car current facing angle
* Car current speed

### MPC Outputs
* Steering Angle
* Throttle Value

### MPC State
* car location x (in car's coordinates)
* car location y (in car's coordinates)
* vehicle orientation
* cross track error 
* orientation error

### Kinematic Model
* State:
```
 [location x (x), location y (y), vehicle orientation (psi),
  speed (v), cross track error (cte), orientation error (epsi)]
```

* Actuators: [ steering angle (delta), throttle from -1 to 1 (a)]

* Formula:
```
 dt = time changes
 Lf = distance between the front of hte vehicle and its center of gravity
 x = x + v * cos(psi) * dt
 y = y + v * sin(psi) * dt
 psi = psi + v / Lf * delta * dt
 v = v + a * dt
 cte = f(x) - y + (v * sin(epsi) * dt)
 psides = desired orienation = arctan(f'(x))
 espi = psi - psides + v / Lf * delta * dt       
```

### Latency
* 100 milli seconds

### Adjustment for Latency
* Predict the car state 100 milli secods later in car's cooorinates
```
  l_dt  = 0.1;   /// latency equivalent to 100ms.
  Lf = 2.67;
  car_px = v * l_dt;  /// car x location after latency adjustment
  car_py = 0;         /// car y location
  car_psi = -v * steer_value / Lf * l_dt;   /// car orientation
  car_v = v + throttle_value * l_dt;
```          

### Others
* Please refer to comments in the source code!
