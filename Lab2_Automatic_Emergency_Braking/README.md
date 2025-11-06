# basic Time-to-Collision (TTC) — Lidar Geometry

$$ iTTC=\frac{r}{\lbrace- \dot{r}\rbrace_{+}} $$

## Variables

- `r` → Distance to obstacle (m)  
- `r_dot` → LiDAR range rate (rate of change of distance to the object) (m/s)  
- `Vx` → Vehicle velocity along the x-axis (m/s)  
- `theta` → Angle between the vehicle velocity vector and the LiDAR beam (radians)

---
## Relationship

<p align="center">
  <img src="https://github.com/Faheemibrahim/f1tenth-Course/blob/main/Lab2_Automatic_Emergency_Braking/trig.png" alt="Lidar Geometry" width="500"/>
</p>

From geometry (using Soh/Cah/Toa), the radial component of the vehicle velocity along the Lidar beam is:

`cos(theta) = r_dot / Vx`    

Equivalently:

`r_dot = Vx * cos(theta)`

---

## Quick Example

Given `r = 10 m`, `Vx = 5 m/s`, and `theta = 0°`:

- `r_dot = 5 * cos(0°) = 5 m/s` → moving away, no collision (TTC = ∞).

Given `r = 10 m`, `Vx = 5 m/s`, and `theta = 180°`:

- `r_dot = 5 * cos(180°) = -5 m/s`
- `TTC = 10 / 5 = 2 s` → (closing in on object)
  
- `TTC is only allowed to be postive to take it as object is aproaching`
  
---

## ROS Interface

### Parameters
<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/lidar.png" alt="Lidar Example" width="500"/>
</p>

- `min_range`→(float, meters): minimum valid Lidar range to consider for TTC.
- `angle_min`→(starting angle of the Lidar scan *(in radians)*)
- `angle_max`→(ending angle of the Lidar scan *(in radians)*)  
- `angle_increment`→(angular spacing between consecutive beams) 
- `ttc_threshold`→(float, seconds): brake when `iTTC < ttc_threshold`.
- `speed_topic`→(string): topic name for current longitudinal speed (optional).

### Subscribers
- `/scan` (`sensor_msgs/LaserScan`): primary Lidar input used to compute `r` and `r_dot` per beam.
- `/odom` (`nav_msgs/Odometry`) or `/vesc/odom` (optional): provides `Vx` if needed.

### Publishers
- `/drive` (`ackermann_msgs/AckermannDriveStamped`): command with reduced speed/zero throttle when braking.

---



