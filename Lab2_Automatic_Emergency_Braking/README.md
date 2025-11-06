# Time-to-Collision (TTC) — Lidar Geometry

## Variables

- `r_dot` → Lidar range rate (rate of change of distance to the object)
- `Vx` → Vehicle velocity along the x-axis
- `theta` → Angle between the vehicle velocity vector and the Lidar beam

---

## ROS Interface

### Parameters
- `min_range` (float, meters): minimum valid Lidar range to consider for TTC.
- `max_range` (float, meters): maximum valid Lidar range to consider for TTC.
- `ttc_threshold` (float, seconds): brake when `iTTC < ttc_threshold`.
- `speed_topic` (string): topic name for current longitudinal speed (optional).

### Subscribers
- `/scan` (`sensor_msgs/LaserScan`): primary Lidar input used to compute `r` and `r_dot` per beam.
- `/odom` (`nav_msgs/Odometry`) or `/vesc/odom` (optional): provides `Vx` if needed.

### Publishers
- `/brake_bool` (`std_msgs/Bool`): true when braking is commanded (iTTC below threshold).
- `/drive` (`ackermann_msgs/AckermannDriveStamped`): command with reduced speed/zero throttle when braking.

Notes:
- Clamp ranges to `[min_range, max_range]` before computing iTTC.
- Ignore beams with invalid/NaN/Inf ranges.

---

## Relationship

From geometry, the radial component of the vehicle velocity along the Lidar beam is:

`r_dot = Vx * cos(theta)`

Equivalently:

`cos(theta) = r_dot / Vx`

---

## Time to Collision (TTC)

If the Lidar (or perception stack) provides the current distance `r` to the obstacle and the range rate `r_dot`:

- When closing in on the obstacle, `r_dot` is negative. A common definition is:
  - `TTC = r / (-r_dot)` for `r_dot < 0`
- If `r_dot >= 0` (not closing), TTC is undefined or treated as infinite.

Notes:
- Ensure `r` and `r_dot` use the same units (e.g., meters and meters/second).
- Clamp or filter noisy `r_dot` to avoid spurious small denominators.
- Apply a minimum TTC threshold to trigger braking in practice (e.g., `TTC < 1.5 s`).

---

## Quick Example

Given `r = 10 m`, `Vx = 5 m/s`, and `theta = 0°`:

- `r_dot = 5 * cos(0°) = 5 m/s` → moving away, no collision (TTC = ∞).

Given `r = 10 m`, `Vx = 5 m/s`, and `theta = 180°` (closing):

- `r_dot = 5 * cos(180°) = -5 m/s`
- `TTC = 10 / 5 = 2 s`

---

## Instantaneous TTC (iTTC)

As discussed in the lecture, the instantaneous TTC is

$$ iTTC = \frac{r}{\{ -\dot{r} \}_+} $$

where the positive-part operator is defined as `\{x\}_+ = max(x, 0)`.

Plain-text equivalent for implementation:

- `iTTC = r / max(-r_dot, 0)`
- If `-r_dot <= 0` (i.e., `r_dot >= 0`), treat `iTTC` as infinite (no closing).

Implementation notes:
- Consider a small epsilon in the denominator to avoid division by zero in code.
- Apply filtering to `r_dot` to reduce noise before computing `iTTC`.
