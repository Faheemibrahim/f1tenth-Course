# Lab 3 – Wall Following Notes

This summarizes the core math and control in `wall_following.py` so you can quickly tune and explain it.

## 1) Formulas

- Angle to wall: `alpha = atan((b − a·cos(theta)) / (a·sin(theta)))`
- Perpendicular distance: `D_t = b·cos(alpha)`
- Lookahead projection: `D_t+1 = D_t + l·sin(alpha)`
- Error (lateral): `e = desired_distance − D_t+1`
- PID terms and output:
  - `P = kp · e`
  - `I = ki · clamp(integral + e·dt, −wind_up, wind_up)`
  - `D = kd · (e − prev_error) / dt`
  - `PID = P + I + D`
- Steering command: `steering_angle = clip(PID, −1, 1) · max_steering_angle`
- Speed policy:
  - if `|steering_angle| < 10°` → `speed = 1.5`
  - else if `|steering_angle| < 20°` → `speed = 1.0`
  - else → `speed = 0.5`

## 2) Variables

- `a` (`a_distance`): LIDAR range at `a_angle = 60°`
- `b` (`b_distance`): LIDAR range at `b_angle = 20°`
- `theta = a_angle − b_angle = 40°` (fixed)
- `alpha`: vehicle orientation relative to wall normal
- `D_t`: current perpendicular distance to wall
- `D_t+1`: projected distance using lookahead `l`
- `e`: lateral error used by PID
- `l`: lookahead distance (default `0.1 m`)
- `desired_distance`: target distance to wall (default `1.0 m`)
- `kp, ki, kd`: PID gains (defaults `2.5, 0.1, 0.1`)
- `wind_up`: integral clamp limit (`±1.0`)
- `dt`: loop timestep (clipped to `[0.005, 0.05] s`; first loop uses `0.004 s`)
- `max_steering_angle`: `0.4189 rad`
- `speed`: chosen from steering magnitude via policy above

## 3) Relationships

- Geometry from two rays (`a` at 60°, `b` at 20°) estimates wall orientation (`alpha`) and distance (`D_t`).
- Lookahead `l` forms a forward-looking error `e = desired − D_t+1` to reduce oscillations.
- PID maps `e` to steering; output is clipped to `[−1, 1]` then scaled by `max_steering_angle`.
- Larger turns (|steering|) imply lower speed for stability and safety.

## Quick Example

Given: `a = 2.0 m` (60°), `b = 1.5 m` (20°), `l = 0.1 m`, `desired = 1.0 m`, `kp=2.5`, `ki=0.1`, `kd=0.1`, `prev_error=0`, `dt=0.01 s`.

- `theta = 40°`, `cos40≈0.7660`, `sin40≈0.6428`
- `alpha ≈ atan((1.5 − 2·0.7660) / (2·0.6428)) ≈ −0.0249 rad`
- `D_t ≈ 1.5·cos(−0.0249) ≈ 1.4995 m`
- `D_t+1 ≈ 1.4995 + 0.1·sin(−0.0249) ≈ 1.4970 m`
- `e ≈ 1.0 − 1.4970 = −0.497`
- `P ≈ −1.243`, `I ≈ −0.0005`, `D ≈ −4.97` → `PID ≈ −6.21` → clip to `−1`
- `steering_angle ≈ −0.4189 rad` (code sends `−steering`, so command is `+0.4189 rad`)
- `|steering| ≈ 24°` → `speed = 0.5 m/s`


