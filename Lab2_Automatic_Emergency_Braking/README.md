# Time-to-Collision (TTC) — Lidar Geometry

## Variables

- `r_dot` → Lidar range rate (rate of change of distance to the object)
- `Vx` → Vehicle velocity along the x-axis
- `theta` → Angle between the vehicle velocity vector and the Lidar beam

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
