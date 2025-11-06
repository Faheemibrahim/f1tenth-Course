# Time to Collision (TTC) — Lidar Geometry Explanation

## Variables

- **\( \dot{r} \)** → Lidar range rate (rate of change of distance to the object)  
- **\( V_x \)** → Velocity of the vehicle (along the x-axis)  
- **\( \theta \)** → Angle between the vehicle's velocity vector and the Lidar beam  

---

## Relationship Derivation

From the geometry:

\[
\dot{r} = V_x \cos(\theta)
\]

This represents the **radial component** of the vehicle's velocity along the Lidar beam.

Therefore:

\[
\cos(\theta) = \frac{\dot{r}}{V_x}
\]

---

## Time to Collision (TTC)

If the Lidar directly measures both the **distance** \( r \) to the obstacle a
