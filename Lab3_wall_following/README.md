# Lab 3 – Wall Following with PID


<p align="center">
  <img src="https://github.com/Faheemibrahim/f1tenth-Course/blob/main/Lab3_wall_following/Screenshot%20from%202025-11-14%2003-08-37.png" alt="Lidar Geometry" width="500"/>
</p>


---
## Section 1 Overview 
***This section provides a high-level explanation of how wall following works. 
We use LiDAR readings to estimate the car’s orientation and distance from the wall,
which form the inputs to the PID controller.***

---
# Wall Following

| **Figure 1 — Lidar Geometry** | **Figure 2 — Future Projection** |
|-------------------------------|----------------------------------|
| <img src="https://github.com/f1tenth/f1tenth_lab3_template/blob/main/img/wall_following_lab_figure_1.png" width="360"/> | <img src="https://github.com/Faheemibrahim/f1tenth-Course/blob/main/Lab3_wall_following/future_distance.png" width="360"/> |

---
To follow a wall, the car must know two things:

1. ***How it is angled relative to the wall***  
2. ***How far it is from the wall***

We use two fixed LiDAR rays:

- `a` at **60°**  
- `b` at **20°**

Since these angles never change:

$$ \theta = 60^\circ - 20^\circ = 40^\circ $$

Using these readings, we compute:

1. `alpha` (car angle)  
2. `D_t` (current distance)  
3. `D_{t+1}` (future distance)  
4. `error` (value sent to PID)

---

# Step-by-Step Breakdown

## **1. Compute `alpha` — car orientation (Figure 1)**

$$
\alpha = \arctan\left(\frac{b - a\cos\theta}{a\sin\theta}\right)
$$

**Meaning:**  
`alpha` tells us ***how much the car is rotated*** toward or away from the wall.

---

## **2. Compute `D_t` — current distance to the wall (Figure 1)**

$$
D_t = b \cdot \cos(\alpha)
$$

**Meaning:**  
`D_t` is the ***current perpendicular distance*** from the car to the wall.

---

## **3. Compute `D_{t+1}` — predicted distance ahead (Figure 2)**

$$
D_{t+1} = D_t + L \cdot \sin(\alpha)
$$

**Meaning:**  
`D_{t+1}` is the ***future distance*** to the wall after the car moves forward a small lookahead `L`.

---

## **4. Compute `error` — value sent to PID (Figures 1 & 2)**

$$ error = {desired \ distance} - D_{t+1} $$

**Meaning:**  
- If **`error > 0`** → ***car is too close***, steer away  
- If **`error < 0`** → ***car is too far***, steer toward the wall  

This final `error` is what the PID controller uses.

---
## Section 2 Overview 
***This section explains how the PID controller is used for wall following.
Robots don’t calculate continuously; instead, they update their controller at a fixed rate.
Because of this, we use discrete time (DT), where the PID formula is computed once every control loop, using the latest sensor data.
We first look at the PID equation in continuous time, then show how it becomes simpler and more practical when converted to discrete time for real robots.***

---

## 1. PID (Continuous Time) formula

$$ u(t) = K_p e(t) + K_i \int_0^t e(t') dt' + K_d \frac{d}{dt}(e(t)) $$

Where:
- **`u(t)`** is the control output.
- **`e(t)`** is the error (difference between desired and measured value).
- **`Kp`**, **`Ki`**, **`Kd`** are the PID gains for the **proportional**, **integral**, and **derivative** terms.
- In discrete time we will refer to the corresponding terms as **`P`**, **`I`**, and **`D`**, which are built from `Kp`, `Ki`, and `Kd`.

---

## 3. PID (Discrete Time) formula Breakdown

### **`dt` (Time Step)**

**`dt`** is the time between each PID update.  
It comes from the control loop frequency:

$$ dt = \frac{1}{\text{Hz}} $$

For example, if the controller runs at **250 Hz**:

$$ dt = \frac{1}{250} = 0.004\ \text{seconds} \ (4\ \text{ms}) $$

In simple terms:

> **`dt` tells the PID controller how much time passed since the last update.**

---

### **`P` (Proportional Term using `Kp`)**

$$ P = K_p \cdot error $$

The proportional term provides an **instant reaction** to the current error. It answers the question:

> *"How far am I from where I should be right now?"*

- A small error → small correction.  
- A large error → large correction.

In wall following: if the robot is too close or too far from the wall, **`Kp` directly adjusts the steering angle** to push the robot back toward the target distance.

However:
- If **`Kp` is too high**, the robot may oscillate aggressively or overcorrect.  
- If **`Kp` is too low**, the robot reacts too slowly and drifts.

---

### **`I` (Integral Term using `Ki`)**

**(`I_prev` = previous errors accumulated)**

$$ I = I_{prev} + K_i \cdot error \cdot dt $$

The integral term looks at **past errors accumulated over time** and answers:

> *"Have I been consistently off in one direction for a long time?"*

It adds up small errors over time:
- If the robot stays slightly too far from the wall, `Ki` will eventually push the robot closer.
- It **eliminates steady-state error**, which `Kp` alone cannot fix.

However, the danger:
- If the error persists for too long (e.g., when the steering is already maxed out), the integral grows excessively.
- This creates **overshoot, oscillation, and instability**.
- This is known as **integral windup**.

---

### **`D` (Derivative Term using `Kd`)**

$$ D = K_d \cdot \frac{error - (previous.error)}{dt} $$

The derivative term focuses on **the rate of change of error over time**. It answers:

> *"Am I approaching the wall too fast?"*

It predicts future error based on recent trend:
- If the robot approaches the wall quickly, `D` applies a counter-steering action.
- If the robot turns away quickly, `D` softens the correction to avoid jerky movement.

`Kd` acts like a **damping force**, helping the robot:
- Stabilize turns  
- Reduce oscillations  
- Produce smoother steering  

In short:
- **`Kp`** sets the strength of the **`P`** term (drives the robot toward the correct distance).  
- **`Ki`** sets the strength of the **`I`** term (fixes long-term bias).  
- **`Kd`** sets the strength of the **`D`** term (prevents overshoot and smooths the motion).  

---

## 4. Anti-Windup (Integral Limiting)

The **integral windup** problem occurs when the integral term accumulates too much error while the robot is already at the limit of its steering capability.

### Example:
- Robot keeps turning left at its max steering angle.  
- Error stays large.  
- `Ki` continues accumulating.  
- System becomes unstable.

---
## 5. Steering Output, Normalization & Speed Control

After computing the PID term:

$$
\text{PID} = P + I + D
$$

---

### **Clipping the `PID` Output**

Clipping ensures the `PID` output stays within a safe, normalized range:

$$
-1 \le \text{PID} \le 1
$$

******Why clipping?******

- Prevents extreme steering commands when the error suddenly spikes.  
- Protects the vehicle from making violent, unsafe turns.  
- Ensures the `PID` output remains compatible with a normalized control pipeline.  
- Creates a stable and predictable steering response.

---

### **Normalization & Scaling to the Maximum Steering Angle**

Once clipped, the `PID` output is normalized, meaning:

- `-1` represents **maximum left** turn  
- `0` represents **straight**  
- `1` represents **maximum right** turn

The normalized `PID` value is then mapped to the car’s mechanical steering limits.


> ******What scaling?******

If the car’s mechanical steering range is ±`0.4` radians:

- `PID = 1`  → `+0.4 rad` (full right)  
- `PID = -1` → `-0.4 rad` (full left)  
- `PID = 0`  → `0 rad` (straight)

> ******Why scaling?******

- The `PID` controller works in a generic, unitless range.  
- The car’s steering operates in real units (radians).  
- Scaling maps the controller output to actual hardware limits.  
- Ensures consistency between the `PID` controller and the vehicle’s actuator limits.

---

### **Speed Adjustment Based on Steering Angle**

Speed is determined by the absolute steering angle `abs_sa` (in degrees). Instead of fixed values, use **ranges** to allow smoother policy tuning:

`|abs_sa| < 10°`  -> speed `1.5 m/s`  
`10° <= |abs_sa| < 20°` -> speed `1.0 m/s`  
`|abs_sa| >= 20°` -> speed `0.5 m/s`

> ***Why adjust speed?***

> - High speeds with sharp steering can lead to instability or drifting.  
> - Lower speeds during tighter turns keep the car controlled and safe.  
> - Ranges allow **adaptive** tuning (by environment, grip, latency) without changing code.

---

### **Trigger Service & Automatic PID Graph Generation**

The system includes a ******trigger service****** for logging and analysis.  
When triggered, the car will:

1. ******Stop immediately******  
2. ******Save PID logs******  
3. ******Generate graphs automatically******

**Generated Graphs:**
- `P` term  
- `I` term  
- `D` term  
- Total `PID` output  
- Error over time

**How to Call the Trigger Service (ROS2):**
```bash
ros2 service call /pid_trigger std_srvs/srv/Trigger "{}"
```









