# Lab 3 – Wall Following with PID
---

## Section 1 Overview 
*** ***

---
Wall following 





## Section 2 Overview 
***This section explains how the PID controller is used for wall following.
Robots don’t calculate continuously instead, they update their controller at a fixed rate.
Because of this, we use discrete time (DT), where the PID formula is computed once every control loop, using the latest sensor data.
We first look at the PID equation in continuous time, then show how it becomes simpler and more practical when converted to discrete time for real robots.***

---

## 1. PID (Continues Time) formula

$$ u(t) = K_p e(t) + K_i \int_0^t e(t') dt' + K_d \frac{d}{dt}(e(t)) $$

Where:
- **`u(t)`** is the control output.
- **`e(t)`** is the error (difference between desired and measured value).
- **`Kp`**, **Ki**, **Kd** are the PID gains.

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

### **`P`  (Proportional Term)**

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

### **I (Integral Term)**

**(`I_prev` = previous errors accumulated)**

$$ I = I_{prev} + K_i \cdot error \cdot dt $$

The integral term looks at **past errors accumulated over time** and answers:

> *"Have I been consistently off in one direction for a long time?"*

It adds up small errors over time:
- If the robot stays slightly too far from the wall, `Ki` will eventually push the robot closer.
- It **eliminates steady-state error**, which Kp alone cannot fix.

However, the danger:
- If the error persists for too long (e.g., when the steering is already maxed out), the integral grows excessively.
- This creates **overshoot, oscillation, and instability**.
- This is known as **integral windup**.

---

### **D (Derivative Term)**

$$ D = K_d \cdot \frac{error - (previous.error)}{dt} $$

The derivative term focuses on **the rate of cahange of error over time**. It answers:

> *"Am I approaching the wall too fast?"*

It predicts future error based on recent trend:
- If the robot approaches the wall quickly, `D` applies a counter-steering action.
- If the robot turns away quickly, `D` softens the correction to avoid jerky movement.

`Kd` acts like a **damping force**, helping the robot:
- Stabilize turns  
- Reduce oscillations  
- Produce smoother steering  

In short:
- **`Kp`** drives the robot toward the correct distance.  
- **`Ki`** fixes long-term bias.  
- **`Kd`** prevents overshoot and smooths the motion.  

---

## 4. Anti-Windup (Integral Limiting)
The **integral windup** problem occurs when the integral term accumulates too much error while the robot is already at the limit of its steering capability.

### Example:
- Robot keeps turning left at its max steering angle.  
- Error stays large.  
- `Ki`continues accumulating.  
- System becomes unstable.


---









