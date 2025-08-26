# Pure Pursuit

A brief introduction to the pure pursuit algorithm

---

## 🎯 Goal
To accurately and smoothly follow a given path (`nav_msgs/msg/Path`) by calculating the required steering angle.

---

## 🧠 Core Concept
Pure Pursuit is a **geometric controller** works like a "carrot on a stick."
1.  The controller finds the point on the path that is a specific **lookahead distance** away from the car's current position. This is the "goal point."
2.  It calculates the **curvature of the arc** that connects the car's rear axle to this goal point.
3.  This curvature is then converted into a **steering angle command** using the kinematic bicycle model equation:

    $$
    \delta = \arctan\left(\frac{2L \sin(\alpha)}{l_{d}}\right)
    $$
    Where:
    * $\delta$ is the steering angle.
    * $L$ is the car's wheelbase.
    * $\alpha$ is the angle between the car's heading and the lookahead point.
    * $l_{d}$ is the lookahead distance.

---

## 🛠️ Parameters & Tuning
The most critical parameter is the **lookahead distance**.

* **Small Lookahead:** Results in a very aggressive, twitchy response. The car will try to cut corners tightly but may become unstable at high speeds.
* **Large Lookahead:** Results in a very smooth, stable response. The car will cut corners less, taking a wider arc. This is safer at high speeds but may be slower overall.
* **Adaptive Lookahead:** A common strategy is to make the lookahead distance proportional to the car's velocity. This gives us stability at high speeds and agility at low speeds.

---

## Resources & Further Learning 📚

* [Original Pure Pursuit Paper](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) - Please go through this paper to understand the geometry, and how the equations are derived
* [Adaptive Pure Pursuit](https://arxiv.org/abs/2111.08873) - A modified version of Pure Pursuit
    
