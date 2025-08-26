# Model Predictive Controller (MPC)

This package contains our high-performance **Model Predictive Control** implementation. This is an advanced, optimization-based controller designed for pushing the car to its physical limits.

**Note:** This controller is more complex and computationally expensive than other options.

---

## 🎯 Goal
To follow a trajectory while optimizing for multiple objectives (e.g., maximizing speed, minimizing error) and respecting the car's physical constraints (e.g., maximum steering angle, tire friction).

---

## 🧠 Core Concept
MPC works by repeatedly solving a finite-horizon optimal control problem. At each time step:
1.  It gets the car's current state (position, velocity, etc.).
2.  It uses a vehicle model (kinematic or dynamic) to **predict** the car's behavior over a short future time window (the "prediction horizon").
3.  It runs an optimization algorithm to find the sequence of control inputs (steering, throttle) that minimizes a **cost function** over that horizon.
4.  It applies the **first** control input from the optimal sequence to the car.
5.  The entire process repeats at the next time step.

---

## 📈 Dependencies & Setup
This controller requires a numerical optimization solver, so we will use **Acados** for our MPC implementation.

---

## 🛠️ Parameters & Tuning
MPC tuning is complex and is done in the config/mpc_params.yaml file. Key parameters include:

* Prediction Horizon (N): How many steps into the future the model predicts. A longer horizon allows for better planning but increases computation time.
* Cost Function Weights (Q & R): These matrices determine the penalty for state deviations (e.g., cross-track error) vs. control effort. Tuning these weights is key to balancing performance and smoothness.
* Constraints: Defining the physical limits of the car, such as max/min steering angle and velocity.

---

## Resources & Further Learning 📚

* **MPC Theory:**
    * [Playlist by MATLAB](https://www.youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEHww8) - Well explained introduction to MPC
    * [TUM Paper](https://arxiv.org/pdf/2109.11986)
    * [Additional videos](https://www.youtube.com/watch?v=XaD8Lngfkzk&pp=ygUQTVBDIGxhc3NlIHBldGVycw%3D%3D)

* **MPC Implementation**
    * [do-mpc](https://www.do-mpc.com/) - The best place to start learning how to implement MPC
    * [ROS2 Specific](https://github.com/kuralme/mpc_ros2) - An implementation of MPC using ROS2
    * [UPenn](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleF/lecture21.html#) - **F1Tenth** specific implementation
    