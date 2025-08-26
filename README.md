# SeDriCa - Controls Subsystem Onboarding 🏎️💨

Welcome to the Controls Subsystem! Our mission is to make the car follow the fastest possible path with precision, stability, and aggression. This is where the magic happens—turning a planned trajectory into real-world motion by commanding the steering and throttle.

This guide will walk you through the core concepts, our software stack, and your first steps to becoming a controls wizard. Let's get started!

## Table of Contents
1.  [The Big Picture: How Our Car Thinks](#the-big-picture-how-our-car-thinks)
2.  [Core Concepts: A Control Theory Primer](#core-concepts-a-control-theory-primer)
    * [Vehicle Models: Kinematic vs. Dynamic](#vehicle-models)
    * [Controller Showdown: From PID to MPC](#controller-showdown)
3.  [Our Software Stack & Getting Started](#our-software-stack--getting-started)
    * [Key ROS2 Topics](#key-ros2-topics)
    * [How to Build and Run the Simulation](#how-to-build-and-run)
4.  [Resources & Further Learning](#resources--further-learning)

---

## The Big Picture: How Our Car Thinks

The control system is the final link in the autonomy chain. It doesn't decide *where* to go, but *how* to get there. The controller's job is to receive the planned path and the car's current state (position, velocity) and calculate the precise steering angle ($\delta$) and throttle/speed ($v$) commands to follow that path as closely as possible. Our work directly impacts lap times and the car's ability to handle tight corners and high speeds.

---

## Core Concepts: A Control Theory Primer

To write good controllers, you need to understand the theory behind them.

### Vehicle Models

We can't control what we can't model. A mathematical model of the car helps us predict how it will react to our commands.

#### 1. The Kinematic Bicycle Model
This is our workhorse model. It's simple, computationally cheap, and works very well at moderate speeds where tire slip is negligible. It treats the car as a bicycle with two wheels.



The state of the car is typically represented by its position ($x, y$) and heading ($\psi$). The state equations are:

$$
\dot{x} = v \cos(\psi) \\
\dot{y} = v \sin(\psi) \\
\dot{\psi} = \frac{v}{L} \tan(\delta)
$$

Where:
* $v$ is the velocity.
* $L$ is the wheelbase (distance between front and rear axles).
* $\delta$ is the steering angle (our control input).

#### 2. The Dynamic Bicycle Model
When we start pushing the limits and the tires begin to slip, the kinematic model breaks down. The dynamic model accounts for forces, inertia, and tire slip, making it more accurate for aggressive racing. It's more complex and computationally expensive. We often use this for high-fidelity simulation and advanced controllers like MPC.

**NOTE**: It is crucial to go more in-depth regarding these models, so do go through **MODULE 5** of the following playlist on [Self Driving Cars](https://www.youtube.com/playlist?list=PL05umP7R6ij321zzKXK6XCQXAaaYjQbzr)

### Controller Showdown

There are many ways to solve the path-tracking problem. We primarily use the following:

#### 1. PID Controller (Proportional-Integral-Derivative)
* **Concept:** The classic feedback controller. It calculates an error value (e.g., how far we are from the center of the track, our "cross-track error") and computes a correction based on the present error (**P**), past errors (**I**), and predicted future errors (**D**).
* **Use Case:** Excellent for simple tasks like wall-following or maintaining a constant speed. It's a great first controller to implement.

#### 2. Pure Pursuit Controller
* **Concept:** A geometric path tracker. It finds a "goal point" on the path a certain **lookahead distance** in front of the car. It then calculates the steering angle ($\delta$) required to drive the car in a perfect arc to intercept that goal point.
* **Use Case:** Very intuitive, robust, and a fantastic starting point for competitive racing. It's easy to implement and tune. Our team has a baseline Pure Pursuit controller you will work with.

#### 3. Stanley Controller
* **Concept:** Another geometric controller, famously used by Stanford's "Stanley" vehicle. It corrects for two things simultaneously:
    1.  The **heading error** relative to the path.
    2.  The **cross-track error** (perpendicular distance to the path).
* **Use Case:** Tends to be more aggressive and can converge to the path faster than Pure Pursuit.

#### 4. Model Predictive Control (MPC)
* **Concept:** The state-of-the-art for high-performance racing. MPC uses a vehicle model (like the kinematic or dynamic model) to **predict the car's future trajectory** over a short time horizon. It then solves an optimization problem to find the sequence of control inputs (steering, throttle) that minimizes a cost function (e.g., minimizes path deviation while maximizing speed) subject to constraints (e.g., track boundaries, tire friction limits).
* **Use Case:** Delivers incredibly high performance by anticipating the future, but is complex to implement and computationally demanding. This is a goal we are always working towards.

**NOTE**: This is just a brief introduction to different controllers. To learn more, please go through **MODULE 6** of the following playlist on [Self Driving Cars](https://www.youtube.com/playlist?list=PL05umP7R6ij321zzKXK6XCQXAaaYjQbzr)

---

## Our Software Stack & Getting Started

Time to get your hands dirty! 💻

Our control node is written in C++ and integrates with the ROS2 ecosystem.

**NOTE**: For quick algorithm testing, it is preferable to write your code in **Python**.

### Key ROS2 Topics

Our controller node subscribes to information and publishes commands. Here are the most important topics:

| Topic Name           | Message Type                  | Description                                            |
| -------------------- | ----------------------------- | ------------------------------------------------------ |
| `/scan`              | `sensor_msgs/msg/LaserScan`   | LiDAR data for perception (sometimes used for safety). |
| `/odom`              | `nav_msgs/msg/Odometry`       | The car's estimated position, orientation, and velocity. **Input** for our controller. |
| `/planned_path`      | `nav_msgs/msg/Path`           | The target trajectory from the planner. **Input** for our controller. |
| `/drive`             | `ackermann_msgs/msg/AckermannDriveStamped` | The steering and speed commands we publish. **Output** of our controller. |

### How to Build and Run

1.  **Clone the Repo:**
    ```bash
    cd ~/your_ros2_ws/src
    git clone <your-repo-url>
    ```

2.  **Install Dependencies:**
    ```bash
    cd ~/your_ros2_ws
    rosdep install -i --from-path src --rosdistro humble -y
    ```

3.  **Build the Workspace:**
    ```bash
    cd ~/your_ros2_ws
    colcon build --symlink-install
    ```

4.  **Run the Simulation:**
    ```bash
    source ~/your_ros2_ws/install/setup.bash
    ros2 launch f1tenth_stack sim.launch.py
    ```
    *(Note: The actual launch file name may differ.)*

---

## Resources & Further Learning 📚

* **Control Theory & Self-Driving:**
    * [Classic Control Theory](https://www.youtube.com/watch?v=oBc_BHxw78s&list=PLUMWjy5jgHK1NC52DXXrriwihVrYZKqjk) - A good introduction to how you can model systems, design controllers, and benchmark their performance.
    * [Modern Control Theory](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m)
    * [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) (Textbook)
    * [Self Driving Cars](https://www.youtube.com/playlist?list=PL05umP7R6ij321zzKXK6XCQXAaaYjQbzr)
* **F1Tenth:**
    * [F1Tenth Official Documentation](https://f1tenth.org/)
    * [F1Tenth Course Lectures (UPenn)](https://www.youtube.com/playlist?list=PLk10-T-eS-daj-J2nC8X9e-IKtG4EwW3_) - **HIGHLY RECOMMENDED**
* **ROS2:**
    * [Official ROS2 Docs Tutorials](https://docs.ros.org/en/humble/Tutorials.html) - These are specifically for **ROS2 Humble**, but the tutorials for older versions can also be found online.
