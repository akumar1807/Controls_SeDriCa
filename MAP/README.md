# Model and Acceleration Based Pure Pursuit Controller (MAP)

This package contains our  **Model and Acceleration Based Pure Pursuit** implementation. This is a computationally inexpensive controller that combines the best of both worlds - MPC and Pure Pursuit!

---

## 🎯 Goal
To generate a Lookup Table using a data driven vehicle dynamics model that uses an artificial neural network to solve for the parameters of the Pacejka Magic Formula. (A lot of technical jargon, sorry!) 

---

## 🧠 Core Concept
The algorithm is similar regular pure pursuit. However, instead of using the equation given before to solve for steering angle, we incorporate the effects of tire slipping at high speeds and calculate required steering angle from a relation we derive between itself and the car's lateral acceleration.

### Deriving the Relation - Pacejka Tire Equation
The Pacejka Tire Equation is a method to model a car's tires, and is an equation between the slip angles and the lateral forces on the tire.

It has 6 parameters that need to be solved for, which is done by training a **neural network** on data collected from the car while driving and using it to make predictions on **new simulated data**.

The simulated data is generated using the known vehicle dynamics equations of the car, and the predictions are tabulated to form our **Lookup Table (LUT)**

---

## 🛠️ Parameters & Tuning
Key parameters include:

* **Neural Network Parameters** - These parameters affect the overall performance of your model, so tuning them is an important part of the algorithm
* **Lookahead Distance**

---

## Resources & Further Learning 📚

* **Pacejka Equation + Neural Networks**
    * [Artificial Neural Networks](https://drive.google.com/file/d/1CH2COTyg7saKnflgvEpczHVko_AzWhzQ/view?usp=sharing) - This paper explains how artificial neural networks can be used for our specific case
    * [ForzaETH Implementation](https://arxiv.org/html/2411.17508v1) - Implementation in ROS by ForzaETH
    * [SeDriCa Implementation](https://github.com/akumar1807/on_track_sysid) - Implementation in ROS2 by SeDriCa 
    * [Neural Networks](https://www.youtube.com/watch?v=mlk0rddP3L4&list=PLuhqtP7jdD8CftMk831qdE8BlIteSaNzD) - Helpful playlist for learning about neural networks

* **MAP Algorithm**
    * [ForzaETH Research Paper](https://arxiv.org/abs/2209.04346) - Implementation by ForzaETH, also contains the link to the Github repo for the algorithm
    