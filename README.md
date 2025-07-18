# Optimal Control Projects
This repository contains four Optimal Control projects developed during my Master's studies in Control Engineering at K. N. Toosi University of Technology under the supervision of Dr. Bijan Moaveni. These projects demonstrate classical and advanced optimal control techniques applied to various dynamic systems, including mechanical and aerospace systems.

# Authors
Ali Eidizadeh — ali80ei@gmail.com

Amirhadi Keyvan — amirhadikeyvan@gmail.com

# Projects Overview

## 1-LQR Quarter Car Suspension System
This project focuses on designing and simulating Linear Quadratic Regulator (LQR) controllers for a quarter-car suspension model. The goal is to improve ride comfort and road handling by optimally balancing state regulation and control effort. Both continuous and discrete-time LQR controllers are implemented and compared.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸System modeled as a quarter-car with two states: body displacement and velocity.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Performance analyzed via simulation results showing suspension response.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸MATLAB code for LQR design and simulation included.

## 2- Finite-Time Linear Quadratic Tracking (LQT) Controller for Mass-Spring-Damper System
In this project, a finite-time LQT controller with integral action is designed for a two-mass-spring-damper mechanical system. The controller tracks a constant reference position within a fixed time horizon while minimizing a quadratic cost function.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸State-space model of coupled masses with damping and stiffness.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Riccati differential equation solved backward in time for finite horizon.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Integral action included to eliminate steady-state error.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Simulation results illustrate output tracking performance.

## 3-Discrete Infinite-Time Tracking LQT for a Linear Quadcopter System
This project designs a discrete-time infinite horizon LQT controller to track predefined trajectories for a linearized quadcopter model. Deviation variables are used to improve control precision.
12-state linearized quadcopter model including positions, velocities, and Euler angles.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Reference trajectories defined as smooth time-varying functions.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Discrete-time LQR gain computed using dlqr.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Simulation shows tracking of 3D trajectories and controller performance.

## 4-Discrete-Time LQG Controller with Kalman Filter for a Linear Quadcopter
Extending the previous quadcopter controller, this project implements a Linear Quadratic Gaussian (LQG) controller that combines an LQR state-feedback controller with a steady-state Kalman filter for state estimation under process and measurement noise.
White Gaussian noise models process disturbances and sensor noise.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Kalman filter designed using discrete algebraic Riccati equation.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Closed-loop simulation with noisy measurements.

## 5-Dynamic Programming
This project demonstrates the application of Dynamic Programming (DP) to a constrained discrete-time optimal control problem. The main goal is to determine an optimal sequence of control inputs that minimizes a quadratic cost function while respecting state and input constraints.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸State estimates compared with true states, demonstrating estimator performance.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;🔸Control inputs stabilize the quadcopter along desired trajectories.
