# Rocket Control Simulation
This project implements a nonlinear, constrained optimal control simulation for mass-varying rocket dynamics.


# The Problem
Consider a rocket whose engine can tilt a fixed amount, for example $\phi \in [-10\degree, 10\degree]$. Assume we can control the thrust $T$ as well as the gimbal angle $\phi$. 
Given some initial conditions for the state vector, find a control that lands the rocket at the target with zero velocity and in an upright position.

![Rocket Sketch](https://github.com/sseso/rocket-control/blob/main/showcase/rocket_sketch.jpg)

## Approach
The simulation was built in three steps:
1. The 1-dimensional problem: Find a control that lands a falling rocket on the ground with zero speed.
2. Rotational dynamics: Allow the thrusters to gimbal $\pm 10 \degree$. Find a control sequence that redirects the rocket from an initial angle $\theta_0$ to a target angle $\theta_t$ in minimal time.
3. The full, 2-dimensional problem (see above).
