# Rocket Control Simulation
This repository contains a **direct collocation** solver (trapezoidal method + CasADi + IPOPT) for a 2D rocket landing problem with variable mass, moving center of mass, and thrust vector control via gimbal. It also includes animations and diagnostic plots for the calculated solutions.

![2D Landing Showcase GIF](https://github.com/sseso/rocket-control/blob/main/showcase/2D_landing_showcase.gif)

# The Problem
Consider a rocket whose engine can tilt a fixed amount, for example $\alpha \in [-10\degree, 10\degree]$. Assume we can control the thrust $T$ as well as the gimbal angle $\alpha$. 
Given some initial conditions for the rocket (see state vector below), find a control that lands the rocket at a specified target with zero velocity and in an upright position.

![Rocket Sketch](https://github.com/sseso/rocket-control/blob/main/showcase/rocket_sketch.jpg)

# Mathematical formulation of the control problem

### State Vector
$$
\mathbf{x}(t) = \begin{bmatrix}
x(t) \\
y(t) \\
v_x(t) \\
v_y(t) \\
\theta(t) \\
\omega(t) \\
m(t)
\end{bmatrix}
\quad
\begin{aligned}
&\text{(horizontal CoM position)} \\
&\text{(vertical CoM position)} \\
&\text{(horizontal velocity)} \\
&\text{(vertical velocity)} \\
&\text{(pitch angle from vertical)} \\
&\text{(angular velocity)} \\
&\text{(total mass)}
\end{aligned}
$$

### Control Vector
$$
\mathbf{u}(t) = \begin{bmatrix}
T(t) \\
\alpha(t)
\end{bmatrix}
\quad
\begin{aligned}
&\text{(thrust magnitude [N])} \\
&\text{(gimbal angle from body axis [rad])}
\end{aligned}
$$

### Mass-Dependent Geometry
$$
d_\text{com}(m) = \frac{m_\text{dry} \cdot d_\text{dry} + (m - m_\text{dry}) \cdot d_\text{fuel}}{m}
$$

$$
I_z(m) = I_\text{dry,cm} + m_\text{dry}(d_\text{dry} - d_\text{com}(m))^2 + I_\text{fuel,cm}(m) + (m - m_\text{dry})(d_\text{fuel} - d_\text{com}(m))^2
$$

where $I_\text{dry,cm} = \frac{1}{12} m_\text{dry} h_\text{rocket}^2$ and $I_\text{fuel,cm}(m) = \frac{1}{12} (m - m_\text{dry}) h_\text{fuel}^2$ (0 if no fuel remains).

### Dynamics
$$
\dot{\mathbf{x}}(t) = f(\mathbf{x}(t), \mathbf{u}(t)) = \begin{bmatrix}
v_x \\
v_y \\
\frac{T}{m} \sin(\theta + \alpha) \\
\frac{T}{m} \cos(\theta + \alpha) - g \\
\omega \\
-\frac{d_\text{com}(m) \cdot T}{I_z(m)} \sin(\alpha) \\
-\frac{T}{v_e}
\end{bmatrix}
$$

with $g = 9.81\,\text{m/s}^2$, $v_e = I_\text{sp} \cdot g_0$.

### Boundary Conditions
**Initial:**

```math
\mathbf{x}(0) = 
\begin{bmatrix}
x_0 \\
y_0 \\
v_{x0} \\
v_{y0} \\
\theta_0 \\
\omega_0 \\
m_0
\end{bmatrix}
```

**Terminal conditions:**

$$
\begin{aligned}
& x(t_f)          = 0, \\
& v_x(t_f)        = 0, \\
& v_y(t_f)        = 0, \\
& \theta(t_f)     = 0, \\
& \omega(t_f)     = 0, \\
& y(t_f)          = d_{\mathrm{com}}(m(t_f)), \\
& \alpha(t_f)     = 0.
\end{aligned}
$$

The condition $y(t_f) = d_\text{com}(m(t_f))$ ensures the nozzle touches the ground at touchdown.

### Path Constraints & Bounds
- $y(t) \geq 0$
- $-50 \leq v_x(t) \leq 50$ [m/s]
- $-100 \leq v_y(t) \leq 50$ [m/s]
- $-0.5 \leq \omega(t) \leq 0.5$ [rad/s]
- $m_\text{dry} \leq m(t) \leq m_0$
- Thrust: $0 \leq T(t) \leq T_\text{max}$
- Gimbal (most of flight): $|\alpha(t)| \leq 10^\circ$
- Gimbal & pitch (near landing): $|\alpha(t)| \leq 2^\circ$, $|\theta(t)| \leq 2^\circ$

### Objective (to be minimized)
$$
J = w_t \, t_f + \int_0^{t_f} L(\mathbf{x},\mathbf{u},\dot{\alpha}) \, dt + J_\text{ground}
$$

Running cost:
```math
\begin{aligned}
L &= w_\text{thrust} \, T^2
  + w_\text{gimbal} \, \alpha^2
  + w_\text{gimbal rate} \left(\frac{d\alpha}{dt}\right)^2 \\
  &\quad + w_\theta \, \theta^2
  + w_\text{alt thrust} \, T^2 \cdot \frac{y}{y_0} \\
  &\quad + w_\text{landing} \, (\theta^2 + \alpha^2) \cdot p(t)
\end{aligned}
```

where
```math
p(t) = \max\left(0, 1 - \frac{y - d_\text{com}(m)}{2\, h_\text{rocket}}\right)^3
```

Ground violation penalty:
```math
J_\text{ground} = 10^6 \int_0^{t_f} \max(0, -(y - d_\text{com}(m)))^2 \, dt
```

Typical weights used in the code:
- $w_t = 10$
- $w_\text{thrust} = 10^{-3}$
- $w_\text{gimbal} = 0.05$
- $w_\text{gimbal rate} = 0.25$
- $w_\theta = 0.5$
- $w_\text{alt thrust} = 0.01$
- $w_\text{landing} = 2000$

## Summary

$$
\begin{aligned}
\underset{\mathbf{x}(\cdot),\,\mathbf{u}(\cdot),\,t_f}{\text{minimize}}\quad
& J(\mathbf{x},\mathbf{u},t_f) \\
\text{subject to}\quad
& \dot{\mathbf{x}} = f(\mathbf{x},\mathbf{u}) \\
& \mathbf{x}(0) = \mathbf{x}_0 \\
& \text{terminal conditions (see above)} \\
& \text{path / box constraints (see above)}
\end{aligned}
$$

The problem is discretized using **trapezoidal collocation** with $N=40$ intervals ($N$ can be varied for desired precision, though computation time increases with N).


# Approach
Since solving the full problem from scratch was quite indimidating, the problem was broken down into three steps:

1. **The 1D Problem**: Consider a falling rocket in a gravitational field (only y-component, no angular deviations). Find a thrust control which lands the rocket with zero velocity.
2. **Rotational dynamics**: Consider a rocket floating in the vacuum of space, with no external forces acting on it. Now assume the engine can gimbal in a fixed range. Find a thrust & gimbal control which rotates the rocket from an initial angle $\theta_0$ to a target angle $\theta_t$.
3. **Combine the dynamics** --> Solve the full 2D Problem.

# Issues & Fixes
## SLSQP vs. CasADi / IPOPT
### SLSQP is not feasible for this kind of problem
The full 2D case is solved with a numerical solver. Initially, the simulation was implemented with scipy.optimize.minimize's SLSQP (Sequential Least SQuares Programming), which is a gradient-based optimization algorithm used to minimize a scalar function of multiple variables subject to bounds, equality, and inequality constraints. However, this algorithm was not suitable for this direct collocation problem:
- No explicit analytical gradients are given, which means SciPy is estimating (with finite differences) them by tweaking every single variable one by one. With $N=40$ collocation nodes, that's ~370 variables. That means SciPy runs the physics simulation ~371 times per iteration just to figure out which direction to step. It's also sensitive to noise; If the physics simulation has small numerical jitters, the estimated gradient can point in an entirely wrong direction, giving wrong results.
- The main way to control the behaviour of the rocket is through tuning the weights of the cost function. Large penalties on constraints (like J_ground = 1e8 * ...) create extremely steep gradients that SLSQP struggles to navigate, causing it to take tiny step sizes and run up the iteration count.
- In direct collocation, a state at node $k$ only affects node $k+1$. This creates a highly diagonal, "sparse" Jacobian matrix. SLSQP cannot recognize that most entries of the Jacobian are zero and therefore don't contribute to the next step, wasting massive amounts of memory and CPU cycles.

This caused **runtimes of 3-10 minutes for just a single simulation**, which is far too inefficient for any real use case.

### The solution: CasADi / IPOPT
The dynamics and objective were ported to CasADi (a Python library specifically built for these kinds of problems). CasADi uses Algorithmic Differentiation (exact gradients with zero finite difference overhead) and uses IPOPT, an interior-point solver designed for large, sparse non-linear programming (NLP) problems. IPOPT knows that the vast majority of entries in the Jacobian are zero and only calculates the non-zero interactions. This reduces the time complexity from $O(n^3)$ to something much closer to $\sim O(n^{1.5})$.

This **reduced the runtime to only a few seconds per simulation**. Without this change, the convergence test across a wide grid of initial conditions would not have been feasible (hours of runtime vs. minutes).

## Numerical / Convergence Issues with CasADi / IPOPT
Still, CasADi / IPOPT is not without its own issues. One main issue is numerical instability, which causes the solver to fail convergence for normally controllable initial conditions. In particular, initial conditions where the initial $x$ position and total velocity were exactly zero consistently failed to converge, which can be seen in the convergence test below.

![Reachable Set with numerical artifacts](https://github.com/sseso/rocket-control/blob/main/showcase/Reachable_Set_Buggy.png)

The reason for this is that when $x=0$, $v=0$, and $\theta=0$, the derivative of the cost function with respect to the gimbal angle might be exactly zero, which can cause the solver to get stuck, as any direction for the next step looks equally "bad".
### The solution: Add a small pertubation (0.0001) to the initial x and v values if they are zero
This simple fix allowed the solver to converge for the finicky zero-valued initial conditions, though some instability (especially for higher initial values) remains even for non-zero inputs which should be controllable; see below.

![Reachable Set with pertubation fix](https://github.com/sseso/rocket-control/blob/main/showcase/Reachable_Set_Improved.png)
