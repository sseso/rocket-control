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
