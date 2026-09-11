# Rocket Control
This repository contains a **direct collocation** solver (trapezoidal method + CasADi + IPOPT) for a 2D rocket landing problem with variable mass, moving center of mass, and thrust vector control via gimbal. It also includes animations and diagnostic plots for the calculated solutions.

![2D Landing Showcase GIF](https://github.com/sseso/rocket-control/blob/main/showcase/rocket_landing_smooth.gif)

# Installation & Usage
## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/sseso/rocket-control.git
   cd rocket-control
   ```
2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate      # Linux/macOS
   # or
   venv\Scripts\activate         # Windows
   ```
3. Install the package (editable) and test extras:
   ```bash
   pip install -e ".[dev]"
   pytest
   ```
**Note:** This project depends on CasADi and IPOPT.
On many systems you can install CasADi via pip (`pip install casadi`).
IPOPT is usually installed automatically with CasADi when using the pip package, but if you encounter solver issues you may need a system-level IPOPT installation (e.g. via conda, brew, apt, or from https://coin-or.github.io/Ipopt/INSTALL.html). Animations require `ffmpeg` on your PATH.

## Usage

```bash
python -m rocket_control <landing|attitude|grid> [flags]
```

Default run: solve, print a report, write an animation (`ffmpeg` required for mp4). Degrees on the CLI are converted to radians internally. Diagnostics are opt-in via `--plots`.

### Landing

Time-optimal 2D NLP. Default IC: \(x=30\,\mathrm{m}\), nozzle alt \(160.42\,\mathrm{m}\), \(v_x=-8\,\mathrm{m/s}\), \(v_y=-30\,\mathrm{m/s}\), \(\theta=\omega=0\).

| Flag | Meaning |
|------|---------|
| `-o, --output MP4` | animation path (default `results/landing.mp4`) |
| `--x M` | initial horizontal offset from the pad [m] (state \(x\)) |
| `--alt M` | initial **nozzle** height [m]; state \(y = \mathrm{alt} + d_{\mathrm{com}}\) |
| `--vx M/S` | initial horizontal velocity [m/s] |
| `--vy M/S` | initial vertical velocity [m/s] (negative = down) |
| `--theta-deg DEG` | initial pitch from vertical [deg] |
| `--omega-deg DEG/S` | initial pitch rate [deg/s] |
| `--no-anim` | skip the mp4 |
| `--plots [PNG]` | diagnostic figure: no path opens a window; a path saves a PNG |
| `--verbose` | IPOPT internals |

```bash
python -m rocket_control landing
python -m rocket_control landing -o results/landing.mp4
python -m rocket_control landing --x 30 --alt 160 --vx -8 --vy -30 --theta-deg 0 --omega-deg 0
python -m rocket_control landing --no-anim --plots
python -m rocket_control landing --no-anim --plots results/landing_plots.png
python -m rocket_control landing --verbose --no-anim
```

### Attitude

Closed-loop bang-bang gimbal on the **same** vacuum plant (\(g=0\)).

| Flag | Meaning |
|------|---------|
| `--mode {rotation,translation,dual}` | `rotation`: body-fixed view; `translation`: free translation; `dual`: both (default `rotation`) |
| `-o, --output MP4` | animation path (default `results/attitude_<mode>.mp4`) |
| `--theta0-deg DEG` | initial pitch [deg] (default 20) |
| `--target-deg DEG` | target pitch [deg] (default 0) |
| `--omega0-deg DEG/S` | initial pitch rate [deg/s] (default 0) |
| `--dry-mass KG` | dry mass (default 1250) |
| `--fuel-mass KG` | fuel mass (default 500) |
| `--thrust N` | constant thrust (default \(T_{\max}=50000\)) |
| `--isp S` | specific impulse (default 500) |
| `--no-anim` | skip the mp4 |
| `--plots [PNG]` | same as landing |
| `--verbose` | extra solver / sim chatter |

```bash
python -m rocket_control attitude --mode rotation --theta0-deg 25 --target-deg 0 --omega0-deg 5
python -m rocket_control attitude --mode translation --theta0-deg 25 --target-deg 0
python -m rocket_control attitude --mode dual --theta0-deg 25 --target-deg 0 --omega0-deg 5 -o results/attitude_dual.mp4
python -m rocket_control attitude --mode rotation --no-anim --plots
```

### Grid

Empirical success map over \((x, \text{nozzle altitude})\). Not a reachable set: each cell is heuristic check + IPOPT + landing tolerances (see Numerical / Convergence Issues below).

| Flag | Meaning |
|------|---------|
| `-o, --output PNG` | figure path (default `results/success_grid.png`); also writes a `.npz` |
| `--show` | open the saved figure after writing |
| `--verbose` | IPOPT internals for every grid point |

```bash
python -m rocket_control grid
python -m rocket_control grid -o results/success_grid.png
python -m rocket_control grid -o results/success_grid.png --show
```


### Layout

```
src/rocket_control/
  core/       shared vehicle, CoM/MoI, f(x,u)   (SI, radians)
  landing/    trapezoidal NLP, costs, evaluation
  attitude/   slew-limited gimbal law + Euler loop
  viz/        matplotlib only
  cli.py
tests/        analytic plant checks (no ffmpeg)
showcase/     curated GIFs
results/      runtime outputs (gitignored)
```

# The Control Problem
Consider a rocket in a gravitational field whose engine can gimbaled a fixed amount, for example $\alpha \in [-10\degree, 10\degree]$ (for simplicity, ignore aerodynamic forces/drag). Assume we can control the thrust $T$ as well as the gimbal angle $\alpha$. 
Given some initial conditions for the rocket (see state vector below), find a control that lands the rocket at a specified target with zero velocity and in an upright position.

![Rocket Sketch](https://github.com/sseso/rocket-control/blob/main/showcase/rocket_sketch.jpg)

# Mathematical formulation

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
&\text{(angular velocity, $\dot{\theta} = \omega$)} \\
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

### Assumptions on Mass Distribution and Time-Dependent Mass

The model simplifies the rocket's mass distribution to handle shifting CoM and MoI during fuel depletion. Key assumptions:

1. **Uniform Distribution**:
   - Dry mass $m_{\text{dry}}$ is uniform along rocket height $h_{\text{rocket}}$, modeled as a slender rod.
   - Fuel mass (initial $m_{\text{fuel}}$) is uniform in a tank of height $h_{\text{fuel}} = \frac{1}{2} h_{\text{rocket}}$.

2. **Fixed Positions**:
   - Measured from nozzle upward.
   - Dry CoM: $d_{\text{dry}} = \frac{1}{2} h_{\text{rocket}}$.
   - Fuel CoM: $d_{\text{fuel}} = \frac{1}{2} h_{\text{fuel}}$ (tank in lower half).
   - Thrust at nozzle; torque arm is current CoM distance.

3. **Time-Dependent Mass**:
   - $m(t)$ decreases: $\dot{m} = -\frac{T}{v_e}$ $\quad (v_e = I_{\text{sp}} \cdot g_0$, where $I_{\text{sp}}$ is the specific impulse).
   - Fuel: $\max(m(t) - m_{\text{dry}}, 0)$.
   - Dry mass constant; no other losses.
   - CoM shifts upward, MoI decreases with fuel depletion.

4. **MoI Simplifications**:
   - Components as uniform rods: $I_{\text{cm}} = \frac{1}{12} m L^2$.
   - Total $I_z(m)$ via parallel axis theorem for 2D rotation.
   - $I_z(m) > 0$ ensured by $m \geq m_{\text{dry}}$.

5. **Implicit Assumptions**:
   - Rigid body; no sloshing or non-rigid effects.
   - Uniform fuel depletion.
   - Mass-dependent (via $m(t))$; updates per step/node.

### Mass-Dependent Geometry Equations

CoM and MoI as functions of mass $m$:

$$
d_{\text{com}}(m) = \frac{m_{\text{dry}} \cdot d_{\text{dry}} + \max(m - m_{\text{dry}}, 0) \cdot d_{\text{fuel}}}{m}
$$

$$
I_z(m) = I_{\text{dry,cm}} + m_{\text{dry}} (d_{\text{dry}} - d_{\text{com}}(m))^2 + I_{\text{fuel,cm}}(m) + \max(m - m_{\text{dry}}, 0) (d_{\text{fuel}} - d_{\text{com}}(m))^2
$$

where

$$
I_{\text{dry,cm}} = \frac{1}{12} m_{\text{dry}} h_{\text{rocket}}^2,
$$

$$
I_{\text{fuel,cm}}(m) = \begin{cases} 
\frac{1}{12} (m - m_{\text{dry}}) h_{\text{fuel}}^2 & m > m_{\text{dry}}, \\
0 & \text{otherwise}.
\end{cases}
$$

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

The condition $y(t_f) = d_\text{com}(m(t_f))$ ensures the nozzle touches the ground at touchdown. Sign convention: $\theta = 0$ is upright, $\omega = \dot\theta$, and pitch acceleration is $-(d_{\mathrm{com}} T / I_z)\sin\alpha$.

### Path Constraints & Bounds
- $y(t) \geq 0$
- $|v_x(t)| \leq v_{x_{\text{max}}}$ [m/s]
- $v_{y_{\text{min}}} \leq v_y(t) \leq v_{y_{\text{max}}}$ [m/s]
- $|\omega(t)| \leq \omega_{\text{max}}$ [rad/s]
- $m_\text{dry} \leq m(t) \leq m_0$
- Thrust: $T_{\min} \leq T(t) \leq T_\text{max}$ with a modelling throttle floor $T_{\min} = 4000\,\text{N}$ (the idealised problem allows $T=0$; the floor keeps IPOPT away from a singular coast). Set `VehicleSpec.T_min = 0` to recover the README idealisation.
- Gimbal (most of flight): $|\alpha(t)| \leq 10^\circ$
- Gimbal & pitch (near landing): $|\alpha(t)| \leq 2^\circ$, $|\theta(t)| \leq 2^\circ$

### Objective Function (to be Minimized)

$$
J = w_t t_f + \int_0^{t_f} L(x,u,\dot{\alpha})\, dt + J_{\text{ground}} + J_{\text{inverse-h}}
$$

### Running Cost

$$
L = w_{\text{thrust}} T^2 + w_{\text{gimbal}} \alpha^2 + w_{\text{gimbal rate}} \left( \frac{d\alpha}{dt} \right)^2 + w_\theta \theta^2 + w_{\text{alt thrust}} T^2 \cdot \frac{y}{y_0} + w_{\text{landing}} (\theta^2 + \alpha^2) \cdot p(t)
$$

where

$$
p(t) = \max\left(0, 1 - \frac{y - d_{\text{com}}(m)}{2 h_{\text{rocket}}}\right)^3
$$

### Additional Penalties

Ground violation penalty (soft complement to $y \ge 0$; weight `w_ground`):

$$
J_{\text{ground}} = w_{\text{ground}} \int_0^{t_f} \max\left(0, -(y - d_{\text{com}}(m))\right)^2 \, dt
$$

with default $w_{\text{ground}} = 10^8$. The implementation applies this as a nodal sum (not multiplied by $\Delta t$).

Velocity-altitude penalty (softened to avoid singularity):

$$
J_{\text{inverse-h}} = w_v \int_0^{t_f} \frac{v_x^2 + v_y^2}{\max(\epsilon, y - d_{\text{com}}(m))} \, dt
$$

with $\epsilon = 5$ m.

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

The problem is discretized using **trapezoidal collocation** with $N=40$ intervals ($N$ can be varied for desired precision, though computation time increases with N, roughly $\sim O(N^2)$ based on test runs).

Exactly-zero $x$, $v_x$, or $v_y$ is regularised by `eps_state = 1e-4` inside the solver so the gimbal cost gradient is not identically zero.


# Approach
Since solving the full problem from scratch was quite indimidating, the problem was broken down into three steps:

1. **The 1D Problem**: Consider a falling rocket in a gravitational field (only y-component, no angular deviations). Find a thrust control which lands the rocket with zero velocity.
2. **Rotational dynamics**: Consider a rocket floating in the vacuum of space, with no external forces acting on it. Now assume the engine can gimbal in a fixed range. A **closed-loop bang-bang / slew-limited gimbal law** (not an NLP) rotates the rocket from an initial angle $\theta_0$ to a target angle $\theta_t$ on the same $f(x,u)$ with $g=0$.
3. **Combine the dynamics** --> Solve the full 2D Problem.

The 1D problem (landing an upright, falling rocket) is not too interesting, as it only involved controlling thrust (for time optimal controls, this happens in a bang-bang manner). The rotational case is already more interesting, as one has to model the gimbal tilt as well as the resulting torque, which changes over time, since the center of mass and the moment of inertia change as the fuel burns. Additionally, any attitude correction maneuvers cause translation, which has to be taken into account, too. Here's how the rotational dynamics turned out:

![Rotation GIF](https://github.com/sseso/rocket-control/blob/main/showcase/rotation.gif)

# Plots
If a trajectory solution to the given initial conditions is found, the program will also display plots showing the evolution of the most important parameters over time, which helps in visualizing the trajecory as well as sanity-checks and diagnostic testing. Below are some (not all) plots corresponding to the landing animation at the very top of the README.

![Plot Example](https://github.com/sseso/rocket-control/blob/main/showcase/plots_example.png)

# Performance, Issues & Fixes
## SLSQP vs. CasADi / IPOPT
### SLSQP is not feasible for this kind of problem
The full 2D case is solved with a numerical solver. Initially, the simulation was implemented with scipy.optimize.minimize's SLSQP (Sequential Least SQuares Programming), which is a gradient-based optimization algorithm used to minimize a scalar function of multiple variables subject to bounds, equality, and inequality constraints. However, this algorithm was not suitable for this direct collocation problem:
- No explicit analytical gradients are given, which means SciPy is estimating (with finite differences) them by tweaking every single variable one by one. With $N=40$ collocation nodes, that's ~370 variables. That means SciPy runs the physics simulation ~371 times per iteration just to figure out which direction to step. It's also sensitive to noise; If the physics simulation has small numerical jitters, the estimated gradient can point in an entirely wrong direction, giving wrong results.
- The main way to control the behaviour of the rocket is through tuning the weights of the cost function. Large penalties on constraints (like J_ground = 1e8 * ...) create extremely steep gradients that SLSQP struggles to navigate, causing it to take tiny step sizes and run up the iteration count.
- In direct collocation, a state at node $k$ only affects node $k+1$. This creates a highly diagonal, "sparse" Jacobian matrix. SLSQP cannot recognize that most entries of the Jacobian are zero and therefore don't contribute to the next step, wasting massive amounts of memory and CPU cycles.

This caused **runtimes of 3-10 minutes for just a single simulation** (solution + animation), which is far too inefficient for any real use case.

### The solution: CasADi / IPOPT
The dynamics and objective were ported to CasADi (a Python library specifically built for these kinds of problems). CasADi uses Algorithmic Differentiation (exact gradients with zero finite difference overhead) and uses IPOPT, an interior-point solver designed for large, sparse non-linear programming (NLP) problems. IPOPT knows that the vast majority of entries in the Jacobian are zero and only calculates the non-zero interactions.

This **reduced the runtime to only a few seconds per simulation** (<5s solver time, ~15-25s animation time). Without this change, the convergence test across a wide grid of initial conditions would not have been feasible (hours of runtime vs. minutes).

## Numerical / Convergence Issues with CasADi / IPOPT
Still, CasADi / IPOPT is not without its own issues. One main issue is numerical instability, which causes the solver to fail convergence for normally controllable initial conditions. In particular, initial conditions where the initial $x$ position and total velocity were exactly zero consistently failed to converge, which can be seen in the convergence test below.

![Reachable Set with numerical artifacts](https://github.com/sseso/rocket-control/blob/main/showcase/Reachable_Set_Buggy.png)

The reason for this is that when $x=0$, $v=0$, and $\theta=0$, the derivative of the cost function with respect to the gimbal angle might be exactly zero, which can cause the solver to get stuck, as any direction for the next step looks equally "bad".
### The solution: regularise $|x|,|v_x|,|v_y| < \varepsilon$ inside the NLP
`landing.nlp.regularize_initial_state` applies $\varepsilon = 10^{-4}$. This lets IPOPT move off the stationary gimbal gradient. Remaining failures on the grid are empirical solver outcomes, not a proof of uncontrollability.

![Reachable Set with pertubation fix](https://github.com/sseso/rocket-control/blob/main/showcase/Reachable_Set_Improved.png)


# Sources
Kelly, M. (2017). An introduction to trajectory optimization: How to do your own direct collocation. SIAM Review, 59(4), 849–904. https://doi.org/10.1137/16M1062569
