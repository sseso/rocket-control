# Notebooks

`rocket_control_dev.ipynb` is an earlier development log. New work should import the installed package:

```python
from rocket_control.landing import default_landing_problem, solve_optimal_landing

problem = default_landing_problem()
traj = solve_optimal_landing(problem.initial_state(), problem)
```
