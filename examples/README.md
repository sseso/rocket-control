# Examples

See the README Usage section for the full flag tables. Short copies:

```bash
python -m rocket_control landing
python -m rocket_control landing --x 30 --alt 160 --vx -8 --vy -30 --no-anim --plots

python -m rocket_control attitude --mode rotation --theta0-deg 25 --target-deg 0
python -m rocket_control attitude --mode translation --theta0-deg 25 --target-deg 0 --no-anim
python -m rocket_control attitude --mode dual --theta0-deg 25 --target-deg 0 --omega0-deg 5

python -m rocket_control grid -o results/success_grid.png
```
