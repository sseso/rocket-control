# Examples

```bash
# Default landing (needs ffmpeg for the mp4)
python -m rocket_control landing -o results/landing.mp4

# NLP only
python -m rocket_control landing --no-anim --no-plots

# Attitude, flags instead of interactive prompts (needs ffmpeg for the mp4)
python -m rocket_control attitude --mode rotation --theta0-deg 30 --target-deg 0

# Empirical success map
python -m rocket_control grid -o results/success_grid.png
```
