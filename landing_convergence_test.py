"""Back-compat shim. Prefer: python -m rocket_control grid"""

import sys

from rocket_control.cli import main

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "success_grid"
    raise SystemExit(main(["grid", "-o", f"results/{name}.png"]))
