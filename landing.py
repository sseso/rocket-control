"""Back-compat shim. Prefer: python -m rocket_control landing"""

import sys

from rocket_control.cli import main

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "landing"
    raise SystemExit(main(["landing", "-o", f"results/{name}.mp4", *sys.argv[2:]]))
