"""Back-compat shim. Prefer: python -m rocket_control attitude"""

import argparse
import sys

from rocket_control.cli import main

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--name", default=None)
    p.add_argument("--mode", default="rotation", choices=["rotation", "translation", "dual"])
    args, rest = p.parse_known_args()
    out = f"results/{args.name}.mp4" if args.name else f"results/attitude_{args.mode}.mp4"
    raise SystemExit(main(["attitude", "--mode", args.mode, "-o", out, "--interactive", *rest]))
