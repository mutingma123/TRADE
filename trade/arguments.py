"""Explicit instance and ladder settings with the experiment defaults."""

import argparse
import math
from pathlib import Path


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="Run the horizon-free intersection experiments")
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("instance", "ladder"):
        child = commands.add_parser(command)
        child.add_argument("--root", type=Path, default=Path("runs"))
        child.add_argument("--arrival", choices=("synchronized", "random"),
                           default="synchronized")
        child.add_argument("--caps", type=positive_int, default=10)
        child.add_argument("--time-limit", type=positive_float, default=600.0)
        child.add_argument("--mip-gap", type=positive_float, default=0.01)
        child.add_argument("--threads", type=positive_int, default=2)
        child.add_argument("--memory-gb", type=positive_float, default=8.0)
        if command == "instance":
            child.add_argument("--n-per-lane", type=int, choices=(2, 3, 4, 5), required=True)
            child.add_argument("--seed", type=positive_int, required=True)
        else:
            child.add_argument("--vehicles-per-lane", type=int, nargs="+", choices=(2, 3, 4, 5),
                               default=[2, 3, 4, 5])
            child.add_argument("--seeds", type=positive_int, nargs="+", default=[1, 2, 3, 4, 5])
            child.add_argument("--workers", type=positive_int, default=1)
    check = commands.add_parser("check")
    check.add_argument("--root", type=Path, default=Path("runs"))
    args = parser.parse_args(argv)
    if getattr(args, "caps", 2) < 2:
        parser.error("--caps must include both anchors, so it must be at least 2")
    if getattr(args, "mip_gap", 0.01) > 1:
        parser.error("--mip-gap must be at most 1")
    if args.command == "ladder" and max(args.seeds) > 20:
        parser.error("ladder seeds must be in 1..20")
    return args
