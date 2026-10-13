# TRADE

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23347048.svg)](https://doi.org/10.5281/zenodo.23347048)

TRADE contains the computational core of the synchronized and random arrival experiments on horizon-free vehicle ladders. It schedules conflict-cell events, realizes continuous vehicle trajectories, validates the trajectories, and evaluates delay and stochastic collision costs on shared definitions.

The repository includes the physical geometry data and the three practical controller baselines: FCFS, actuated signals, and all-way stop. It is licensed under Apache 2.0.

Archived releases are available on [Zenodo](https://doi.org/10.5281/zenodo.23347048). To cite version 0.1.0 specifically, use [10.5281/zenodo.23347049](https://doi.org/10.5281/zenodo.23347049).

## Installation

Use Python 3.11 or later on Linux or macOS and a Gurobi license that permits the full model sizes. The pip size-limited license is insufficient for these experiments. Install from this repository:

```sh
python -m pip install .
```

The numerical dependencies are Gurobi, NumPy, and SciPy. Geometry is bundled with the installed package, so commands can run from any working directory.

## Run the experiments

Run one 16-vehicle instance with synchronized arrivals:

```sh
trade instance --n-per-lane 2 --seed 1 --arrival synchronized --root runs
```

Run both complete ladders:

```sh
trade ladder --arrival synchronized --workers 2 --root runs
trade ladder --arrival random --workers 2 --root runs
trade check --root runs
```

`python -m trade` is equivalent to `trade`. Each ladder defaults to 2, 3, 4, and 5 vehicles per lane across eight lanes, seeds 1 through 5, and ten exposure ceilings per instance. Together these commands request 40 instances, 400 ceiling schedules, and 120 controller runs. `--vehicles-per-lane` and `--seeds` accept space-separated subsets. The selected setting bounds the supported seeds.

The defaults are a 600-second limit per optimization, a 0.01 relative MIP gap, two solver threads, and an 8 GB solver soft memory limit. Set `--time-limit`, `--mip-gap`, `--threads`, and `--memory-gb` explicitly to change them. Worker count defaults to one; each worker uses the requested solver threads and memory allowance. Building the physical mode catalogue, validating trajectories, and writing artifacts occur outside the optimization time limit. The trajectory LP can retry once after a numerical failure.

For a short execution check, use a separate output root:

```sh
trade instance --n-per-lane 2 --seed 1 --arrival random --time-limit 5 --memory-gb 1 --root quick-runs
trade check --root quick-runs
```

Short budgets test execution and feasibility; their schedules and objective gaps can differ from results at the 600-second default. The limit applies to every optimization, including the Stage 2 trajectory LP, so a limit of a few seconds can stop a 40-vehicle Stage 2 solve before it finds a solution, and the instance then reports `FAILED`. A validated time-limited incumbent is retained with its bound and gap. `COMPLETE` means every requested ceiling and controller produced a validated trajectory and cost record; it does not mean every optimization proved the target gap.

## Outputs and resuming

Each instance writes to `<root>/<setting>/seed_NNNN/`. It contains the settings and source file hashes, anchors, controller trajectories and costs, and one `caps/cap_NN/` folder per ceiling with stage solve records, model and solution archives, polynomial trajectories, validation, and costs. Runtime output is outside the installed package. The repository ignores its default `runs/` folder.

Ceilings are solved from the loosest to the tightest with warm starts. Ceiling 00 is zero exposure when the constructive start succeeds. Ceiling 09 in the ten-ceiling grid is the fast-tie endpoint, which corresponds to the Ma & Li (TRC 2023) speed-maximization benchmark. The fast-tie solve minimizes exposure while retaining the fast anchor's speed reward within 0.001 ft/s. A safe anchor is solved only if the constructive start is incomplete.

Rerunning an identical command skips complete instances only after checking the request identity and required artifacts. Changed numerical settings or source files require a different root for completed results. Interrupted, invalid, or incomplete attempts are moved to timestamped sibling folders before restarting. Operating-system locks prevent simultaneous workers from writing the same instance. Failed anchors, ceilings, or controllers produce a nonzero exit code. `trade check` also fails when no instances exist.

## Core layout

`Decomposed/` supplies the configuration and the eight ladder settings, the 16-cell geometry, seeded scenarios, solver recording, the rule-based controllers, sampled-trajectory validation, and the delay and collision cost definitions. `HorizonFree/` supplies the horizon-free model: `motion/` builds attainable motion modes, `scheduling/` holds the Stage 1 event model, `realization/` holds the Stage 2 trajectory LP, `polynomial/` evaluates Bernstein segments, `certification/` checks delivered polynomials, and `metrics.py` prices validated trajectories on the common definitions. `trade/` contains command-line execution, ladder scheduling, artifact checks, and run records. Code is imported from the module that defines it.

Stage 1 is an inner approximation using attainable motion modes and a lower approximation of the capped approach-speed reward. Stage 2 is a continuous trajectory LP with vehicle-local polynomial intervals. Independent checks use polynomial extrema and threshold roots with float64 tolerances. Neither stage uses a common terminal horizon. The shared physics horizon remains relevant to the controller simulations.

Derivative bounds and segment joins include a scale-dependent float64 roundoff allowance, which is reported alongside the base validation tolerance. This matters for very short waiting segments. Physical discontinuities beyond that allowance still fail validation. Passage lookup finds the first and last crossing of every target in one pass over the trajectory segments, even when position samples contain tiny numerical reversals.

## Cost definitions and scope

Optimization uses mean capped approach speed and a crossing-entry proximity ceiling. Costs are evaluated after trajectory validation using a common lifetime from vehicle release through its individual clearance. Records keep delay cost, expected crossing loss, and expected rear-end loss separate. Collision expectations assume the declared Gaussian timing error, with a default standard deviation of one second. They are conditional model outputs, not measured crash frequencies. The `public` price scenario and its provenance are defined in `Decomposed/cost/pricing/`.

Following checks cover overlapping recorded vehicle lifetimes. Diverging movements are compared on their shared approach; downstream merges between distinct movements are outside this geometry. Initial acceleration is free, and acceleration is continuous thereafter. Runtime behavior and tie solutions can vary with solver version and hardware.

## Verification

With a 20-second limit per optimization, both ladders complete all 40 instances, 400 ceilings, and 120 controller runs from an installed wheel, and `trade check` passes. Every realized trajectory validates, and its cell events occur within 1e-12 s of the scheduled times.
