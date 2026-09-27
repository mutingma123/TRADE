"""Schedule independent ladder instances in separate solver processes."""

from concurrent.futures import ProcessPoolExecutor, as_completed
from copy import copy
from multiprocessing import get_context

from Decomposed.result.artifacts import write_json
from trade.execution.instance_request import run_one
from trade.records.run_status import utc_now


def run_ladder(args):
    jobs = []
    for count in dict.fromkeys(args.vehicles_per_lane):
        for seed in dict.fromkeys(args.seeds):
            job = copy(args)
            job.n_per_lane, job.seed = count, seed
            jobs.append(job)
    rows = []
    with ProcessPoolExecutor(max_workers=min(args.workers, len(jobs)),
                             mp_context=get_context("spawn")) as executor:
        futures = {executor.submit(run_one, job): job for job in jobs}
        for future in as_completed(futures):
            job = futures[future]
            row = dict(n_per_lane=job.n_per_lane, seed=job.seed)
            try:
                row["exit_code"] = future.result()
            except Exception as error:
                row.update(exit_code=1, error=f"{type(error).__name__}: {error}")
            rows.append(row)
            print(f"ladder {args.arrival}: {len(rows)}/{len(jobs)} finished; {row}", flush=True)
    complete = all(row["exit_code"] == 0 for row in rows)
    write_json(args.root / f"ladder_{args.arrival}.json",
               dict(valid=complete, finished_at=utc_now(),
                    instances=sorted(rows, key=lambda row: (row["n_per_lane"], row["seed"]))))
    return 0 if complete else 1
