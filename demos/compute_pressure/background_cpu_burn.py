#!/usr/bin/env python3
"""Low-priority CPU burner for Scenario B.

This intentionally stresses CPU without touching the Offboard setpoint path.
Use this to demonstrate that high CPU% alone does not necessarily imply
flight-control boundary degradation.
"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import time


def worker(duration_sec: float, duty_cycle: float, nice: int) -> None:
    try:
        os.nice(nice)
    except Exception:
        pass

    end = time.time() + duration_sec
    period = 0.1
    busy_time = max(0.0, min(1.0, duty_cycle)) * period
    idle_time = max(0.0, period - busy_time)
    x = 0.123456789

    while time.time() < end:
        t0 = time.time()
        while time.time() - t0 < busy_time:
            # CPU-bound arithmetic without external dependencies.
            x = math.sin(x + 1.2345) * math.cos(x + 0.3333) + math.sqrt(abs(x) + 1.0)
        if idle_time > 0.0:
            time.sleep(idle_time)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--workers", type=int, default=max(1, os.cpu_count() or 1))
    parser.add_argument("--duty-cycle", type=float, default=0.95)
    parser.add_argument("--nice", type=int, default=10)
    args = parser.parse_args()

    print(
        f"Starting CPU burn: workers={args.workers}, duty={args.duty_cycle}, "
        f"duration={args.duration_sec}s, nice={args.nice}"
    )

    procs = [
        mp.Process(target=worker, args=(args.duration_sec, args.duty_cycle, args.nice), daemon=True)
        for _ in range(max(1, args.workers))
    ]
    for p in procs:
        p.start()

    try:
        for p in procs:
            p.join()
    except KeyboardInterrupt:
        for p in procs:
            p.terminate()


if __name__ == "__main__":
    main()
