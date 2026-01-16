# Copyright (C) 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Julia Jia

"""
Convert `run_onnx.log` style logs into a pickle usable by
`Open_Duck_Playground/playground/common/plot_saved_obs.py`.

The plotting script expects a pickle containing:
  obses = [obs_0, obs_1, ...]
where each obs_i is a 1D vector (typically length 101).

This converter parses lines like:
  ... [motion_controller]: [RAW_OBS] step=901, obs=[...101 floats...]

Example:
  python3 convert_run_onnx_log_to_pkl.py \
    --log /path/to/run_onnx.log \
    --out ros2_saved_obs.pkl
"""

from __future__ import annotations

import argparse
import pickle
import re
from pathlib import Path
from typing import List, Optional, Tuple


RAW_OBS_RE = re.compile(r"\[RAW_OBS\]\s+step=(\d+),\s+obs=\[(.*?)\]\s*$")


def _parse_floats_csv(payload: str) -> List[float]:
    # payload is a comma-separated list of floats (no brackets)
    parts = [p.strip() for p in payload.split(",") if p.strip() != ""]
    return [float(p) for p in parts]


def parse_raw_obs_lines(
    log_path: Path, expected_len: Optional[int] = 101
) -> Tuple[List[List[float]], List[int]]:
    obses: List[List[float]] = []
    steps: List[int] = []

    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "[RAW_OBS]" not in line:
                continue
            m = RAW_OBS_RE.search(line.strip())
            if not m:
                continue

            step = int(m.group(1))
            vec = _parse_floats_csv(m.group(2))

            if expected_len is not None and len(vec) != expected_len:
                # Skip malformed / different-format lines (some logs may truncate).
                continue

            steps.append(step)
            obses.append(vec)

    return obses, steps


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert run_onnx.log [RAW_OBS] lines to a pickle list for plotting."
    )
    parser.add_argument(
        "--log",
        type=Path,
        required=True,
        help="Path to run_onnx.log (or any log containing [RAW_OBS] lines).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output .pkl path (will contain a Python list of obs vectors).",
    )
    parser.add_argument(
        "--expected-len",
        type=int,
        default=101,
        help="Expected length of each obs vector (default: 101). Use 0 to disable length checking.",
    )
    args = parser.parse_args()

    if not args.log.exists():
        raise FileNotFoundError(str(args.log))

    expected_len = None if args.expected_len == 0 else args.expected_len
    obses, steps = parse_raw_obs_lines(args.log, expected_len=expected_len)

    if not obses:
        raise RuntimeError(
            f"No valid [RAW_OBS] lines found in {args.log} (expected_len={expected_len})."
        )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("wb") as f:
        pickle.dump(obses, f)

    # Small, human-readable summary to stdout
    first_step = steps[0]
    last_step = steps[-1]
    vec_len = len(obses[0])
    print(
        f"Wrote {len(obses)} obs vectors (len={vec_len}) to {args.out}. "
        f"Steps: first={first_step}, last={last_step}."
    )


if __name__ == "__main__":
    main()

