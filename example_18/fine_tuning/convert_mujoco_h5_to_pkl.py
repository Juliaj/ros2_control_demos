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
Convert Open_Duck_Playground HDF5 recordings into a pickle usable by
`Open_Duck_Playground/playground/common/plot_saved_obs.py`.

That plotting script expects:
  obses = [obs_0, obs_1, ...]
where each obs_i is a 1D vector (typically length 101).

This converter reads an H5 dataset (default: `observations`) and dumps it as such.

Example:
  python3 convert_mujoco_h5_to_pkl.py \
    --h5 /home/juliajia/dev/Open_Duck_Playground/mujoco_manual_data.h5 \
    --out /tmp/mujoco_saved_obs.pkl
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path
from typing import Optional


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert mujoco_manual_data.h5 observations to plot_saved_obs.py pickle format."
    )
    parser.add_argument(
        "--h5",
        type=Path,
        required=True,
        help="Path to .h5 file (e.g. mujoco_manual_data.h5).",
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default="observations",
        help="H5 dataset name to export (default: observations).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output .pkl path (list of obs vectors).",
    )
    parser.add_argument(
        "--expected-len",
        type=int,
        default=101,
        help="Expected length of each obs vector (default: 101). Use 0 to disable check.",
    )
    args = parser.parse_args()

    if not args.h5.exists():
        raise FileNotFoundError(str(args.h5))

    try:
        import h5py  # type: ignore
    except ImportError as e:
        raise RuntimeError(
            "h5py is required. Install it in your environment (e.g. `pip install h5py`)."
        ) from e

    expected_len: Optional[int] = None if args.expected_len == 0 else args.expected_len

    with h5py.File(args.h5, "r") as f:
        if args.dataset not in f:
            raise KeyError(
                f"Dataset '{args.dataset}' not found in {args.h5}. Available: {list(f.keys())}"
            )
        ds = f[args.dataset]
        if ds.ndim != 2:
            raise ValueError(f"Expected a 2D dataset, got shape {ds.shape} for '{args.dataset}'.")
        if expected_len is not None and ds.shape[1] != expected_len:
            raise ValueError(
                f"Expected width {expected_len}, got {ds.shape[1]} for '{args.dataset}'."
            )

        # Convert to a python list-of-lists to match plot_saved_obs.py usage.
        obses = ds[...].tolist()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("wb") as f:
        pickle.dump(obses, f)

    print(f"Wrote {len(obses)} obs vectors (len={len(obses[0])}) to {args.out}.")


if __name__ == "__main__":
    main()

