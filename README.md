# OmniNav Standalone Frontier

Non-ROS C++ lidar grid-map and frontier detector with a Python `ctypes` wrapper.

## Build

```bash
cd /home/nuc/ws/isaac_ai-demo/omninav_slow_test_client/standalone_frontier
./scripts/build.sh
```

This produces:

```text
build/libomninav_frontier.so
```

## Python Use

```python
import math
import sys

import numpy as np

sys.path.insert(
    0,
    "/home/nuc/ws/isaac_ai-demo/omninav_slow_test_client/standalone_frontier/python",
)
from omninav_frontier import FrontierMap

frontier_map = FrontierMap(
    width=1024,
    height=1024,
    meters_per_pixel=0.1,
    origin_x=0.0,
    origin_y=0.0,
    min_frontier_distance_m=0.5,
    max_frontier_distance_m=20.0,
)

ranges = np.full(720, 8.0, dtype=np.float64)
result = frontier_map.update(
    ranges=ranges,
    angle_min=-math.pi,
    angle_increment=(2.0 * math.pi) / ranges.size,
    range_min=0.05,
    range_max=20.0,
    odom=(0.0, 0.0, 0.0, 0.0),
    reset=True,
)

grid_map = result.grid_map
frontiers = result.frontiers
```

## Interface

Input:

- `ranges`: one-dimensional 2D lidar ranges, `float64` is passed to C++ without conversion.
- `angle_min`: first beam angle in radians.
- `angle_increment`: beam angle increment in radians.
- `range_min`, `range_max`: valid lidar range limits.
- `odom`: `(x, y, z, yaw)` in ROS-style planar odom convention.
- `reset`: when true, clears the persistent map before applying this scan.

Output:

- `grid_map`: `np.ndarray`, shape `(height, width)`, dtype `uint8`.
  - `0`: unknown
  - `1`: free
  - `2`: occupied
- `frontiers`: list of dictionaries:
  - `id`: string, for example `"f0"`
  - `index`: integer segment index
  - `cell_count`: connected frontier-cell count
  - `boundary_edge_count`: free/unknown boundary length in pixels
  - `midpoint_rc`: `[row, col]`
  - `local_xz`: `[right_m, forward_m]`
- `metadata`: map metadata plus the same `frontiers` list.

The C++ core is independent of ROS2. The Python layer only depends on `numpy`.
