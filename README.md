# putm_recorder

ROS 2 package for automatic rosbag recording triggered by the RTD (Ready To Drive) signal.

## Overview

`putm_recorder` subscribes to the `/putm_vcl/rtd` topic and:
- **Starts recording** when `state = true` (car is Ready To Drive)
- **Stops recording** 20 seconds after `state = false` (with a timer, so it won't stop if the car becomes ready again before timeout)

Recordings are saved in [MCAP](https://mcap.dev/) format.

## Dependencies

- `rclpy`
- `putm_vcl_interfaces` (for the `Rtd` message)

## Recording path

| Condition | Path |
|---|---|
| `/home/putm/rosbag/` exists | `/home/putm/rosbag/recording_<timestamp>` |
| Fallback (dev machine) | `/tmp/rosbag/recording_<timestamp>` |

The node automatically creates the fallback directory if needed and logs a warning.

## Build

```bash
# Build with dependencies
colcon build --packages-up-to putm_recorder

# Or just this package (putm_vcl_interfaces must already be built)
colcon build --packages-select putm_recorder
```

## Running

```bash
source install/setup.bash

# Via launch file (recommended)
ros2 launch putm_recorder putm_recorder.launch.py

# Or directly
ros2 run putm_recorder rec_node
```

## Testing (without hardware)

**Terminal 1 — start the node:**
```bash
source /install/setup.bash
ros2 launch putm_recorder putm_recorder.launch.py
```

**Terminal 2 — verify node is running:**
```bash
ros2 node list
# expected: /putm_recorder/rec_node

ros2 node info /putm_recorder/rec_node
```

**Terminal 2 — simulate RTD signal:**
```bash
# Trigger recording start (state = true)
ros2 topic pub --once /putm_vcl/rtd putm_vcl_interfaces/msg/Rtd "{state: true}"

# Trigger recording stop after 20s (state = false)
ros2 topic pub --once /putm_vcl/rtd putm_vcl_interfaces/msg/Rtd "{state: false}"
```

**Expected logs in Terminal 1:**
```
[rec_node]: Car is ready. Starting data recording.
[rec_node]: Started recording with PID: XXXX
...
[rec_node]: Car is not ready. Stopping data recording after 20 seconds.
[rec_node]: 20 seconds passed. Stopping recording.
[rec_node]: Stopping process (PID: XXXX)...
[rec_node]: Process ros2 bag record completed.
```

## Package structure

```
putm_recorder/
├── launch/
│   └── putm_recorder.launch.py   # Launch file
├── putm_recorder/
│   ├── __init__.py
│   └── rec_node.py               # Main node
├── resource/
│   └── putm_recorder             # ament index marker
├── package.xml
├── setup.cfg
└── setup.py
```
