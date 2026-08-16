# raicom_hmi

`raicom_hmi` is a pure presenter/controller model for operator state display. It
accepts semantic `MissionSnapshot` JSON only; it has **no ROS publishers,
services, action clients, shell execution, or base/arm/pump control paths**.

## Input

The read-only presenter subscribes to `/raicom/mission/snapshot` (`std_msgs/String`)
when launched. The JSON schema is normalized by `raicom_state_server` and supports:
`mode`, `state`, `task`, `primitive`, `dispatch_queue`, `readiness_reasons`,
`referee`, `safety`, and named `capabilities`.

Each capability reports `source`, `mode`, `verification`, and `authorization`.
Unknown, missing, hardware-unverified, unconfigured, or unauthorized capability
values are amber, never green. `STOP_UNCONFIRMED`, `ESTOP`, and referee waiting
are displayed explicitly.

## Operator intent model

The only represented AUTO intents are `START`, `PAUSE`, `SOFT_ESTOP`, `ACK`,
`REFEREE_ACCEPT`, and `REFEREE_REJECT`. `semantic_request()` validates these
allowlisted strings and returns `NOT_DISPATCHED`; it does not transmit anything.
MANUAL and calibration are permanently displayed as `NOT_CONFIGURED` and
`HW_UNVERIFIED`, with device commands disabled.

Start the non-actuating display logger with:

```bash
roslaunch raicom_bringup hmi.launch
```
