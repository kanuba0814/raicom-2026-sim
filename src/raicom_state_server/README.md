# raicom_state_server semantic MissionSnapshot

The state server ingests `/raicom/mission/snapshot` as a `std_msgs/String`
containing JSON. It normalizes it into the read-only `snapshot().mission` field;
malformed or absent data becomes explicit `UNKNOWN`, `WAITING`, and empty-list
values rather than inferred readiness.

Supported semantic fields are `mode`, `state`, `task`, `primitive`,
`dispatch_queue`, `readiness_reasons`, `referee.state`, `referee.reason`,
`safety.state`, `safety.estop`, `safety.motion_allowed`, and `capabilities`.
Every capability is normalized to `source`, `mode`, `verification`, and
`authorization`; omitted fields are `NOT_CONFIGURED`, `UNKNOWN`,
`HW_UNVERIFIED`, and `NOT_AUTHORIZED` respectively.

This is ingestion only. The state server does not interpret operator requests or
publish any device command.
