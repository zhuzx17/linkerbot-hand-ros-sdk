# Lifecycle and parameters

## Lifecycle

- configure validates parameters and constructs the device/ROS entities, without opening or querying hardware.
- activate opens, verifies identity, reads baseline and enables SDK polling/commands.
- deactivate stops commands and polling, retaining the connection without implicit motion/hold commands.
- cleanup/shutdown/error attempts bounded worker/device shutdown; failed cleanup retains resources for retry and reports errors.

Launch defaults to `autostart:=true`. Manual single-hand launch (`discover:=false`)
automatically configures and activates only at startup. After an initial failure or
cleanup, a subsequent configure stays inactive until an explicit activate.
Activation requires a successful response to that startup's configure request;
transition events alone do not authorize it. Startup waits up to 30 seconds in
total for service discovery and both responses. A failure or unconfirmed result
ends automatic startup without resending transitions. A timed-out request may
still execute: check the lifecycle state before managing it manually.
For external management:

```bash
ros2 launch linkerbot_hand_ros hand.launch.py autostart:=false
```

In another configured terminal, perform the required transitions:

```bash
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
ros2 lifecycle set /linkerbot_hand deactivate
ros2 lifecycle set /linkerbot_hand cleanup
```

## Static parameters

Changes after configure require deactivate/cleanup first.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model` | `l6` | Case-insensitive SDK model; `l20-lite` also accepted |
| `side` | `left` | `left/right`; empty for L30 |
| `transport_json` | SocketCAN `can0` | Strict backend-specific JSON |
| `command_rate_hz` | 250 | SDK continuous-command scheduling ceiling |
| `discrete_queue_capacity` | 16 | Bounded configuration/operation FIFO |
| `mailbox_capacity` | 256 | Device mailbox capacity |
| `route_queue_capacity` | 256 | Transport route queue capacity |
| `identity_timeout_s` | 0.2 | Each identity response stage, including follow-ups |
| `transaction_timeout_s` | 0.0 | Protocol defaults; positive values override ordinary response stages |
| `shutdown_timeout_s` | 2.0 | Worker/connection shutdown budget |
| `auto_reconnect` | false | SDK automatic safe recovery |
| `reconnect_initial_delay_s` | 0.1 | Initial reconnect backoff |
| `reconnect_max_delay_s` | 2.0 | Maximum reconnect backoff |
| `recovery_max_attempts` | 3 | Initialization attempts per recovery cycle |
| `recovery_timeout_s` | 5.0 | Execution budget per recovery cycle |
| `recovery_retry_delay_s` | 0.2 | Minimum receive-quiet interval before retry |

`model` uses SDK names; `product_code` retains reported product identity and is not a model argument.

## Runtime polling and publication

| Parameter | Default | Meaning |
| --- | --- | --- |
| `angles_hz` | 100 | Angle feedback target; policy selects queries/feedback |
| `angle_feedback` | `adaptive` | Query/command-feedback policy |
| `speeds_hz` | 0 | Measured speeds |
| `force_hz` | 0 | Available fn + fns |
| `force_fn_hz`, `force_fns_hz`, `force_ft_hz`, `force_fts_hz`, `force_fd_hz`, `force_fds_hz`, `force_px_hz` | 0 | Named tactile field requests |
| `temperature_hz`, `current_hz`, `faults_hz`, `voltage_hz` | 0 | Corresponding queried telemetry |
| `torques_hz` | 0 | Capability-declared torque channel; L6 is output/mode, O30 measured torque |
| `publish_rate_hz` | 100 | Idle cache age/stale checks; zero pauses state publication |
| `diagnostic_rate_hz` | 1 | Diagnostics publication |

Unsupported positive query rates fail validation. Zero stops that channel's query request,
without erasing cache or preventing command feedback. SDK cache changes trigger immediate publication;
sequence or validity changes produce state messages. Publication rate does not cap device polling.

`angle_feedback` accepts `query_only` (only query responses publish as angle state),
`command_feedback_only` (no supplementary queries; requires `returns_current_state=true`),
and `adaptive` (prefer feedback, query gaps). Query-only still retains command feedback in SDK cache/metrics.

```bash
ros2 param set /linkerbot_hand angles_hz 60.0
ros2 param set /linkerbot_hand angle_feedback query_only
ros2 param set /linkerbot_hand current_hz 10.0
```

Switch tactile shorthand/field rates atomically with
`AsyncParameterClient.set_parameters_atomically()` or `SetParametersAtomically`;
separate CLI calls are not one transaction. See [tactile polling](force.md#polling).

Updates apply after all ROS validation succeeds. Failed application restores the whole parameter/runtime
set; use the final response, as parameter events can show both proposal and restoration. Failed restoration
stops control and closes the connection: complete cleanup, configure and activate before resuming.
If the error path completed cleanup, configure/activate suffices. Nested parameter updates are rejected.
Lifecycle and parameter services share a mutually exclusive callback group; direct calls must also serialize.
The production process uses a multithreaded executor and a separate cache-change publisher.

## Timeouts and recovery

`identity_timeout_s` covers each identity response stage. ROS `transaction_timeout_s=0.0` maps to SDK
`None`; positive values override ordinary baseline/read/poll/response-requiring-command stages.
Timing starts after each request batch is sent and ends on a complete reply. Protocol pacing and write retries are unchanged.

Service `timeout_s` is a finite positive caller-wait budget with room for worker completion within
`shutdown_timeout_s`; it is not the response window and does not cancel shared recovery.

Recovery allows at most `recovery_max_attempts` initialization attempts within `recovery_timeout_s`.
Only identity/baseline response timeouts retry, not identity mismatches, protocol errors or writes.
The quiet interval is the maximum of configured delay, failed response window and protocol guard;
endpoint replies restart it within the same budget. Unsequenced protocols cannot detect arbitrarily late replies.
Blocking I/O is checked on return; expired recovery cannot commit readiness. Automatic reconnect can start another bounded cycle after backoff.

Recovery verifies identity, clears targets and reads baseline, without replaying commands, clearing faults,
enabling or calibrating. After generation changes, applications revalidate feedback and submit new targets.

## Transport JSON

See [connection examples](manual_connection.md). Every object requires `backend`; unknown keys, wrong types
and missing required fields fail validation.

| Backend | Required fields | Optional fields |
| --- | --- | --- |
| `socketcan` | `channel` | `nominal_bitrate`, `data_bitrate`, `auto_reconfigure`, `receive_own_messages` |
| `python_can` | `interface_name`, `interface_type` | `nominal_bitrate`; classic CAN only |
| `vendor_canfd` | None; indices default to 0 | `device_index`, `channel_index`, `library_path`, `nominal_bitrate`, `data_bitrate`, `config_flags`, `model`, `can_type`, `frame_type` |

Bitrates must match capability constraints; classic CAN rejects FD-only settings. SocketCAN booleans default
false. Vendor defaults are `config_flags=7`, `model=0`, `can_type=1`, `frame_type=4`; confirm adapter and
termination wiring. The vendor `model` field is not the hand model.

## Multi-device

Use one physical channel per hand. Namespaces isolate ROS names, not device frames.
The container configures every node before activating any; failure rolls back in reverse order and reports
cleanup errors. Independent launch processes do not provide group lifecycle management.

```yaml
/linkerbot_hand_container:
  ros__parameters:
    device_namespaces: [l6_left, o6_right]
    autostart: true

/l6_left/linkerbot_hand:
  ros__parameters:
    model: l6
    side: left
    transport_json: '{"backend":"socketcan","channel":"can0"}'
    angles_hz: 100.0

/o6_right/linkerbot_hand:
  ros__parameters:
    model: o6
    side: right
    transport_json: '{"backend":"socketcan","channel":"can1"}'
    angles_hz: 100.0
```

Save and launch:

```bash
ros2 launch linkerbot_hand_ros multi_hand.launch.py config:=/absolute/path/hands.yaml
```

`device_namespaces` must be nonempty/unique and have matching node parameter sections.
`autostart` defaults true; false allows individual lifecycle management. Separate channels use separate runtimes.
[Discovery deployments](discovery.md) add scanning and identity constraints using a different YAML schema.

[Back to overview](getting_started.md) · [简体中文](zh/configuration.md)
