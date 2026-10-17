"""Launch routing and zero-I/O validation for manual/discovered configurations."""

from __future__ import annotations

import asyncio
import json
from dataclasses import replace
from pathlib import Path

import yaml
from launch.actions import OpaqueCoroutine, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.ros_adapters import get_ros_node
from launch_ros.utilities import make_namespace_absolute, prefix_namespace
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from linkerbot_hand import ValidationError

from .discovery_config import (
    channel_filter,
    discovery_settings,
    load_yaml,
    parse_discovery_config,
    read_yaml_document,
)
from .discovery_deployment import SCHEMA, load_deployment

_AUTOSTART_TIMEOUT_S = 30.0
_AUTOSTART_POLL_S = 0.02


def _value(context, name: str, default: str = "") -> str:
    return LaunchConfiguration(name, default=default).perform(context)


def _boolean(value: str, name: str) -> bool:
    if value.lower() not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value.lower() == "true"


def _typed_strings(parameters: dict) -> dict:
    # JSON, empty strings and YAML-like paths must remain ROS string values.
    return {
        name: ParameterValue(value, value_type=str) if isinstance(value, str) else value
        for name, value in parameters.items()
    }


def _discovery_exit(event, context):
    if event.returncode != 0 and not context.is_shutdown:
        raise RuntimeError(f"hand startup failed (exit code {event.returncode})")
    return []


async def _autostart_once(context, *, driver):
    """Configure once, activating only after its explicit successful response."""

    logger = get_logger("linkerbot_hand_ros.autostart")
    loop = context.asyncio_loop
    # This bounds launch's observation, not a native transport call in the driver.
    # Cancelling a request cannot undo a transition already executing remotely.
    deadline = loop.time() + _AUTOSTART_TIMEOUT_S
    node = None
    client = None
    pending = None
    phase = "service discovery"

    async def wait_until(predicate):
        while True:
            if context.is_shutdown or (node is not None and not node.context.ok()):
                raise asyncio.CancelledError
            remaining = deadline - loop.time()
            if remaining <= 0:
                raise TimeoutError
            if predicate():
                return
            await asyncio.sleep(min(_AUTOSTART_POLL_S, remaining))

    try:
        if context.is_shutdown:
            return
        node = get_ros_node(context)
        await wait_until(lambda: True)
        client = node.create_client(ChangeState, f"{driver.node_name}/change_state")
        await wait_until(client.service_is_ready)
        for phase, identifier in (
            ("configure", Transition.TRANSITION_CONFIGURE),
            ("activate", Transition.TRANSITION_ACTIVATE),
        ):
            # Recheck shutdown and the same deadline before each non-idempotent call.
            await wait_until(lambda: True)
            pending = client.call_async(
                ChangeState.Request(transition=Transition(id=identifier))
            )
            await wait_until(
                lambda request=pending: request.done() or request.cancelled()
            )
            if pending.cancelled():
                logger.warning(
                    f"{driver.node_name}: initial {phase} request was cancelled; "
                    "its outcome is unknown and autostart will not continue"
                )
                return
            response = pending.result()
            client.remove_pending_request(pending)
            pending = None
            if not response.success:
                logger.error(
                    f"{driver.node_name}: initial {phase} failed; "
                    "autostart will not continue"
                )
                return
    except TimeoutError:
        outcome = (
            "the sent request's outcome is unknown"
            if pending is not None
            else "no request is awaiting confirmation"
        )
        logger.error(
            f"{driver.node_name}: autostart timed out during {phase}; {outcome}; "
            "no further automatic transition will be sent"
        )
    except asyncio.CancelledError:
        logger.warning(
            f"{driver.node_name}: autostart cancelled during {phase}; "
            "any sent request may still execute; no further automatic transition will be sent"
        )
    except Exception as error:  # noqa: BLE001 - stop at the launch/ROS boundary
        logger.error(
            f"{driver.node_name}: autostart stopped during {phase}: {error}; "
            "the transition result cannot be confirmed; "
            "no further automatic transition will be sent"
        )
    finally:
        if pending is not None:
            try:
                try:
                    client.remove_pending_request(pending)
                finally:
                    pending.cancel()
            except Exception as error:  # noqa: BLE001 - cleanup must still destroy the client
                logger.warning(
                    f"{driver.node_name}: could not discard autostart request: {error}"
                )
        if client is not None:
            try:
                node.destroy_client(client)
            except Exception as error:  # noqa: BLE001 - adapter may already be shutting down
                logger.warning(
                    f"{driver.node_name}: could not destroy autostart client: {error}"
                )


def check_hand_mode(context):
    """Run before manual defaults are declared, retaining explicit intent."""

    discover = _boolean(_value(context, "discover", "false"), "discover")
    if discover:
        conflicts = {
            "model",
            "side",
            "transport_json",
        } & context.launch_configurations.keys()
        if conflicts:
            raise ValueError(
                f"discover cannot use explicit manual parameters {sorted(conflicts)}; use channels or discovery_config"
            )
    elif _value(context, "channels") or _value(context, "discovery_config"):
        raise ValueError("channels/discovery_config require discover:=true")
    return []


def _settings(context) -> dict:
    path_value = _value(context, "discovery_config")
    if path_value:
        path = Path(path_value).expanduser().resolve()
        try:
            config = parse_discovery_config(load_yaml(path), base_dir=path.parent)
        except ValidationError as error:
            raise ValidationError(f"discovery_config={path}: {error}") from error
    else:
        config = parse_discovery_config({})
    channels = channel_filter(_value(context, "channels"))
    if channels is not None:
        if config.channels is not None and config.channels != channels:
            raise ValueError("channels conflicts with discovery_config.channels")
        config = replace(config, channels=channels)
    return discovery_settings(config)


def build_hand_launch(context, *, arguments):
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _default, value_type in arguments
    }
    if not _boolean(_value(context, "discover"), "discover"):
        driver = LifecycleNode(
            package="linkerbot_hand_ros",
            executable="hand_node",
            name="linkerbot_hand",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[parameters],
        )
        if not _boolean(_value(context, "autostart"), "autostart"):
            return [driver]
        # A private, bounded service sequence does not depend on initial volatile
        # transition events. Later manual configurations cannot restart autostart.
        return [
            RegisterEventHandler(
                OnProcessStart(
                    target_action=driver,
                    on_start=[
                        OpaqueCoroutine(
                            coroutine=_autostart_once, kwargs={"driver": driver}
                        )
                    ],
                    handle_once=True,
                )
            ),
            driver,
        ]
    runtime = {
        name: value.evaluate(context)
        for name, value in parameters.items()
        if name not in {"model", "side", "transport_json"}
    }
    return [
        Node(
            package="linkerbot_hand_ros",
            executable="hand_container",
            output="screen",
            on_exit=_discovery_exit,
            parameters=[
                _typed_strings(
                    {
                        "discovery_mode": "single",
                        "discovery_config_json": json.dumps(_settings(context)),
                        "driver_parameters_json": json.dumps(runtime),
                        "discovery_namespace": make_namespace_absolute(
                            prefix_namespace(
                                context.launch_configurations.get("ros_namespace"),
                                _value(context, "namespace"),
                            )
                        ),
                        "autostart_override": str(
                            _boolean(_value(context, "autostart"), "autostart")
                        ).lower(),
                    }
                )
            ],
        )
    ]


def build_multi_launch(context, *, default_config):
    discover = _boolean(_value(context, "discover"), "discover")
    config_value = _value(context, "config")
    override = _value(context, "autostart")
    discovery_file = False
    if override:
        override = str(_boolean(override, "autostart")).lower()
    if discover:
        if config_value:
            raise ValueError("config and discover:=true are mutually exclusive")
        output = Path(_value(context, "output") or "hands.yaml").expanduser().absolute()
        parameters = [
            {
                "discovery_mode": "multi",
                "discovery_config_json": json.dumps(_settings(context)),
                "discovery_output": str(output),
                "autostart_override": override,
            }
        ]
    else:
        if any(
            _value(context, name) for name in ("channels", "discovery_config", "output")
        ):
            raise ValueError("channels/discovery_config/output require discover:=true")
        path = (
            Path(config_value or default_config.perform(context)).expanduser().resolve()
        )
        # A legacy file may only become valid YAML after launch substitutions.
        # New deployments are static and validated against their original path.
        try:
            document = read_yaml_document(path, strict=False)
        except ValidationError as error:
            if not isinstance(error.__cause__, yaml.YAMLError):
                raise
            document = None
        discovery_file = (
            isinstance(document, dict)
            and "schema" in document
            and not isinstance(document["schema"], dict)
        )
        if discovery_file:
            if document["schema"] != SCHEMA:
                raise ValueError(f"unsupported deployment schema: config={path}")
            load_deployment(path)
            parameters = [
                {
                    "discovery_mode": "file",
                    "deployment_config": str(path),
                    "autostart_override": override,
                }
            ]
        else:
            from launch_ros.parameter_descriptions import ParameterFile

            parameters = [ParameterFile(str(path), allow_substs=True)]
            if override:
                parameters.append({"autostart": override == "true"})
    return [
        Node(
            package="linkerbot_hand_ros",
            executable="hand_container",
            output="screen",
            parameters=[
                _typed_strings(value) if isinstance(value, dict) else value
                for value in parameters
            ],
            on_exit=_discovery_exit if discover or discovery_file else None,
        )
    ]
