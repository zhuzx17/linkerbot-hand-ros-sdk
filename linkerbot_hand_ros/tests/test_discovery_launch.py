"""Exercise actual launch descriptions without scanning adapters or spawning nodes."""

import asyncio
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueCoroutine, OpaqueFunction
from launch.events.process import ProcessStarted
from launch.substitutions import TextSubstitution
from launch_ros.actions import LifecycleNode as LaunchLifecycleNode
from launch_ros.actions import PushRosNamespace
from launch_ros.events.lifecycle import StateTransition
from launch_ros.utilities import evaluate_parameters, normalize_parameters
from lifecycle_msgs.msg import State, TransitionEvent
from linkerbot_hand import ValidationError
from test_discovery_startup import _save_one

from linkerbot_hand_ros import discovery_launch

PACKAGE_ROOT = Path(__file__).parents[1]


@pytest.fixture
def actions(monkeypatch):
    def node(**options):
        options["parameters"] = [
            evaluate_parameters(LaunchContext(), normalize_parameters([value]))[0]
            if isinstance(value, dict)
            else value
            for value in options.get("parameters", [])
        ]
        return SimpleNamespace(kind="container", **options)

    def lifecycle(**options):
        return SimpleNamespace(kind="lifecycle", **options)

    monkeypatch.setattr(discovery_launch, "Node", node)
    monkeypatch.setattr(discovery_launch, "LifecycleNode", lifecycle)


def _hand_launch(values, *, context=None):
    spec = importlib.util.spec_from_file_location(
        "discovery_test_hand_launch", PACKAGE_ROOT / "launch/hand.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = context or LaunchContext()
    context.launch_configurations.update(values)
    actions = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context) or [])
    return context, actions


def test_hand_manual_keeps_original_defaults_and_lifecycle_launch(actions):
    context, nodes = _hand_launch({"autostart": "false"})
    node = nodes[0]
    assert node.kind == "lifecycle"
    assert node.executable == "hand_node"
    assert node.parameters[0]["model"].evaluate(context) == "l6"
    assert (
        node.parameters[0]["transport_json"].evaluate(context)
        == '{"backend":"socketcan","channel":"can0"}'
    )
    assert node.namespace.perform(context) == ""


def test_manual_autostart_runs_one_handshake_from_its_driver_start(monkeypatch):
    calls = []

    async def autostart_once(context, *, driver):
        calls.append((context, driver))

    monkeypatch.setattr(discovery_launch, "_autostart_once", autostart_once)
    context, actions = _hand_launch({})
    register, driver = actions
    assert isinstance(driver, LaunchLifecycleNode)
    assert register.event_handler.handle_once
    for start, goal in (
        ("configuring", "inactive"),
        ("deactivating", "inactive"),
        ("configuring", "unconfigured"),
    ):
        event = StateTransition(
            action=driver,
            msg=TransitionEvent(
                start_state=State(label=start), goal_state=State(label=goal)
            ),
        )
        assert not register.event_handler.matches(event)

    def started(action):
        return ProcessStarted(
            action=action, name="hand", cmd=["hand_node"], cwd=None, env=None, pid=1
        )

    assert not register.event_handler.matches(started(object()))
    event = started(driver)
    register.execute(context)
    assert context.would_handle_event(event)
    (coroutine,) = register.event_handler.handle(event, context)
    assert isinstance(coroutine, OpaqueCoroutine)
    assert not context.would_handle_event(event)

    async def run():
        context._set_asyncio_loop(asyncio.get_running_loop())
        coroutine.execute(context)
        await coroutine.get_asyncio_future()

    asyncio.run(run())
    assert calls == [(context, driver)]


def test_manual_autostart_false_creates_no_transition_handlers():
    _context, actions = _hand_launch({"autostart": "false"})
    assert len(actions) == 1
    assert isinstance(actions[0], LaunchLifecycleNode)


def test_hand_discover_routes_to_same_container_with_root_namespace_and_runtime_rates(
    actions,
):
    _, nodes = _hand_launch(
        {
            "discover": "true",
            "channels": "can1",
            "angles_hz": "50",
            "autostart": "false",
        }
    )
    node = nodes[0]
    assert node.kind == "container"
    assert node.executable == "hand_container"
    params = node.parameters[0]
    assert params["discovery_namespace"] == "/"
    assert params["autostart_override"] == "false"
    assert json.loads(params["discovery_config_json"])["channels"] == ["can1"]
    runtime = json.loads(params["driver_parameters_json"])
    assert runtime["angles_hz"] == 50.0
    assert not {"model", "side", "transport_json"} & runtime.keys()


@pytest.mark.parametrize(
    "namespace,expected",
    [
        ("", "/robot"),
        ("hand", "/robot/hand"),
        ("/absolute", "/absolute"),
    ],
)
def test_single_discovery_namespace_follows_ros_parent_and_absolute_rules(
    actions, namespace, expected
):
    context = LaunchContext()
    PushRosNamespace("/robot").execute(context)
    _, nodes = _hand_launch(
        {"discover": "true", "namespace": namespace}, context=context
    )
    assert nodes[0].parameters[0]["discovery_namespace"] == expected


@pytest.mark.parametrize(
    "name,value",
    [("model", "l6"), ("side", "left"), ("side", ""), ("transport_json", "{}")],
)
def test_even_explicit_old_default_identity_is_rejected_in_auto_mode(
    actions, name, value
):
    with pytest.raises(ValueError, match="explicit manual"):
        _hand_launch({"discover": "true", name: value})


@pytest.mark.parametrize(
    "values",
    [{"discover": "yes"}, {"channels": "can0"}, {"discovery_config": "settings.yaml"}],
)
def test_hand_rejects_invalid_mode_and_unused_discovery_options(actions, values):
    with pytest.raises(ValueError):
        _hand_launch(values)


def _multi(values, default):
    context = LaunchContext()
    context.launch_configurations.update({"discover": "false", **values})
    return context, discovery_launch.build_multi_launch(
        context, default_config=TextSubstitution(text=str(default))
    )[0]


def test_multi_default_uses_original_ros_parameter_file_and_preserves_yaml_autostart(
    actions,
):
    context, node = _multi({}, PACKAGE_ROOT / "config/multi_hand.yaml")
    assert node.executable == "hand_container"
    assert len(node.parameters) == 1
    parameter_file = node.parameters[0]
    try:
        assert Path(parameter_file.evaluate(context)).read_text().strip()
    finally:
        parameter_file.cleanup()


def test_multi_manual_autostart_override_is_explicit(actions):
    _, node = _multi({"autostart": "false"}, PACKAGE_ROOT / "config/multi_hand.yaml")
    assert node.parameters[-1] == {"autostart": False}


def test_multi_preserves_legacy_yaml_merge_keys(actions, tmp_path):
    import yaml

    path = tmp_path / "legacy.yaml"
    path.write_text("""/linkerbot_hand_container:
  ros__parameters:
    device_namespaces: [left, right]
    autostart: false
/left/linkerbot_hand:
  ros__parameters: &driver
    model: l6
    side: left
/right/linkerbot_hand:
  ros__parameters:
    <<: *driver
    side: right
""")
    context, node = _multi({"config": str(path)}, "unused")
    parameter_file = node.parameters[0]
    try:
        resolved = yaml.safe_load(Path(parameter_file.evaluate(context)).read_text())
        assert resolved["/right/linkerbot_hand"]["ros__parameters"] == {
            "model": "l6",
            "side": "right",
        }
    finally:
        parameter_file.cleanup()


@pytest.mark.parametrize(
    "template",
    [
        "$(var hand_parameters)\n",
        "/linkerbot_hand_container:\n  ros__parameters: $(var settings)\n",
    ],
)
def test_multi_preserves_legacy_launch_templates(actions, tmp_path, template):
    import yaml

    path = tmp_path / "template.yaml"
    path.write_text(template)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "discover": "false",
            "config": str(path),
            "hand_parameters": "/linkerbot_hand_container:\n  ros__parameters:\n    device_namespaces: [left, right]\n    autostart: false\n",
            "settings": "{device_namespaces: [left, right], autostart: false}",
        }
    )
    node = discovery_launch.build_multi_launch(
        context, default_config=TextSubstitution(text="unused")
    )[0]
    parameter_file = node.parameters[0]
    try:
        resolved = yaml.safe_load(Path(parameter_file.evaluate(context)).read_text())
        assert resolved["/linkerbot_hand_container"]["ros__parameters"] == {
            "device_namespaces": ["left", "right"],
            "autostart": False,
        }
    finally:
        parameter_file.cleanup()


def test_multi_preserves_field_substitutions(actions, tmp_path):
    import yaml

    path = tmp_path / "fields.yaml"
    path.write_text(
        "/linkerbot_hand_container:\n  ros__parameters:\n    device_namespaces: [$(var left), $(var right)]\n    autostart: $(var start)\n"
    )
    context, node = _multi(
        {"config": str(path), "left": "left", "right": "right", "start": "false"},
        "unused",
    )
    parameter_file = node.parameters[0]
    try:
        resolved = yaml.safe_load(Path(parameter_file.evaluate(context)).read_text())
        assert resolved["/linkerbot_hand_container"]["ros__parameters"][
            "device_namespaces"
        ] == ["left", "right"]
        assert not resolved["/linkerbot_hand_container"]["ros__parameters"]["autostart"]
    finally:
        parameter_file.cleanup()


def test_legacy_file_can_become_valid_yaml_only_after_substitution(actions, tmp_path):
    import yaml

    path = tmp_path / "expression.yaml"
    path.write_text(
        "/linkerbot_hand_container:\n  ros__parameters:\n    device_namespaces: [$(eval \"','.join(['left', 'right'])\")]\n"
    )
    with pytest.raises(yaml.YAMLError):
        yaml.safe_load(path.read_text())
    context, node = _multi({"config": str(path)}, "unused")
    parameter_file = node.parameters[0]
    try:
        resolved = yaml.safe_load(Path(parameter_file.evaluate(context)).read_text())
        assert resolved["/linkerbot_hand_container"]["ros__parameters"][
            "device_namespaces"
        ] == ["left", "right"]
    finally:
        parameter_file.cleanup()


def test_invalid_discovery_schema_is_not_treated_as_legacy(actions, tmp_path):
    path = tmp_path / "invalid.yaml"
    path.write_text("schema: linkerbot_hand_ros.discovery\nversion: 2\nhands: []\n")
    with pytest.raises(ValidationError, match="schema/version"):
        _multi({"config": str(path)}, "unused")


def test_multi_discover_does_not_read_packaged_or_missing_default_yaml(
    actions, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    _, node = _multi(
        {"discover": "true", "channels": "can0,can1"}, tmp_path / "missing.yaml"
    )
    params = node.parameters[0]
    assert params["discovery_mode"] == "multi"
    assert params["discovery_output"] == str(tmp_path / "hands.yaml")
    assert json.loads(params["discovery_config_json"])["channels"] == ["can0", "can1"]


def test_discovery_child_failure_is_propagated_but_orderly_shutdown_is_preserved():
    event = SimpleNamespace(returncode=1)
    context = SimpleNamespace(is_shutdown=False)
    with pytest.raises(RuntimeError, match="exit code 1"):
        discovery_launch._discovery_exit(event, context)
    event.returncode = 0
    assert discovery_launch._discovery_exit(event, context) == []
    event.returncode = -2
    context.is_shutdown = True
    assert discovery_launch._discovery_exit(event, context) == []


def test_multi_generated_config_routes_to_file_mode_without_passing_custom_schema_to_ros(
    actions, tmp_path
):
    path = _save_one(tmp_path)
    _, node = _multi({"config": str(path)}, "unused")
    assert node.parameters[0]["discovery_mode"] == "file"
    assert node.parameters[0]["deployment_config"] == str(path)
    assert node.parameters[0]["autostart_override"] == ""


@pytest.mark.parametrize(
    "values",
    [
        {"discover": "true", "config": "old.yaml"},
        {"channels": "can0"},
        {"output": "hands.yaml"},
        {"autostart": "0"},
        {"discover": "0"},
    ],
)
def test_multi_rejects_conflicting_or_unused_options_before_process_start(
    actions, values
):
    with pytest.raises(ValueError):
        _multi(values, "unused")


def test_advanced_settings_resolve_relative_library_and_do_not_override_explicit_scope(
    actions, tmp_path
):
    path = tmp_path / "settings.yaml"
    path.write_text("channels: [can0]\nminicanfd:\n  library_path: lib/device.so\n")
    _, nodes = _hand_launch({"discover": "true", "discovery_config": str(path)})
    settings = json.loads(nodes[0].parameters[0]["discovery_config_json"])
    assert settings["minicanfd"]["library_path"] == str(tmp_path / "lib/device.so")
    with pytest.raises(ValueError, match="conflicts"):
        _hand_launch(
            {"discover": "true", "discovery_config": str(path), "channels": "can1"}
        )
