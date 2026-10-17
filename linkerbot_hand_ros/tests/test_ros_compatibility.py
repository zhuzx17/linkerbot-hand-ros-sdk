import ast
import re
import runpy
import xml.etree.ElementTree as ET
from pathlib import Path

from linkerbot_hand_ros.driver_config import DRIVER_PARAMETERS

PACKAGE_ROOT = Path(__file__).parents[1]


def test_management_services_use_keyword_only_callback_groups() -> None:
    source = (PACKAGE_ROOT / "linkerbot_hand_ros/management.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_service"
    ]

    assert calls
    for call in calls:
        assert len(call.args) == 3
        assert any(keyword.arg == "callback_group" for keyword in call.keywords)


def test_entry_points_use_idempotent_rclpy_shutdown() -> None:
    entry_points = (
        PACKAGE_ROOT / "linkerbot_hand_ros/node.py",
        PACKAGE_ROOT / "linkerbot_hand_ros/multi_device.py",
        PACKAGE_ROOT / "linkerbot_hand_ros/examples/angle_command.py",
        PACKAGE_ROOT / "linkerbot_hand_ros/examples/angle_controller.py",
        PACKAGE_ROOT / "linkerbot_hand_ros/examples/frequency_benchmark.py",
        PACKAGE_ROOT / "linkerbot_hand_ros/examples/state_monitor.py",
    )

    for path in entry_points:
        source = path.read_text(encoding="utf-8")
        assert "rclpy.try_shutdown()" in source
        assert "rclpy.shutdown()" not in source


def test_production_entry_points_keep_default_rclpy_signal_handling() -> None:
    for relative in ("node.py", "multi_device.py"):
        source = (PACKAGE_ROOT / "linkerbot_hand_ros" / relative).read_text()

        assert "rclpy.init(args=args)" in source
        assert "SignalHandlerOptions.NO" not in source


def test_driver_entry_points_spin_multi_threaded_and_tolerate_destroyed_entities():
    node_source = (PACKAGE_ROOT / "linkerbot_hand_ros/node.py").read_text(
        encoding="utf-8"
    )
    multi_source = (PACKAGE_ROOT / "linkerbot_hand_ros/multi_device.py").read_text(
        encoding="utf-8"
    )

    for source in (node_source, multi_source):
        assert "SingleThreadedExecutor" not in source
        main = _function_definition(source, "main")
        constructed = {
            call.func.id
            for call in ast.walk(main)
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        }
        methods = {
            call.func.attr
            for call in ast.walk(main)
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
        }
        assert "MultiThreadedExecutor" in constructed
        # A destroyed entity must not end the process for every other hand, so
        # no entry point calls spin() directly.
        assert "spin_until_shutdown" in constructed
        assert "spin" not in methods

    assert "num_threads=CALLBACK_GROUPS_PER_HAND" in node_source
    assert "CALLBACK_GROUPS_PER_HAND * len(nodes)" in multi_source


def _function_definition(source: str, name: str) -> ast.FunctionDef:
    for node in ast.parse(source).body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} is not defined at module level")


def test_launch_exposes_every_driver_parameter() -> None:
    launch_path = PACKAGE_ROOT / "launch/hand.launch.py"
    module = runpy.run_path(str(launch_path))
    declared = {name for name, _default, _type in module["_ARGUMENTS"]}

    assert declared == DRIVER_PARAMETERS
    source = launch_path.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("namespace"' in source
    assert 'DeclareLaunchArgument("autostart"' in source


def test_launch_does_not_require_distribution_specific_autostart_argument() -> None:
    source = (PACKAGE_ROOT / "linkerbot_hand_ros/discovery_launch.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    lifecycle_nodes = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LifecycleNode"
    ]

    assert len(lifecycle_nodes) == 1
    assert not any(
        keyword.arg == "autostart" for keyword in lifecycle_nodes[0].keywords
    )


def test_multi_hand_launch_preserves_each_internal_node_name() -> None:
    source = (PACKAGE_ROOT / "linkerbot_hand_ros/discovery_launch.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    multi_builder = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "build_multi_launch"
    )
    nodes = [
        node
        for node in ast.walk(multi_builder)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]

    assert len(nodes) == 1
    keywords = {keyword.arg for keyword in nodes[0].keywords}
    assert "name" not in keywords
    assert "namespace" not in keywords
    assert 'executable="hand_container"' in source
    assert "ParameterFile(str(path), allow_substs=True)" in source
    entrypoint = (PACKAGE_ROOT / "launch/multi_hand.launch.py").read_text(
        encoding="utf-8"
    )
    assert "function=build_multi_launch" in entrypoint


def test_ros_callbacks_contain_no_synchronous_device_reads() -> None:
    callback_sources = (
        "linkerbot_hand_ros/node.py",
        "linkerbot_hand_ros/command_interface.py",
        "linkerbot_hand_ros/driver_interfaces.py",
        "linkerbot_hand_ros/state_publisher.py",
        "linkerbot_hand_ros/diagnostics.py",
    )
    source = "\n".join(
        (PACKAGE_ROOT / path).read_text(encoding="utf-8") for path in callback_sources
    )

    for method in (
        "get_angles(",
        "get_speeds(",
        "get_force(",
        "get_temperatures(",
        "get_currents(",
        "get_faults(",
        ".wait(",
    ):
        assert method not in source
    assert "get_snapshot()" in source
    assert "get_health()" in source
    assert "start_polling(" in source


def test_old_adapter_and_configuration_implementations_are_removed() -> None:
    assert not (PACKAGE_ROOT / "linkerbot_hand_ros/adapter.py").exists()
    assert not (PACKAGE_ROOT / "linkerbot_hand_ros/configuration.py").exists()


def test_package_depends_on_hand_sdk_without_version_pin() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert '"linkerbot-hand"' in setup_source
    assert '"linkerbot>=' not in setup_source
    assert '"linkerbot-transport' not in setup_source


def test_package_registers_the_in_process_hand_container() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "hand_container = linkerbot_hand_ros.multi_device:main" in setup_source


def test_ros_source_uses_only_the_public_hand_import_path() -> None:
    sources = {
        path: path.read_text(encoding="utf-8")
        for path in (PACKAGE_ROOT / "linkerbot_hand_ros").rglob("*.py")
    }
    source = "\n".join(sources.values())

    assert "from linkerbot_hand" in source
    assert "from linkerbot.hand" not in source
    assert "MODEL_SPECS" not in source
    assert "Float64MultiArray" not in source
    assert "SensorType" not in source
    transport_importers = {
        path.name
        for path, contents in sources.items()
        if "from linkerbot_transport" in contents
    }
    assert transport_importers == {"transport_factory.py", "discovery_config.py"}
    config_tree = ast.parse(
        sources[PACKAGE_ROOT / "linkerbot_hand_ros/discovery_config.py"]
    )
    imports = [
        node
        for node in ast.walk(config_tree)
        if isinstance(node, ast.ImportFrom)
        and node.module.startswith("linkerbot_transport")
    ]
    assert [(node.module, [name.name for name in node.names]) for node in imports] == [
        ("linkerbot_transport.canfd", ["CANFDConfigOptions"])
    ]


def test_package_declares_direct_runtime_dependencies() -> None:
    package_source = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    for dependency in (
        "builtin_interfaces",
        "diagnostic_msgs",
        "launch",
        "launch_ros",
        "rcl_interfaces",
        "rclpy",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_source
    assert "<exec_depend>std_msgs</exec_depend>" not in package_source
    assert "<exec_depend>std_srvs</exec_depend>" not in package_source
    assert (
        '<exec_depend version_gte="2.0.0" version_lt="3.0.0">'
        "linkerbot_hand_msgs</exec_depend>"
    ) in package_source


def test_package_declares_ament_python_only_as_build_type() -> None:
    for package_name in ("linkerbot_hand_ros", "linkerbot_hand_gui"):
        manifest = PACKAGE_ROOT.parent / package_name / "package.xml"
        package = ET.parse(manifest).getroot()
        buildtool_dependencies = {
            dependency.text for dependency in package.findall("buildtool_depend")
        }

        assert "ament_python" not in buildtool_dependencies, manifest
        assert package.findtext("./export/build_type") == "ament_python", manifest


def test_rosdep_instructions_fail_on_unresolved_dependencies() -> None:
    documentation = tuple(
        PACKAGE_ROOT / "docs" / language / name
        for language in ("", "zh")
        for name in ("quick_start.md", "update.md")
    )

    for path in documentation:
        commands = [
            line
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.startswith("rosdep install ")
        ]
        assert commands
        assert all('--rosdistro "${ROS_DISTRO}"' in command for command in commands)
        assert all(" -r " not in command for command in commands)


def test_sdk_install_instructions_use_main_sources() -> None:
    documentation = tuple(
        PACKAGE_ROOT / "docs" / language / "quick_start.md" for language in ("", "zh")
    )

    for path in documentation:
        source = path.read_text(encoding="utf-8")
        commands = source.replace("\\\n", "")
        for repository, directory in (
            ("linkerbot-transport", "linkerbot-transport"),
            ("linkerbot-hand-python-sdk", "linkerbot-hand"),
        ):
            assert re.search(
                rf"git clone --branch main --depth 1\s+"
                rf"https://github\.com/linker-bot/{repository}\.git\s+{directory}",
                commands,
            ), path
            update_path = path.with_name("update.md")
            update_commands = update_path.read_text(encoding="utf-8").replace(
                "\\\n", ""
            )
            assert re.search(
                rf"git -C ~/linkerbot_deps/{directory} remote set-url origin\s+"
                rf"https://github\.com/linker-bot/{repository}\.git",
                update_commands,
            ), update_path
        assert "-e './linkerbot-transport[all]' -e ./linkerbot-hand" in source
        assert not re.search(r"linkerbot-(?:hand|transport)(?:\[all\])?[<>=]", source)


def test_python_and_ros_package_versions_match() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    package_source = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    messages_package_source = (
        PACKAGE_ROOT.parent / "linkerbot_hand_msgs/package.xml"
    ).read_text(encoding="utf-8")
    module_source = (PACKAGE_ROOT / "linkerbot_hand_ros/__init__.py").read_text(
        encoding="utf-8"
    )
    setup_version = re.search(r'version="([^"]+)"', setup_source)
    package_version = re.search(r"<version>([^<]+)</version>", package_source)
    messages_package_version = re.search(
        r"<version>([^<]+)</version>", messages_package_source
    )
    module_version = re.search(r'__version__ = "([^"]+)"', module_source)

    assert setup_version is not None
    assert package_version is not None
    assert messages_package_version is not None
    assert module_version is not None
    assert (
        setup_version.group(1)
        == package_version.group(1)
        == messages_package_version.group(1)
        == module_version.group(1)
    )
    # Keep all four version declarations consistent without fixing their value.
