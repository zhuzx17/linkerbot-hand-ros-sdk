"""Run production lifecycle startup against a real driver with an in-memory SDK."""

import asyncio
import subprocess
import sys
import textwrap
from pathlib import Path
from uuid import uuid4

import pytest
import rclpy
from launch import LaunchContext, LaunchDescription, LaunchService
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import StateTransition
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from test_discovery_launch import _hand_launch

from linkerbot_hand_ros import discovery_launch


@pytest.mark.parametrize(
    "autostart,fail_initial_configure,drop_initial_state_response,"
    "drop_transition_events,drop_configure_response",
    [
        ("true", False, False, False, False),
        ("false", False, False, False, False),
        ("true", True, False, False, False),
        ("false", False, True, False, False),
        ("true", False, False, True, False),
        ("true", False, False, False, True),
    ],
)
def test_manual_launch_respects_autostart_and_stays_inactive_after_deactivation(
    autostart,
    fail_initial_configure,
    drop_initial_state_response,
    drop_transition_events,
    drop_configure_response,
):
    # Cross-process discovery needs fresh DDS participants on both sides.
    # Reusing the suite's process after many init/shutdown cycles can lose the
    # initial transition event even though the same check passes in isolation.
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).resolve()),
            autostart,
            str(fail_initial_configure),
            str(drop_initial_state_response),
            str(drop_transition_events),
            str(drop_configure_response),
        ],
        capture_output=True,
        text=True,
        timeout=45.0,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _check_manual_launch(
    monkeypatch,
    autostart,
    fail_initial_configure,
    drop_initial_state_response,
    drop_transition_events,
    drop_configure_response,
):
    # Replace only the executable; parameters, ROS launch events and the driver
    # lifecycle remain real. DeviceBuilder never opens a physical adapter.
    child = textwrap.dedent(
        f"""
        import sys
        sys.path.insert(0, {str(Path(__file__).parent)!r})
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from linkerbot_hand_ros.executor import MultiThreadedExecutor
        from fakes_v1 import DeviceBuilder
        from linkerbot_hand_ros.node import (
            CALLBACK_GROUPS_PER_HAND, LinkerbotHandNode, spin_until_shutdown,
        )
        rclpy.init()
        builder = DeviceBuilder()
        fail_configure = {fail_initial_configure!r}
        def device_factory(configuration):
            global fail_configure
            if fail_configure:
                fail_configure = False
                raise RuntimeError("injected initial configuration failure")
            return builder(configuration)
        node = LinkerbotHandNode(device_factory=device_factory)
        if {drop_configure_response!r}:
            change_service = next(
                service for service in node.services
                if service.srv_name.endswith("/change_state")
            )
            send_change_response = change_service.send_response
            drop_change_response = True
            def send_configuration_response(response, header):
                global drop_change_response
                if drop_change_response:
                    drop_change_response = False
                    return
                send_change_response(response, header)
            change_service.send_response = send_configuration_response
        if {drop_initial_state_response!r}:
            state_service = next(
                service for service in node.services
                if service.srv_name.endswith("/get_state")
            )
            send_response = state_service.send_response
            drop_response = True
            def send_state_response(response, header):
                global drop_response
                if drop_response:
                    drop_response = False
                    return
                send_response(response, header)
            state_service.send_response = send_state_response
        executor = MultiThreadedExecutor(num_threads=CALLBACK_GROUPS_PER_HAND)
        executor.add_node(node)
        try:
            spin_until_shutdown(executor, node.get_logger())
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
        """
    )

    def driver(**options):
        options.update(package=None, executable=sys.executable, arguments=["-c", child])
        return LifecycleNode(**options)

    monkeypatch.setattr(discovery_launch, "LifecycleNode", driver)
    if drop_transition_events:
        # Deliberately lose launch's lifecycle notifications. Service results
        # must still control startup, independently of this event stream.
        emit_event = LaunchContext.emit_event_sync

        def emit_without_transitions(context, event):
            if not isinstance(event, StateTransition):
                return emit_event(context, event)

        monkeypatch.setattr(LaunchContext, "emit_event_sync", emit_without_transitions)
    # Use a new name on every run, including repeats of the same case. A graph
    # announcement retained by another participant must not make an old
    # driver's services appear ready before this driver's readers are matched.
    namespace = f"/launch_compat_{uuid4().hex}"
    context, actions = _hand_launch({"autostart": autostart, "namespace": namespace})
    startup = actions[0].event_handler.describe()[1][0] if autostart == "true" else None

    async def exercise():
        launch = LaunchService()
        launch.context.launch_configurations.update(context.launch_configurations)
        launch.include_launch_description(LaunchDescription(actions))
        task = asyncio.create_task(launch.run_async())
        rclpy.init()
        caller = Node("lifecycle_launch_test_client")
        executor = SingleThreadedExecutor()
        executor.add_node(caller)
        prefix = f"{namespace}/linkerbot_hand"
        get_state = caller.create_client(GetState, f"{prefix}/get_state")
        change_state = caller.create_client(ChangeState, f"{prefix}/change_state")
        initial_state_requests = 0

        async def wait_until(predicate, *, timeout=10.0):
            deadline = asyncio.get_running_loop().time() + timeout
            while not predicate():
                assert not task.done(), "driver launch exited before verification"
                assert asyncio.get_running_loop().time() < deadline, "lifecycle timeout"
                executor.spin_once(timeout_sec=0.0)
                await asyncio.sleep(0.01)

        async def call(client, request, *, initial_discovery=False):
            nonlocal initial_state_requests
            await wait_until(client.service_is_ready)
            deadline = asyncio.get_running_loop().time() + 10.0
            while True:
                if initial_discovery:
                    initial_state_requests += 1
                future = client.call_async(request)
                attempt_deadline = (
                    min(deadline, asyncio.get_running_loop().time() + 1.0)
                    if initial_discovery
                    else deadline
                )
                while not future.done():
                    assert not task.done(), "driver launch exited before verification"
                    if asyncio.get_running_loop().time() >= attempt_deadline:
                        break
                    executor.spin_once(timeout_sec=0.0)
                    await asyncio.sleep(0.01)
                if future.done():
                    return future.result()
                client.remove_pending_request(future)
                future.cancel()
                assert (
                    initial_discovery and asyncio.get_running_loop().time() < deadline
                ), f"service timeout: {client.srv_name}"

        async def state(*, initial_discovery=False):
            return (
                await call(
                    get_state, GetState.Request(), initial_discovery=initial_discovery
                )
            ).current_state.id

        async def transition(identifier):
            response = await call(
                change_state,
                ChangeState.Request(transition=Transition(id=identifier)),
            )
            assert response.success

        try:
            # Readiness of the request endpoint can precede discovery of the
            # response reader. Retry only this initial idempotent query within
            # the existing deadline; lifecycle changes are never resent.
            initial_state = await state(initial_discovery=True)
            if drop_initial_state_response:
                assert initial_state_requests >= 2
            if startup is not None:
                # Completion belongs to this one startup request sequence.
                # A volatile state event is not an acknowledgement of it.
                await wait_until(lambda: startup.get_asyncio_future() is not None)
                startup_task = startup.get_asyncio_future()
                await wait_until(
                    startup_task.done,
                    timeout=discovery_launch._AUTOSTART_TIMEOUT_S + 5.0,
                )
                assert not startup_task.cancelled()
                assert startup_task.exception() is None
            if fail_initial_configure:
                assert await state() == State.PRIMARY_STATE_UNCONFIGURED
                await transition(Transition.TRANSITION_CONFIGURE)
                # Allow launch to process the successful configure event. An
                # aborted autostart must not activate this manual retry.
                await asyncio.sleep(0.3)
                assert await state() == State.PRIMARY_STATE_INACTIVE
                await transition(Transition.TRANSITION_ACTIVATE)
                assert await state() == State.PRIMARY_STATE_ACTIVE
            elif drop_configure_response:
                # Configure executed, but its result was never acknowledged.
                # Autostart must finish without opening hardware or resending.
                assert await state() == State.PRIMARY_STATE_INACTIVE
                await transition(Transition.TRANSITION_CLEANUP)
                await transition(Transition.TRANSITION_CONFIGURE)
                await asyncio.sleep(0.3)
                assert await state() == State.PRIMARY_STATE_INACTIVE
                await transition(Transition.TRANSITION_ACTIVATE)
                assert await state() == State.PRIMARY_STATE_ACTIVE
            elif autostart == "true":
                assert await state() == State.PRIMARY_STATE_ACTIVE
            else:
                assert initial_state == State.PRIMARY_STATE_UNCONFIGURED
                await transition(Transition.TRANSITION_CONFIGURE)
                assert await state() == State.PRIMARY_STATE_INACTIVE
                await transition(Transition.TRANSITION_ACTIVATE)
                assert await state() == State.PRIMARY_STATE_ACTIVE

            await transition(Transition.TRANSITION_DEACTIVATE)
            await asyncio.sleep(0.3)
            assert await state() == State.PRIMARY_STATE_INACTIVE
            await transition(Transition.TRANSITION_CLEANUP)
            assert await state() == State.PRIMARY_STATE_UNCONFIGURED
            await transition(Transition.TRANSITION_CONFIGURE)
            await asyncio.sleep(0.3)
            assert await state() == State.PRIMARY_STATE_INACTIVE
            await transition(Transition.TRANSITION_CLEANUP)
        finally:
            shutdown = launch.shutdown()
            if shutdown is not None:
                await shutdown
            try:
                assert await asyncio.wait_for(task, timeout=10.0) == 0
            finally:
                executor.shutdown()
                caller.destroy_node()
                rclpy.try_shutdown()

    asyncio.run(exercise())


if __name__ == "__main__":
    with pytest.MonkeyPatch.context() as patch:
        _check_manual_launch(
            patch,
            sys.argv[1],
            sys.argv[2] == "True",
            sys.argv[3] == "True",
            sys.argv[4] == "True",
            sys.argv[5] == "True",
        )
