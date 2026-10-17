"""Verify one-shot launch handshakes without DDS or wall-clock deadlines."""

import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.task import Future

from linkerbot_hand_ros import discovery_launch

CONFIGURE = Transition.TRANSITION_CONFIGURE
ACTIVATE = Transition.TRANSITION_ACTIVATE


class _LogicalClock:
    def __init__(self):
        self.now = 0.0
        self.delays = []
        self.on_sleep = lambda: None

    def time(self):
        return self.now

    async def sleep(self, delay):
        assert delay >= 0
        self.delays.append(delay)
        assert len(self.delays) < 100, "handshake exceeded its logical deadline"
        self.now += delay
        self.on_sleep()
        # Yield to real task cancellation without spending the requested time.
        await asyncio.sleep(0)


class _Client:
    """Keep real rclpy Futures and emulate its pending-request ownership."""

    def __init__(self):
        self.srv_name = "/robot/left/linkerbot_hand/change_state"
        self.ready = True
        self.responses = [True, True]
        self.requests = []
        self.futures = []
        self.pending = set()
        self.removed = []
        self.on_request = lambda: None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request.transition.id)
        self.on_request()
        outcome = self.responses.pop(0)
        if isinstance(outcome, BaseException):
            raise outcome
        future = Future()
        self.futures.append(future)
        self.pending.add(future)
        future.add_done_callback(self.remove_pending_request)
        if outcome == "cancelled":
            future.cancel()
        elif outcome == "exception":
            future.set_exception(RuntimeError("injected response failure"))
        elif outcome is not None:
            future.set_result(ChangeState.Response(success=outcome))
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)
        self.pending.discard(future)


class _Node:
    def __init__(self, client):
        self.client = client
        self.alive = True
        self.context = SimpleNamespace(ok=lambda: self.alive)
        self.created = []
        self.destroyed = []

    def create_client(self, service_type, name):
        assert service_type is ChangeState
        self.created.append(name)
        return self.client

    def destroy_client(self, client):
        self.destroyed.append(client)
        return True


@pytest.fixture
def handshake(monkeypatch):
    clock = _LogicalClock()
    client = _Client()
    node = _Node(client)
    context = SimpleNamespace(asyncio_loop=clock, is_shutdown=False)
    driver = SimpleNamespace(node_name="/robot/left/linkerbot_hand")
    logger = Mock()
    monkeypatch.setattr(discovery_launch, "_AUTOSTART_TIMEOUT_S", 0.05)
    monkeypatch.setattr(discovery_launch, "_AUTOSTART_POLL_S", 0.02)
    # Replace only the production module's reference, never global asyncio.sleep.
    monkeypatch.setattr(
        discovery_launch,
        "asyncio",
        SimpleNamespace(sleep=clock.sleep, CancelledError=asyncio.CancelledError),
    )
    monkeypatch.setattr(discovery_launch, "get_ros_node", lambda _: node)
    monkeypatch.setattr(discovery_launch, "get_logger", lambda _: logger)

    def run():
        asyncio.run(discovery_launch._autostart_once(context, driver=driver))

    return SimpleNamespace(
        clock=clock,
        client=client,
        node=node,
        context=context,
        driver=driver,
        logger=logger,
        run=run,
    )


def _assert_released(handshake):
    assert not handshake.client.pending
    assert handshake.node.destroyed == [handshake.client]


def test_success_configures_once_then_activates_once_and_releases_client(handshake):
    handshake.run()

    assert handshake.client.requests == [CONFIGURE, ACTIVATE]
    assert handshake.node.created == ["/robot/left/linkerbot_hand/change_state"]
    _assert_released(handshake)


@pytest.mark.parametrize("outcome", [False, "exception", "cancelled"])
def test_failed_configure_never_activates(handshake, outcome):
    handshake.client.responses = [outcome]
    handshake.run()

    assert handshake.client.requests == [CONFIGURE]
    _assert_released(handshake)


@pytest.mark.parametrize("stage", ["configure", "activate"])
def test_request_submission_exception_stops_without_resending(handshake, stage):
    handshake.client.responses = [RuntimeError("injected send failure")]
    expected = [CONFIGURE]
    if stage == "activate":
        handshake.client.responses.insert(0, True)
        expected.append(ACTIVATE)

    handshake.run()

    assert handshake.client.requests == expected
    _assert_released(handshake)


@pytest.mark.parametrize("outcome", [False, "exception", "cancelled"])
def test_failed_activate_never_repeats_a_lifecycle_request(handshake, outcome):
    handshake.client.responses = [True, outcome]
    handshake.run()

    assert handshake.client.requests == [CONFIGURE, ACTIVATE]
    _assert_released(handshake)


@pytest.mark.parametrize("stage", ["configure", "activate"])
def test_timeout_removes_pending_request_and_late_success_cannot_restart(
    handshake, stage
):
    handshake.client.responses = [None]
    expected = [CONFIGURE]
    if stage == "activate":
        handshake.client.responses.insert(0, True)
        expected.append(ACTIVATE)

    handshake.run()

    assert handshake.client.requests == expected
    pending = handshake.client.futures[-1]
    assert pending.cancelled()
    assert pending in handshake.client.removed
    assert handshake.clock.now <= 0.05
    _assert_released(handshake)
    # A response already taken by an executor can arrive after local timeout.
    pending.set_result(ChangeState.Response(success=True))
    assert handshake.client.requests == expected


def test_unavailable_service_spends_only_total_budget_and_sends_nothing(handshake):
    handshake.client.ready = False
    handshake.run()

    assert handshake.client.requests == []
    assert handshake.clock.delays
    assert handshake.clock.now <= 0.05
    _assert_released(handshake)


@pytest.mark.parametrize("stop", ["launch", "ros_context"])
def test_already_shutdown_creates_no_client_or_request(handshake, stop):
    if stop == "launch":
        handshake.context.is_shutdown = True
    else:
        handshake.node.alive = False

    handshake.run()

    assert handshake.node.created == []
    assert handshake.node.destroyed == []
    assert handshake.client.requests == []
    assert handshake.clock.delays == []


@pytest.mark.parametrize("stop", ["launch", "ros_context"])
def test_shutdown_while_awaiting_configure_abandons_activation(handshake, stop):
    handshake.client.responses = [None]

    def shutdown():
        if stop == "launch":
            handshake.context.is_shutdown = True
        else:
            handshake.node.alive = False

    handshake.clock.on_sleep = shutdown
    handshake.run()

    assert handshake.client.requests == [CONFIGURE]
    pending = handshake.client.futures[0]
    assert pending.cancelled()
    assert len(handshake.clock.delays) == 1
    _assert_released(handshake)
    pending.set_result(ChangeState.Response(success=True))
    assert handshake.client.requests == [CONFIGURE]


def test_shutdown_on_configure_response_prevents_activate(handshake):
    def shutdown():
        handshake.context.is_shutdown = True

    handshake.client.on_request = shutdown
    handshake.run()

    assert handshake.client.requests == [CONFIGURE]
    _assert_released(handshake)


def test_coroutine_cancellation_cleans_request_and_late_success_cannot_activate(
    handshake,
):
    handshake.client.responses = [None]

    async def exercise():
        task = asyncio.create_task(
            discovery_launch._autostart_once(handshake.context, driver=handshake.driver)
        )
        handshake.clock.on_sleep = task.cancel
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(exercise())

    assert handshake.client.requests == [CONFIGURE]
    pending = handshake.client.futures[0]
    assert pending.cancelled()
    _assert_released(handshake)
    pending.set_result(ChangeState.Response(success=True))
    assert handshake.client.requests == [CONFIGURE]


def test_waiting_for_service_uses_the_same_budget_as_configure(handshake):
    handshake.client.ready = False
    handshake.client.responses = [None]

    def discover():
        if handshake.clock.now >= 0.04:
            handshake.client.ready = True

    handshake.clock.on_sleep = discover
    handshake.run()

    assert handshake.client.requests == [CONFIGURE]
    assert handshake.clock.now <= 0.05
    _assert_released(handshake)


def test_pending_configure_must_finish_before_activate_is_sent(handshake):
    handshake.client.responses = [None, True]

    def reply():
        assert handshake.client.requests == [CONFIGURE]
        handshake.client.futures[0].set_result(ChangeState.Response(success=True))

    handshake.clock.on_sleep = reply
    handshake.run()

    assert handshake.clock.delays
    assert handshake.client.requests == [CONFIGURE, ACTIVATE]
    _assert_released(handshake)


def test_activation_observation_does_not_reset_configure_budget(handshake):
    handshake.client.responses = [None, None]

    def configure_response():
        if len(handshake.client.requests) == 1 and handshake.clock.now >= 0.04:
            handshake.client.futures[0].set_result(ChangeState.Response(success=True))

    handshake.clock.on_sleep = configure_response
    handshake.run()

    assert handshake.client.requests == [CONFIGURE, ACTIVATE]
    assert handshake.clock.now <= 0.05
    assert handshake.client.futures[-1].cancelled()
    _assert_released(handshake)


def test_pending_cleanup_failure_still_cancels_and_destroys_client(handshake):
    handshake.client.responses = [None]
    remove_pending = handshake.client.remove_pending_request
    removal_failed = False

    def fail_removal(future):
        nonlocal removal_failed
        if not removal_failed:
            removal_failed = True
            raise RuntimeError("injected pending cleanup failure")
        remove_pending(future)

    handshake.client.remove_pending_request = fail_removal
    handshake.run()

    assert handshake.client.requests == [CONFIGURE]
    assert handshake.client.futures[0].cancelled()
    _assert_released(handshake)
    handshake.logger.warning.assert_called_once()
    warning = handshake.logger.warning.call_args.args[0]
    assert "could not discard autostart request" in warning
    assert "injected pending cleanup failure" in warning


def test_shutdown_after_adapter_destroyed_client_keeps_cleanup_idempotent(handshake):
    handshake.client.responses = [None]

    def adapter_shutdown():
        handshake.context.is_shutdown = True
        # ROSAdapter owns the shared node and may have destroyed our client first.
        handshake.node.destroy_client = Mock(return_value=False)

    handshake.clock.on_sleep = adapter_shutdown
    handshake.run()

    assert handshake.client.requests == [CONFIGURE]
    assert handshake.client.futures[0].cancelled()
    assert not handshake.client.pending
    handshake.node.destroy_client.assert_called_once_with(handshake.client)
