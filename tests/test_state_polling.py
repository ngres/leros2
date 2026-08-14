# Copyright 2026 Nicolas Gres
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regression tests for StateComponent's polled subscriptions.

StateComponent polls its subscription with ``take_message`` instead of relying
on a callback. A spinning executor takes the same messages and dispatches them
to the component's no-op callback, so a state node that ends up in the executor
silently starves ``get_state``. These tests pin the two nodes apart.
"""

import multiprocessing as mp
import time
from dataclasses import dataclass

import pytest
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from leros2.common import ROS2Common
from leros2.components.joint_state import (
    JointConfig,
    JointStateComponent,
    JointStateComponentConfig,
)


TOPIC = "/leros2_test/joint_states"
PUBLISH_PERIOD = 0.02  # 50 Hz, deliberately faster than the polling consumer
POLL_PERIOD = 0.1  # 10 Hz, a plausible control rate
POLLS = 20
DISCOVERY_SETTLE = 2.0


@dataclass
class _Config:
    """Minimal stand-in for a robot/teleoperator config."""

    node_name: str


def _publisher_proc() -> None:
    """Publish a ramp on TOPIC until killed.

    The value must change every message: a constant is indistinguishable from
    the value StateComponent caches in ``_value``, which would make a starved
    consumer look healthy.
    """
    rclpy.init()
    node = rclpy.create_node("leros2_test_publisher")
    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    pub = node.create_publisher(JointState, TOPIC, qos)
    counter = 0

    def tick() -> None:
        nonlocal counter
        counter += 1
        msg = JointState()
        msg.name = ["joint_1"]
        msg.position = [(counter % 100) / 100.0]
        pub.publish(msg)

    node.create_timer(PUBLISH_PERIOD, tick)
    rclpy.spin(node)


def _make_common() -> tuple[ROS2Common, JointStateComponent]:
    comp = JointStateComponent(
        JointStateComponentConfig(topic=TOPIC, joints=[JointConfig(name="joint_1")])
    )
    return ROS2Common(_Config(node_name="leros2_test_consumer"), [comp]), comp


@pytest.fixture
def publisher():
    proc = mp.Process(target=_publisher_proc, daemon=True)
    proc.start()
    time.sleep(DISCOVERY_SETTLE)
    yield
    proc.terminate()
    proc.join(timeout=5.0)


def test_state_node_is_not_in_the_executor() -> None:
    """The state node must never be handed to the spinning executor.

    Deterministic guard: this is the mistake that starves the polling, and it
    fails here rather than as flaky state loss under load.
    """
    common, _ = _make_common()
    common.connect()
    try:
        executor_nodes = common._executor.get_nodes()
        assert common._node in executor_nodes
        assert common._state_node not in executor_nodes
    finally:
        common.disconnect()


def test_get_state_sees_live_messages(publisher) -> None:
    """Polling must actually retrieve messages, not fall back to the cache."""
    common, comp = _make_common()
    common.connect()

    takes = 0
    inner = comp._take

    def counting_take():
        nonlocal takes
        msg = inner()
        if msg is not None:
            takes += 1
        return msg

    comp._take = counting_take

    values = []
    try:
        for _ in range(POLLS):
            time.sleep(POLL_PERIOD)
            values.append(comp.get_state()["joint_1.pos"])
    finally:
        common.disconnect()

    # With the state node in the executor this lands around 20-50%; without it,
    # it is consistently at or near 100%. The threshold leaves room for the
    # discovery warm-up at the start of the run.
    assert takes >= 0.75 * POLLS, f"only {takes}/{POLLS} polls retrieved a message"
    assert len(set(values)) >= 0.75 * POLLS, "get_state returned stale cached values"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
