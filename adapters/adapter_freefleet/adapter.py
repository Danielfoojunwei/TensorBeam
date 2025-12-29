"""free_fleet Adapter.

Integrates with free_fleet (Open-RMF fleet adapter) via Zenoh or ROS2.
"""

from __future__ import annotations

import json
import logging
import uuid
from datetime import datetime
from typing import Any, Callable

from robops_edge_agent.adapter_base import AdapterConfig, RobOpsAdapter
from robops_edge_agent.events import (
    EventType,
    InferenceDecision,
    RobotOpsEvent,
)

logger = logging.getLogger(__name__)


class FreeFleetAdapter(RobOpsAdapter):
    """free_fleet adapter.

    free_fleet uses Zenoh for robot-adapter communication.
    Can also integrate via ROS2 topics.

    Topics:
    - /free_fleet/robot_state (robot states)
    - /free_fleet/mode_request (mode commands)
    - /free_fleet/path_request (path commands)
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._node: Any = None
        self._subscribers: list[Any] = []
        self._publisher: Any = None
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None

    @property
    def platform_name(self) -> str:
        return "free_fleet"

    def connect(self) -> None:
        """Initialize ROS2 subscriptions for free_fleet."""
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init()

            node_name = self._config.connection_params.get("node_name", "moai_free_fleet")
            self._node = rclpy.create_node(node_name)

            # Subscribe to robot states
            topics = self._config.connection_params.get("topics", [
                "/free_fleet/robot_state",
            ])

            for topic in topics:
                sub = self._node.create_subscription(
                    String, topic, self._ros_callback, 10
                )
                self._subscribers.append(sub)
                logger.info(f"Subscribed to free_fleet topic: {topic}")

            # Publisher for mode requests
            pub_topic = self._config.connection_params.get("publish_topic", "/moai/fleet_decision")
            self._publisher = self._node.create_publisher(String, pub_topic, 10)

        except ImportError:
            logger.warning("ROS2 not available - free_fleet adapter in mock mode")

    def disconnect(self) -> None:
        if self._node:
            self._node.destroy_node()
            self._node = None

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        self._event_callback = callback

    def _ros_callback(self, msg: Any) -> None:
        try:
            data = json.loads(msg.data)
            event = self.ingest_event(data)
            if self._event_callback:
                self._event_callback(event)
        except Exception as e:
            logger.error(f"free_fleet callback error: {e}")

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert free_fleet robot state to event."""
        robot_name = raw_event.get("name", "unknown")
        mode = raw_event.get("mode", {}).get("mode", 0)

        # Mode mapping
        mode_names = {0: "IDLE", 1: "CHARGING", 2: "MOVING", 3: "PAUSED", 4: "WAITING"}
        mode_str = mode_names.get(mode, "UNKNOWN")

        text_content = f"Robot {robot_name} in {mode_str} mode"

        return RobotOpsEvent(
            event_id=str(uuid.uuid4()),
            event_type=EventType.STATUS_UPDATE,
            timestamp=datetime.now(),
            source_platform="free_fleet",
            robot_id=robot_name,
            fleet_id=raw_event.get("fleet_name"),
            text_content=text_content,
            location=str(raw_event.get("location", {})),
            metadata=raw_event,
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Publish decision to free_fleet."""
        if self._publisher is None:
            return

        try:
            from std_msgs.msg import String

            msg = String()
            msg.data = json.dumps({
                "decision_id": decision.decision_id,
                "classification": decision.class_label,
                "confidence": decision.confidence,
            })
            self._publisher.publish(msg)

        except Exception as e:
            logger.error(f"free_fleet writeback error: {e}")


DEFAULT_FREEFLEET_CONFIG = AdapterConfig(
    platform_name="free_fleet",
    connection_params={
        "node_name": "moai_free_fleet",
        "topics": ["/free_fleet/robot_state"],
        "publish_topic": "/moai/fleet_decision",
    },
)
