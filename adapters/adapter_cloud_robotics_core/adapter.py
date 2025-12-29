"""Google Cloud Robotics Core Adapter.

Deploys MOAI as K8s service with robot-side agent.
"""

from __future__ import annotations

import json
import logging
import os
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


class CloudRoboticsCoreAdapter(RobOpsAdapter):
    """Google Cloud Robotics Core adapter.

    Integration:
    - Deploy moai-service as K8s workload in cloud robotics cluster
    - Robot-side agent communicates via K8s service discovery
    - Uses ROS2 for local robot events

    Configuration:
    - MOAI_SERVICE_HOST: K8s service endpoint
    - ROBOT_ID: Robot identifier
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._node: Any = None
        self._subscribers: list[Any] = []
        self._publisher: Any = None
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None
        self._robot_id = config.connection_params.get(
            "robot_id", os.environ.get("ROBOT_ID", "robot-unknown")
        )

    @property
    def platform_name(self) -> str:
        return "cloud_robotics_core"

    def connect(self) -> None:
        """Initialize ROS2 node for robot-side agent."""
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init()

            node_name = self._config.connection_params.get(
                "node_name", f"moai_crc_{self._robot_id}"
            )
            self._node = rclpy.create_node(node_name)

            # Subscribe to robot events
            topics = self._config.connection_params.get("subscribe_topics", [
                "/robot/events",
                "/robot/incidents",
            ])

            for topic in topics:
                sub = self._node.create_subscription(
                    String, topic, self._ros_callback, 10
                )
                self._subscribers.append(sub)
                logger.info(f"CRC: Subscribed to {topic}")

            # Publisher for decisions
            pub_topic = self._config.connection_params.get(
                "publish_topic", "/moai/decision"
            )
            self._publisher = self._node.create_publisher(String, pub_topic, 10)
            logger.info(f"CRC: Publishing to {pub_topic}")

        except ImportError:
            logger.warning("ROS2 not available - CRC adapter in mock mode")

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
            logger.error(f"CRC callback error: {e}")

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert robot event to RobotOpsEvent."""
        event_type_str = raw_event.get("type", "unknown")

        type_map = {
            "incident": EventType.INCIDENT,
            "task": EventType.TASK_REQUEST,
            "alert": EventType.ALERT,
        }

        return RobotOpsEvent(
            event_id=raw_event.get("id", str(uuid.uuid4())),
            event_type=type_map.get(event_type_str.lower(), EventType.UNKNOWN),
            timestamp=datetime.now(),
            source_platform="cloud_robotics_core",
            robot_id=self._robot_id,
            text_content=raw_event.get("message", raw_event.get("description")),
            metadata=raw_event,
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Publish decision via ROS2."""
        if self._publisher is None:
            return

        try:
            from std_msgs.msg import String

            msg = String()
            msg.data = json.dumps({
                "decision_id": decision.decision_id,
                "event_id": decision.event_id,
                "robot_id": self._robot_id,
                "classification": decision.class_label,
                "confidence": decision.confidence,
                "timestamp": decision.timestamp.isoformat(),
            })
            self._publisher.publish(msg)
            logger.info(f"CRC: Published decision {decision.decision_id}")

        except Exception as e:
            logger.error(f"CRC writeback error: {e}")


DEFAULT_CRC_CONFIG = AdapterConfig(
    platform_name="cloud_robotics_core",
    connection_params={
        "node_name": "moai_crc_agent",
        "subscribe_topics": ["/robot/events", "/robot/incidents"],
        "publish_topic": "/moai/decision",
        "robot_id": os.environ.get("ROBOT_ID", "robot-001"),
    },
)
