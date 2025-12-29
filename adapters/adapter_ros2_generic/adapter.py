"""ROS2 Generic Adapter.

Subscribes to configurable ROS2 topics and publishes decisions.
"""

from __future__ import annotations

import json
import logging
import uuid
from datetime import datetime
from typing import Any, Callable

from robops_edge_agent.adapter_base import AdapterConfig, RobOpsAdapter
from robops_edge_agent.events import (
    DecisionType,
    EventType,
    InferenceDecision,
    RobotOpsEvent,
)

logger = logging.getLogger(__name__)


class ROS2GenericAdapter(RobOpsAdapter):
    """Generic ROS2 adapter.

    Subscribes to configurable topics and publishes decisions.
    Works with any ROS2 message type that can be converted to JSON.

    Configuration:
    - subscribe_topics: List of topics to subscribe to
    - publish_topic: Topic for publishing decisions
    - event_field_mapping: Map ROS2 message fields to RobotOpsEvent fields
    """

    def __init__(self, config: AdapterConfig) -> None:
        """Initialize ROS2 adapter.

        Args:
            config: Adapter configuration.
        """
        self._config = config
        self._node: Any = None
        self._subscribers: list[Any] = []
        self._publisher: Any = None
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None

    @property
    def platform_name(self) -> str:
        return "ros2_generic"

    def connect(self) -> None:
        """Initialize ROS2 node and subscriptions.

        Note: Requires rclpy to be available and ros2 context initialized.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String

            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()

            # Create node
            node_name = self._config.connection_params.get("node_name", "moai_edge_agent")
            self._node = rclpy.create_node(node_name)

            # Create subscribers
            subscribe_topics = self._config.connection_params.get("subscribe_topics", ["/robot/events"])
            for topic in subscribe_topics:
                sub = self._node.create_subscription(
                    String,
                    topic,
                    self._ros_callback,
                    10,
                )
                self._subscribers.append(sub)
                logger.info(f"Subscribed to ROS2 topic: {topic}")

            # Create publisher
            publish_topic = self._config.connection_params.get("publish_topic", "/moai/decision")
            self._publisher = self._node.create_publisher(String, publish_topic, 10)
            logger.info(f"Publishing to ROS2 topic: {publish_topic}")

        except ImportError:
            logger.warning("ROS2 (rclpy) not available - ROS2 adapter running in mock mode")
            self._node = None

    def disconnect(self) -> None:
        """Shutdown ROS2 node."""
        if self._node:
            self._node.destroy_node()
            self._node = None
        self._subscribers = []
        self._publisher = None

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        """Set callback for incoming events.

        Args:
            callback: Function to call with normalized events.
        """
        self._event_callback = callback

    def _ros_callback(self, msg: Any) -> None:
        """Handle incoming ROS2 message.

        Args:
            msg: ROS2 message (std_msgs/String with JSON).
        """
        try:
            # Parse JSON from String message
            data = json.loads(msg.data)
            event = self.ingest_event(data)

            if self._event_callback:
                self._event_callback(event)

        except Exception as e:
            logger.error(f"Error processing ROS2 message: {e}")

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert ROS2 message data to RobotOpsEvent.

        Args:
            raw_event: Parsed JSON from ROS2 message.

        Returns:
            Normalized RobotOpsEvent.
        """
        mappings = self._config.event_mappings

        # Extract fields using mappings
        event_id = raw_event.get(mappings.get("event_id", "id"), str(uuid.uuid4()))
        event_type_str = raw_event.get(mappings.get("event_type", "type"), "unknown")
        text_content = raw_event.get(mappings.get("text_content", "message"), None)
        robot_id = raw_event.get(mappings.get("robot_id", "robot_id"), None)

        # Map event type
        event_type_map = {
            "incident": EventType.INCIDENT,
            "task": EventType.TASK_REQUEST,
            "compliance": EventType.COMPLIANCE_CHECK,
            "sop": EventType.SOP_QUERY,
            "alert": EventType.ALERT,
        }
        event_type = event_type_map.get(event_type_str.lower(), EventType.UNKNOWN)

        return RobotOpsEvent(
            event_id=event_id,
            event_type=event_type,
            timestamp=datetime.now(),
            source_platform="ros2",
            robot_id=robot_id,
            text_content=text_content,
            metadata=raw_event,
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Publish decision to ROS2 topic.

        Args:
            decision: Inference decision to publish.
        """
        if self._publisher is None:
            logger.warning("ROS2 publisher not available")
            return

        try:
            from std_msgs.msg import String

            # Convert decision to JSON
            decision_data = {
                "decision_id": decision.decision_id,
                "event_id": decision.event_id,
                "decision_type": decision.decision_type.value,
                "timestamp": decision.timestamp.isoformat(),
                "predicted_class": decision.predicted_class,
                "class_label": decision.class_label,
                "confidence": decision.confidence,
                "logits": decision.logits,
                "probabilities": decision.probabilities,
            }

            msg = String()
            msg.data = json.dumps(decision_data)
            self._publisher.publish(msg)

            logger.info(f"Published decision to ROS2: {decision.decision_id}")

        except Exception as e:
            logger.error(f"Error publishing to ROS2: {e}")

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        """Process one round of ROS2 callbacks.

        Args:
            timeout_sec: Timeout for waiting.
        """
        if self._node:
            import rclpy

            rclpy.spin_once(self._node, timeout_sec=timeout_sec)


# Default configuration for ROS2 adapter
DEFAULT_ROS2_CONFIG = AdapterConfig(
    platform_name="ros2_generic",
    connection_params={
        "node_name": "moai_edge_agent",
        "subscribe_topics": ["/robot/events", "/robot/incidents"],
        "publish_topic": "/moai/decision",
    },
    event_mappings={
        "event_id": "id",
        "event_type": "type",
        "text_content": "message",
        "robot_id": "robot_id",
    },
)
