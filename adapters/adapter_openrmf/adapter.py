"""Open-RMF Adapter.

Integrates with Open-RMF via ROS2 topics for task/traffic management.
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


class OpenRMFAdapter(RobOpsAdapter):
    """Open-RMF adapter for fleet task management.

    Subscribes to RMF task states and publishes decision constraints.

    RMF Topics:
    - /task_states (rmf_task_msgs/TaskState)
    - /fleet_states (rmf_fleet_msgs/FleetState)
    - /door_states, /lift_states (infrastructure)

    Integration points:
    - Task dispatch: Add risk/compliance labels to tasks
    - Traffic: Generate constraints for scheduling
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._node: Any = None
        self._subscribers: list[Any] = []
        self._publisher: Any = None
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None

    @property
    def platform_name(self) -> str:
        return "open_rmf"

    def connect(self) -> None:
        """Initialize ROS2 node and RMF subscriptions."""
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init()

            node_name = self._config.connection_params.get("node_name", "moai_rmf_adapter")
            self._node = rclpy.create_node(node_name)

            # Subscribe to RMF topics (using String wrapper for JSON)
            rmf_topics = self._config.connection_params.get("rmf_topics", [
                "/task_states",
                "/fleet_states",
            ])

            for topic in rmf_topics:
                sub = self._node.create_subscription(
                    String,
                    topic,
                    lambda msg, t=topic: self._rmf_callback(msg, t),
                    10,
                )
                self._subscribers.append(sub)
                logger.info(f"Subscribed to RMF topic: {topic}")

            # Publisher for MOAI decisions
            decision_topic = self._config.connection_params.get(
                "decision_topic", "/moai/rmf_constraints"
            )
            self._publisher = self._node.create_publisher(String, decision_topic, 10)
            logger.info(f"Publishing RMF constraints to: {decision_topic}")

        except ImportError:
            logger.warning("ROS2/RMF not available - running in mock mode")
            self._node = None

    def disconnect(self) -> None:
        if self._node:
            self._node.destroy_node()
            self._node = None
        self._subscribers = []
        self._publisher = None

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        self._event_callback = callback

    def _rmf_callback(self, msg: Any, topic: str) -> None:
        try:
            data = json.loads(msg.data)
            data["_source_topic"] = topic
            event = self.ingest_event(data)
            if self._event_callback:
                self._event_callback(event)
        except Exception as e:
            logger.error(f"Error processing RMF message from {topic}: {e}")

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert RMF message to RobotOpsEvent."""
        source_topic = raw_event.get("_source_topic", "unknown")

        # Determine event type from topic
        if "task" in source_topic:
            event_type = EventType.TASK_REQUEST
            text_content = raw_event.get("task_profile", {}).get("description", "")
            task_id = raw_event.get("booking", {}).get("id", str(uuid.uuid4()))
        elif "fleet" in source_topic:
            event_type = EventType.STATUS_UPDATE
            text_content = f"Fleet {raw_event.get('name', 'unknown')} status update"
            task_id = None
        else:
            event_type = EventType.UNKNOWN
            text_content = json.dumps(raw_event)[:200]
            task_id = None

        return RobotOpsEvent(
            event_id=str(uuid.uuid4()),
            event_type=event_type,
            timestamp=datetime.now(),
            source_platform="open_rmf",
            robot_id=raw_event.get("robot_name"),
            fleet_id=raw_event.get("fleet_name"),
            text_content=text_content,
            task_id=task_id,
            metadata=raw_event,
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Publish decision as RMF constraint."""
        if self._publisher is None:
            logger.warning("RMF publisher not available")
            return

        try:
            from std_msgs.msg import String

            # Format as RMF-compatible constraint
            constraint = {
                "decision_id": decision.decision_id,
                "event_id": decision.event_id,
                "constraint_type": "moai_classification",
                "classification": decision.class_label,
                "confidence": decision.confidence,
                "requires_review": decision.confidence < 0.8 if decision.confidence else True,
                "timestamp": decision.timestamp.isoformat(),
            }

            msg = String()
            msg.data = json.dumps(constraint)
            self._publisher.publish(msg)
            logger.info(f"Published RMF constraint: {decision.decision_id}")

        except Exception as e:
            logger.error(f"Error publishing RMF constraint: {e}")


DEFAULT_OPENRMF_CONFIG = AdapterConfig(
    platform_name="open_rmf",
    connection_params={
        "node_name": "moai_rmf_adapter",
        "rmf_topics": ["/task_states", "/fleet_states"],
        "decision_topic": "/moai/rmf_constraints",
    },
    event_mappings={},
)
