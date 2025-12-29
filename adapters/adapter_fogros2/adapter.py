"""FogROS2 Adapter.

Guides deployment of MOAI components across cloud resources.
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime
from typing import Any, Callable

from robops_edge_agent.adapter_base import AdapterConfig, RobOpsAdapter
from robops_edge_agent.events import (
    EventType,
    InferenceDecision,
    RobotOpsEvent,
)

logger = logging.getLogger(__name__)


class FogROS2Adapter(RobOpsAdapter):
    """FogROS2 adapter.

    FogROS2 deploys ROS2 nodes to cloud resources.
    This adapter provides integration guidance for MOAI.

    Key consideration:
    - Secret key must stay on trusted edge, not cloud
    - moai-service can run in cloud
    - edge agent with decryption runs on robot

    Usage:
    - Configure FogROS2 launch to deploy moai-service to cloud
    - Keep edge agent local for key custody
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._node: Any = None
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None

    @property
    def platform_name(self) -> str:
        return "fogros2"

    def connect(self) -> None:
        """Initialize ROS2 node (runs on trusted edge)."""
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init()

            node_name = self._config.connection_params.get(
                "node_name", "moai_fogros2_edge"
            )
            self._node = rclpy.create_node(node_name)
            logger.info(f"FogROS2: Edge node {node_name} started (key custody here)")

        except ImportError:
            logger.warning("ROS2 not available")

    def disconnect(self) -> None:
        if self._node:
            self._node.destroy_node()
            self._node = None

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        self._event_callback = callback

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert ROS2 message to event."""
        return RobotOpsEvent(
            event_id=raw_event.get("id", ""),
            event_type=EventType.UNKNOWN,
            timestamp=datetime.now(),
            source_platform="fogros2",
            text_content=str(raw_event),
            metadata=raw_event,
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Log decision (FogROS2 doesn't have specific writeback)."""
        logger.info(f"FogROS2 decision: {decision.class_label} ({decision.confidence})")

    @staticmethod
    def get_launch_guidance() -> str:
        """Return FogROS2 launch file guidance."""
        return '''
# FogROS2 Launch Configuration for MOAI
#
# CRITICAL: Keep edge_agent local for secret key custody!
#
# Example fogros2_launch.py:

from fogros2 import FogROSLaunchDescription, AWSCloudInstance
from launch_ros.actions import Node

def generate_launch_description():
    return FogROSLaunchDescription([
        # MOAI Service - CAN run in cloud (no secrets)
        Node(
            package="moai_service",
            executable="moai_server",
            name="moai_service",
            # Deploy to cloud for compute power
            fog_cloud=AWSCloudInstance(
                region="us-west-2",
                instance_type="c5.2xlarge",
            ),
        ),

        # Edge Agent - MUST run locally (has secret key)
        Node(
            package="robops_edge_agent",
            executable="edge_agent",
            name="moai_edge_agent",
            # DO NOT set fog_cloud - runs on robot
        ),
    ])
'''


DEFAULT_FOGROS2_CONFIG = AdapterConfig(
    platform_name="fogros2",
    connection_params={
        "node_name": "moai_fogros2_edge",
    },
)
