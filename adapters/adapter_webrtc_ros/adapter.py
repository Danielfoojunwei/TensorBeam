"""webrtc_ros Integration Notes.

This adapter provides guidance for integrating MOAI decisions
with operator UI overlays via webrtc_ros.
"""

from __future__ import annotations

import json
import logging
from datetime import datetime
from typing import Any

from robops_edge_agent.adapter_base import AdapterConfig, RobOpsAdapter
from robops_edge_agent.events import InferenceDecision, RobotOpsEvent, EventType

logger = logging.getLogger(__name__)


class WebRTCROSAdapter(RobOpsAdapter):
    """webrtc_ros integration adapter.

    webrtc_ros provides video streaming between robots and operator UIs.
    This adapter overlays MOAI decisions on operator displays.

    Integration approach:
    - Subscribe to MOAI decision topics
    - Publish overlay annotations for webrtc_ros
    - Operator UI shows classification + confidence on video feed
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._node: Any = None
        self._publisher: Any = None

    @property
    def platform_name(self) -> str:
        return "webrtc_ros"

    def connect(self) -> None:
        """Initialize ROS2 node for overlay publishing."""
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init()

            node_name = self._config.connection_params.get(
                "node_name", "moai_overlay_publisher"
            )
            self._node = rclpy.create_node(node_name)

            # Publish overlays for operator UI
            overlay_topic = self._config.connection_params.get(
                "overlay_topic", "/moai/ui_overlay"
            )
            self._publisher = self._node.create_publisher(String, overlay_topic, 10)
            logger.info(f"webrtc_ros: Publishing overlays to {overlay_topic}")

        except ImportError:
            logger.warning("ROS2 not available - webrtc_ros adapter in mock mode")

    def disconnect(self) -> None:
        if self._node:
            self._node.destroy_node()
            self._node = None

    def set_event_callback(self, callback: Any) -> None:
        pass  # Not used - this adapter is write-only

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        return RobotOpsEvent(
            event_id="",
            event_type=EventType.UNKNOWN,
            timestamp=datetime.now(),
            source_platform="webrtc_ros",
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Publish decision as UI overlay annotation."""
        if self._publisher is None:
            return

        try:
            from std_msgs.msg import String

            overlay = {
                "type": "moai_decision",
                "decision_id": decision.decision_id,
                "classification": decision.class_label,
                "confidence": decision.confidence,
                "color": self._get_overlay_color(decision),
                "position": self._config.connection_params.get("overlay_position", "top-right"),
                "duration_ms": 5000,
                "timestamp": decision.timestamp.isoformat(),
            }

            msg = String()
            msg.data = json.dumps(overlay)
            self._publisher.publish(msg)

        except Exception as e:
            logger.error(f"webrtc_ros overlay error: {e}")

    def _get_overlay_color(self, decision: InferenceDecision) -> str:
        """Get color based on classification."""
        if decision.class_label in ("low", "approved", "compliant"):
            return "green"
        elif decision.class_label in ("medium", "pending_review", "warning"):
            return "yellow"
        elif decision.class_label in ("high", "critical", "rejected", "violation"):
            return "red"
        return "blue"

    @staticmethod
    def get_integration_notes() -> str:
        """Return integration notes for webrtc_ros."""
        return '''
# webrtc_ros Integration for MOAI

## Overview

webrtc_ros provides video streaming between robots and operator UIs.
MOAI decisions can be overlaid on the operator's video feed.

## Integration Architecture

```
Robot Camera → webrtc_ros → Operator UI
                   ↑
           MOAI Decision Overlay
```

## Implementation Steps

1. Subscribe to MOAI decisions:
   - Topic: /moai/decision

2. Publish overlay annotations:
   - Topic: /moai/ui_overlay
   - Format: JSON with classification, confidence, color

3. Operator UI renders overlays:
   - Parse JSON from /moai/ui_overlay
   - Draw classification badge on video feed

## Example Overlay Message

```json
{
  "type": "moai_decision",
  "classification": "high",
  "confidence": 0.92,
  "color": "red",
  "position": "top-right",
  "duration_ms": 5000
}
```

## UI Rendering Suggestions

- Badge with classification text + confidence %
- Color-coded border (green/yellow/red)
- Fade out after duration_ms
- Click to view full decision details
'''


DEFAULT_WEBRTCROS_CONFIG = AdapterConfig(
    platform_name="webrtc_ros",
    connection_params={
        "node_name": "moai_overlay_publisher",
        "overlay_topic": "/moai/ui_overlay",
        "overlay_position": "top-right",
    },
)
