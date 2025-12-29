"""Generic Webhook Adapter.

Configurable HTTP webhook receiver and sender for proprietary RobOps platforms:
- Formant
- InOrbit
- AWS RoboRunner
- Any REST/webhook system
"""

from __future__ import annotations

import json
import logging
import threading
import uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Callable

import httpx

from robops_edge_agent.adapter_base import AdapterConfig, RobOpsAdapter
from robops_edge_agent.events import (
    EventType,
    InferenceDecision,
    RobotOpsEvent,
)

logger = logging.getLogger(__name__)


class WebhookAdapter(RobOpsAdapter):
    """Generic webhook adapter for proprietary RobOps platforms.

    Provides:
    - HTTP server for receiving events (webhook receiver)
    - HTTP client for writing back decisions (webhook sender)
    - Configurable field mappings to normalize events
    """

    def __init__(self, config: AdapterConfig) -> None:
        """Initialize webhook adapter.

        Args:
            config: Adapter configuration.
        """
        self._config = config
        self._server: HTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None
        self._http_client: httpx.Client | None = None

    @property
    def platform_name(self) -> str:
        return self._config.platform_name

    def connect(self) -> None:
        """Start webhook receiver server."""
        host = self._config.connection_params.get("host", "0.0.0.0")
        port = self._config.connection_params.get("port", 8080)

        # Create server
        adapter_self = self

        class WebhookHandler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                content_length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(content_length)

                try:
                    data = json.loads(body)
                    event = adapter_self.ingest_event(data)

                    if adapter_self._event_callback:
                        adapter_self._event_callback(event)

                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "received"}).encode())

                except Exception as e:
                    logger.error(f"Webhook error: {e}")
                    self.send_response(400)
                    self.end_headers()

            def log_message(self, format: str, *args: Any) -> None:
                logger.debug(f"Webhook: {format % args}")

        self._server = HTTPServer((host, port), WebhookHandler)
        self._server_thread = threading.Thread(target=self._server.serve_forever)
        self._server_thread.daemon = True
        self._server_thread.start()

        # Create HTTP client for writeback
        self._http_client = httpx.Client(timeout=30.0)

        logger.info(f"Webhook adapter started on {host}:{port}")

    def disconnect(self) -> None:
        """Stop webhook server."""
        if self._server:
            self._server.shutdown()
            self._server = None

        if self._http_client:
            self._http_client.close()
            self._http_client = None

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        """Set callback for incoming events."""
        self._event_callback = callback

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert webhook payload to RobotOpsEvent."""
        mappings = self._config.event_mappings

        # Extract fields using mappings
        event_id = raw_event.get(mappings.get("event_id", "id"), str(uuid.uuid4()))
        event_type_str = raw_event.get(mappings.get("event_type", "type"), "unknown")
        text_content = raw_event.get(mappings.get("text_content", "message"), None)
        robot_id = raw_event.get(mappings.get("robot_id", "robot_id"), None)

        # Navigate nested paths if specified (e.g., "data.robot.id")
        for key, path in mappings.items():
            if "." in path:
                parts = path.split(".")
                value = raw_event
                for part in parts:
                    value = value.get(part, {}) if isinstance(value, dict) else None
                if value and key == "text_content":
                    text_content = value
                elif value and key == "robot_id":
                    robot_id = value

        # Map event type
        event_type_map = {
            "incident": EventType.INCIDENT,
            "task": EventType.TASK_REQUEST,
            "compliance": EventType.COMPLIANCE_CHECK,
            "sop": EventType.SOP_QUERY,
            "alert": EventType.ALERT,
            "work_order": EventType.TASK_REQUEST,
        }
        event_type = event_type_map.get(event_type_str.lower(), EventType.UNKNOWN)

        return RobotOpsEvent(
            event_id=event_id,
            event_type=event_type,
            timestamp=datetime.now(),
            source_platform=self.platform_name,
            robot_id=robot_id,
            text_content=text_content,
            metadata=raw_event,
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Send decision to callback URL."""
        callback_url = self._config.writeback_config.get("callback_url")

        if not callback_url or not self._http_client:
            logger.warning("No callback URL configured or client not available")
            return

        # Build payload using writeback mappings
        payload_template = self._config.writeback_config.get("payload_template", {})
        payload = self._build_payload(decision, payload_template)

        try:
            response = self._http_client.post(callback_url, json=payload)
            response.raise_for_status()
            logger.info(f"Writeback sent to {callback_url}: {decision.decision_id}")

        except Exception as e:
            logger.error(f"Writeback failed: {e}")

    def _build_payload(
        self,
        decision: InferenceDecision,
        template: dict[str, Any],
    ) -> dict[str, Any]:
        """Build writeback payload from template."""
        # Default payload
        payload = {
            "decision_id": decision.decision_id,
            "event_id": decision.event_id,
            "decision_type": decision.decision_type.value,
            "timestamp": decision.timestamp.isoformat(),
            "predicted_class": decision.predicted_class,
            "class_label": decision.class_label,
            "confidence": decision.confidence,
        }

        # Merge with template
        for key, value in template.items():
            if value == "$decision_id":
                payload[key] = decision.decision_id
            elif value == "$class_label":
                payload[key] = decision.class_label
            elif value == "$confidence":
                payload[key] = decision.confidence
            else:
                payload[key] = value

        return payload


# Example configurations for common platforms

FORMANT_LIKE_CONFIG = AdapterConfig(
    platform_name="formant_like",
    connection_params={
        "host": "0.0.0.0",
        "port": 8080,
    },
    event_mappings={
        "event_id": "id",
        "event_type": "eventType",
        "text_content": "data.message",
        "robot_id": "data.robotId",
    },
    writeback_config={
        "callback_url": "https://example.formant.io/api/incidents/update",
        "payload_template": {
            "incidentId": "$event_id",
            "severity": "$class_label",
            "aiConfidence": "$confidence",
        },
    },
)

INORBIT_LIKE_CONFIG = AdapterConfig(
    platform_name="inorbit_like",
    connection_params={
        "host": "0.0.0.0",
        "port": 8080,
    },
    event_mappings={
        "event_id": "eventId",
        "event_type": "type",
        "text_content": "payload.description",
        "robot_id": "robotId",
    },
    writeback_config={
        "callback_url": "https://api.inorbit.ai/webhooks/decision",
        "payload_template": {
            "missionId": "$event_id",
            "classification": "$class_label",
        },
    },
)

ROBORUNNER_LIKE_CONFIG = AdapterConfig(
    platform_name="aws_roborunner_like",
    connection_params={
        "host": "0.0.0.0",
        "port": 8080,
    },
    event_mappings={
        "event_id": "workOrderId",
        "event_type": "work_order",
        "text_content": "details.description",
        "robot_id": "workerFleet.workerId",
    },
    writeback_config={
        "callback_url": "https://api.example.com/orders/annotate",
        "payload_template": {
            "orderId": "$event_id",
            "riskLevel": "$class_label",
        },
    },
)
