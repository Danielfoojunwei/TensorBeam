"""openTCS Adapter.

Connects to openTCS via Web API for transport order management.
"""

from __future__ import annotations

import json
import logging
import uuid
from datetime import datetime
from typing import Any, Callable

import httpx

from robops_edge_agent.adapter_base import AdapterConfig, RobOpsAdapter
from robops_edge_agent.events import (
    EventType,
    InferenceDecision,
    RobotOpsEvent,
)

logger = logging.getLogger(__name__)


class OpenTCSAdapter(RobOpsAdapter):
    """openTCS adapter using Web API.

    openTCS Web API endpoints:
    - GET /v1/transportOrders - List transport orders
    - GET /v1/vehicles - List vehicles
    - POST /v1/transportOrders - Create order
    - PUT /v1/transportOrders/{name}/properties - Update order properties

    Integration:
    - Poll for new transport orders
    - Add MOAI decision annotations to order properties
    - Flag orders for review if classification confidence is low
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._http_client: httpx.Client | None = None
        self._polling = False
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None

    @property
    def platform_name(self) -> str:
        return "opentcs"

    def connect(self) -> None:
        """Initialize HTTP client for openTCS Web API."""
        base_url = self._config.connection_params.get(
            "base_url", "http://localhost:55200/v1"
        )
        timeout = self._config.connection_params.get("timeout", 30.0)

        self._http_client = httpx.Client(
            base_url=base_url,
            timeout=timeout,
            headers={"Content-Type": "application/json"},
        )
        logger.info(f"Connected to openTCS at {base_url}")

    def disconnect(self) -> None:
        if self._http_client:
            self._http_client.close()
            self._http_client = None
        self._polling = False

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        self._event_callback = callback

    def poll_transport_orders(self) -> list[RobotOpsEvent]:
        """Poll for transport orders and convert to events.

        Returns:
            List of events from transport orders.
        """
        if not self._http_client:
            logger.warning("openTCS client not connected")
            return []

        events = []
        try:
            response = self._http_client.get("/transportOrders")
            response.raise_for_status()
            orders = response.json()

            for order in orders:
                # Only process orders that haven't been annotated
                if not order.get("properties", {}).get("moai_processed"):
                    event = self.ingest_event(order)
                    events.append(event)

        except Exception as e:
            logger.error(f"Error polling openTCS: {e}")

        return events

    def get_vehicle_states(self) -> list[dict]:
        """Get current vehicle states."""
        if not self._http_client:
            return []

        try:
            response = self._http_client.get("/vehicles")
            response.raise_for_status()
            return response.json()
        except Exception as e:
            logger.error(f"Error getting vehicles: {e}")
            return []

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert openTCS transport order to RobotOpsEvent."""
        order_name = raw_event.get("name", str(uuid.uuid4()))
        destinations = raw_event.get("destinations", [])

        # Build text description
        dest_names = [d.get("destinationPoint", "unknown") for d in destinations]
        text_content = f"Transport order {order_name}: {' -> '.join(dest_names)}"

        # Get vehicle assignment
        vehicle = raw_event.get("processingVehicle")
        state = raw_event.get("state", "UNKNOWN")

        return RobotOpsEvent(
            event_id=order_name,
            event_type=EventType.TASK_REQUEST,
            timestamp=datetime.now(),
            source_platform="opentcs",
            robot_id=vehicle,
            text_content=text_content,
            task_id=order_name,
            metadata={
                "state": state,
                "destinations": destinations,
                "properties": raw_event.get("properties", {}),
            },
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """Update transport order with MOAI decision."""
        if not self._http_client:
            logger.warning("openTCS client not connected")
            return

        order_name = decision.event_id

        try:
            # Update order properties with decision
            properties = {
                "moai_processed": "true",
                "moai_classification": decision.class_label or "",
                "moai_confidence": str(decision.confidence or 0.0),
                "moai_requires_review": str(decision.confidence < 0.8 if decision.confidence else True).lower(),
                "moai_decision_id": decision.decision_id,
                "moai_timestamp": decision.timestamp.isoformat(),
            }

            response = self._http_client.put(
                f"/transportOrders/{order_name}/properties",
                json=properties,
            )
            response.raise_for_status()
            logger.info(f"Updated openTCS order {order_name} with decision")

        except httpx.HTTPStatusError as e:
            if e.response.status_code == 404:
                logger.warning(f"Transport order {order_name} not found")
            else:
                logger.error(f"Error updating openTCS order: {e}")
        except Exception as e:
            logger.error(f"Error writing back to openTCS: {e}")


DEFAULT_OPENTCS_CONFIG = AdapterConfig(
    platform_name="opentcs",
    connection_params={
        "base_url": "http://localhost:55200/v1",
        "timeout": 30.0,
        "poll_interval_seconds": 5.0,
    },
    event_mappings={},
)
