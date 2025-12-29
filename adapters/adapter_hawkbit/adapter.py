"""Eclipse hawkBit Adapter.

OTA updates for edge agent and model configs.
"""

from __future__ import annotations

import json
import logging
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


class HawkBitAdapter(RobOpsAdapter):
    """Eclipse hawkBit adapter for OTA updates.

    Uses hawkBit DDI (Direct Device Integration) API.

    Capabilities:
    - Pull software updates for edge agent
    - Download model config bundles
    - Report deployment status

    hawkBit can distribute:
    - Edge agent container images
    - Model configuration files
    - Plaintext student models (for fast-lane inference)
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._http_client: httpx.Client | None = None
        self._target_token: str | None = None
        self._controller_id = config.connection_params.get("controller_id", "moai-edge")
        self._event_callback: Callable[[RobotOpsEvent], None] | None = None

    @property
    def platform_name(self) -> str:
        return "hawkbit"

    def connect(self) -> None:
        """Initialize hawkBit DDI client."""
        base_url = self._config.connection_params.get(
            "base_url", "http://localhost:8080"
        )
        tenant = self._config.connection_params.get("tenant", "DEFAULT")
        self._target_token = self._config.connection_params.get("target_token", "")

        self._http_client = httpx.Client(
            base_url=f"{base_url}/{tenant}/controller/v1/{self._controller_id}",
            headers={
                "Authorization": f"TargetToken {self._target_token}",
                "Accept": "application/json",
            },
            timeout=30.0,
        )

        # Register with hawkBit
        try:
            response = self._http_client.get("/")
            response.raise_for_status()
            logger.info(f"Connected to hawkBit as {self._controller_id}")
        except Exception as e:
            logger.warning(f"hawkBit connection failed (may be offline): {e}")

    def disconnect(self) -> None:
        if self._http_client:
            self._http_client.close()
            self._http_client = None

    def set_event_callback(self, callback: Callable[[RobotOpsEvent], None]) -> None:
        self._event_callback = callback

    def poll_for_updates(self) -> list[dict]:
        """Poll hawkBit for pending deployments.

        Returns:
            List of deployment actions.
        """
        if not self._http_client:
            return []

        try:
            response = self._http_client.get("/deploymentBase")
            if response.status_code == 200:
                data = response.json()
                return data.get("actions", [])
        except Exception as e:
            logger.error(f"hawkBit poll error: {e}")

        return []

    def report_feedback(
        self,
        action_id: str,
        status: str = "proceeding",
        message: str = "",
    ) -> None:
        """Report deployment feedback to hawkBit.

        Args:
            action_id: Deployment action ID.
            status: closed/proceeding/canceled/scheduled/rejected/resumed.
            message: Status message.
        """
        if not self._http_client:
            return

        feedback = {
            "id": action_id,
            "status": {
                "execution": status,
                "result": {"finished": "success" if status == "closed" else "none"},
                "details": [message] if message else [],
            },
        }

        try:
            response = self._http_client.post(
                f"/deploymentBase/{action_id}/feedback",
                json=feedback,
            )
            response.raise_for_status()
            logger.info(f"hawkBit feedback sent for action {action_id}")
        except Exception as e:
            logger.error(f"hawkBit feedback error: {e}")

    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert hawkBit action to event."""
        return RobotOpsEvent(
            event_id=str(raw_event.get("id", "")),
            event_type=EventType.STATUS_UPDATE,
            timestamp=datetime.now(),
            source_platform="hawkbit",
            text_content=f"OTA update: {raw_event.get('softwareModule', {}).get('name', 'unknown')}",
            metadata=raw_event,
            raw_event=json.dumps(raw_event).encode(),
        )

    def writeback(self, decision: InferenceDecision) -> None:
        """No direct writeback for hawkBit (OTA focused)."""
        logger.debug(f"hawkBit: decision noted (no writeback): {decision.decision_id}")

    @staticmethod
    def get_ota_flow() -> str:
        """Return OTA update flow documentation."""
        return '''
# hawkBit OTA Flow for MOAI

## Artifacts to Distribute

1. Edge Agent Container
   - Container image for robops_edge_agent
   - Tag: moai/edge-agent:v0.1.0

2. Model Config Bundle
   - model_config.yaml with model parameters
   - Can update without full agent rebuild

3. Plaintext Student Models (optional)
   - For fast-lane inference (non-FHE path)
   - Smaller models for latency-critical decisions

## Rollout Steps

1. Create Software Module in hawkBit
   hawkBit UI → Software → Create Module

2. Upload Artifacts
   Attach container image or config files

3. Create Distribution Set
   Group modules for deployment

4. Create Campaign
   Target specific robots/groups

5. Monitor Rollout
   Track deployment status per device

## Edge Agent Integration

```python
from adapters.adapter_hawkbit import HawkBitAdapter

adapter = HawkBitAdapter(config)
adapter.connect()

# Poll for updates
updates = adapter.poll_for_updates()
for update in updates:
    # Process update
    adapter.report_feedback(update["id"], "proceeding")
    # ... apply update ...
    adapter.report_feedback(update["id"], "closed")
```
'''


DEFAULT_HAWKBIT_CONFIG = AdapterConfig(
    platform_name="hawkbit",
    connection_params={
        "base_url": "http://localhost:8080",
        "tenant": "DEFAULT",
        "controller_id": "moai-edge-001",
        "target_token": "",  # Set via environment
    },
)
