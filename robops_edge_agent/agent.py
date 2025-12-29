"""RobOps Edge Agent - Main orchestrator."""

from __future__ import annotations

import logging
import uuid
from datetime import datetime
from typing import TYPE_CHECKING

import numpy as np

from robops_edge_agent.events import (
    DecisionType,
    EventType,
    InferenceDecision,
    RobotOpsEvent,
    INCIDENT_SEVERITY_LABELS,
)

if TYPE_CHECKING:
    from moai_client_sdk import MOAIClient
    from robops_edge_agent.adapter_base import RobOpsAdapter

logger = logging.getLogger(__name__)


class EdgeAgent:
    """RobOps Edge Agent.

    Orchestrates the flow:
    1. Receive events from platform adapters
    2. Convert to inference requests
    3. Call MOAI for encrypted inference
    4. Route decisions back via adapters

    Supports multiple concurrent adapters.
    """

    def __init__(
        self,
        moai_client: "MOAIClient",
        default_model_id: str = "demo-linear-classifier",
    ) -> None:
        """Initialize edge agent.

        Args:
            moai_client: MOAI client for FHE inference.
            default_model_id: Default model to use.
        """
        self._moai_client = moai_client
        self._default_model_id = default_model_id
        self._adapters: dict[str, "RobOpsAdapter"] = {}
        self._running = False

    def register_adapter(self, name: str, adapter: "RobOpsAdapter") -> None:
        """Register a platform adapter.

        Args:
            name: Unique name for the adapter.
            adapter: Adapter instance.
        """
        self._adapters[name] = adapter
        logger.info(f"Registered adapter: {name}")

    def process_event(self, event: RobotOpsEvent) -> InferenceDecision:
        """Process a single event through MOAI.

        Args:
            event: Normalized robot ops event.

        Returns:
            Inference decision.
        """
        logger.info(f"Processing event: {event.event_id} ({event.event_type.value})")

        # Get or generate embedding
        if event.embedding:
            embedding = np.array(event.embedding, dtype=np.float64)
        else:
            # For demo: generate a random embedding from text
            # In production: use a text embedding model
            embedding = self._generate_demo_embedding(event.text_content or "")

        # Determine output dimension based on event type
        output_dim = self._get_output_dim(event.event_type)

        # Run encrypted inference
        result = self._moai_client.infer(
            model_id=self._default_model_id,
            embedding=embedding,
            output_dim=output_dim,
        )

        # Convert to decision
        decision = self._result_to_decision(event, result, output_dim)

        logger.info(
            f"Decision for {event.event_id}: "
            f"class={decision.class_label}, confidence={decision.confidence:.2%}"
        )

        return decision

    def process_and_writeback(
        self,
        event: RobotOpsEvent,
        adapter_name: str,
    ) -> InferenceDecision:
        """Process event and write back via adapter.

        Args:
            event: Event to process.
            adapter_name: Adapter to use for writeback.

        Returns:
            Inference decision.
        """
        decision = self.process_event(event)

        adapter = self._adapters.get(adapter_name)
        if adapter:
            adapter.writeback(decision)
        else:
            logger.warning(f"Adapter not found for writeback: {adapter_name}")

        return decision

    def _generate_demo_embedding(self, text: str) -> np.ndarray:
        """Generate a demo embedding from text.

        In production, use a proper text encoder.
        """
        # Simple hash-based embedding for demo
        np.random.seed(hash(text) % (2**32))
        embedding = np.random.randn(64).astype(np.float64)
        embedding = embedding / np.linalg.norm(embedding)

        return embedding

    def _get_output_dim(self, event_type: EventType) -> int:
        """Get output dimension for event type."""
        # Default to 3 classes (linear classifier demo)
        return 3

    def _result_to_decision(
        self,
        event: RobotOpsEvent,
        result: "InferenceResult",  # type: ignore[name-defined]
        output_dim: int,
    ) -> InferenceDecision:
        """Convert inference result to decision.

        Args:
            event: Source event.
            result: MOAI inference result.
            output_dim: Output dimension.

        Returns:
            Inference decision.
        """
        # Get labels based on event type
        if event.event_type == EventType.INCIDENT:
            labels = INCIDENT_SEVERITY_LABELS[:output_dim]
        else:
            labels = [f"class_{i}" for i in range(output_dim)]

        predicted_class = result.predicted_class or 0
        confidence = float(result.probabilities[predicted_class]) if result.probabilities is not None else 0.0

        return InferenceDecision(
            decision_id=str(uuid.uuid4()),
            event_id=event.event_id,
            decision_type=DecisionType.CLASSIFICATION,
            timestamp=datetime.now(),
            predicted_class=predicted_class,
            class_label=labels[predicted_class] if predicted_class < len(labels) else str(predicted_class),
            confidence=confidence,
            logits=result.logits.tolist() if result.logits is not None else None,
            probabilities=result.probabilities.tolist() if result.probabilities is not None else None,
            latency_ms=result.metadata.get("latency_ms") if result.metadata else None,
        )


class InferenceResult:
    """Placeholder for type checking."""

    logits: np.ndarray | None
    probabilities: np.ndarray | None
    predicted_class: int | None
    metadata: dict | None
