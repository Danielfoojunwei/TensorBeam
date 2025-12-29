"""Event and decision data structures for RobOps integration."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any


class EventType(Enum):
    """Types of robot operations events."""

    INCIDENT = "incident"
    TASK_REQUEST = "task_request"
    COMPLIANCE_CHECK = "compliance_check"
    SOP_QUERY = "sop_query"
    STATUS_UPDATE = "status_update"
    ALERT = "alert"
    UNKNOWN = "unknown"


class DecisionType(Enum):
    """Types of inference decisions."""

    CLASSIFICATION = "classification"
    RANKING = "ranking"
    SCORE = "score"
    CONSTRAINT = "constraint"
    APPROVAL = "approval"


class Severity(Enum):
    """Incident severity levels."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class RobotOpsEvent:
    """Normalized robot operations event.

    This is the common format that all adapters convert to.
    """

    event_id: str
    event_type: EventType
    timestamp: datetime
    source_platform: str
    robot_id: str | None = None
    fleet_id: str | None = None

    # Content (at least one should be present)
    text_content: str | None = None  # For text-based inference
    embedding: list[float] | None = None  # Pre-computed embedding
    structured_data: dict[str, Any] | None = None

    # Context
    location: str | None = None
    zone: str | None = None
    task_id: str | None = None

    # Metadata
    metadata: dict[str, Any] = field(default_factory=dict)
    raw_event: bytes | None = None  # Original event for debugging


@dataclass
class InferenceDecision:
    """Decision from MOAI inference.

    This is returned to adapters for writeback.
    """

    decision_id: str
    event_id: str  # Links back to source event
    decision_type: DecisionType
    timestamp: datetime

    # Classification output
    predicted_class: int | None = None
    class_label: str | None = None
    confidence: float | None = None

    # Ranking output (for SOP rerank etc)
    rankings: list[str] | None = None
    scores: list[float] | None = None

    # Score output
    score: float | None = None

    # Constraint output
    constraints: list[str] | None = None
    allowed: bool | None = None

    # Raw logits for advanced use
    logits: list[float] | None = None
    probabilities: list[float] | None = None

    # Metadata
    metadata: dict[str, Any] = field(default_factory=dict)
    latency_ms: float | None = None


# Example class mappings for demo
INCIDENT_SEVERITY_LABELS = ["low", "medium", "high", "critical"]
COMPLIANCE_LABELS = ["compliant", "warning", "violation"]
TASK_APPROVAL_LABELS = ["approved", "pending_review", "rejected"]
