"""Base adapter interface for RobOps platforms."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from robops_edge_agent.events import InferenceDecision, RobotOpsEvent


class RobOpsAdapter(ABC):
    """Abstract base class for platform adapters.

    Each adapter must implement:
    - connect(): Establish connection to platform
    - ingest_event(): Convert platform-specific event to RobotOpsEvent
    - writeback(): Write decision back to platform
    """

    @property
    @abstractmethod
    def platform_name(self) -> str:
        """Platform identifier."""
        ...

    @abstractmethod
    def connect(self) -> None:
        """Establish connection to the platform.

        May start subscribers, open connections, etc.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Close connection to the platform."""
        ...

    @abstractmethod
    def ingest_event(self, raw_event: Any) -> RobotOpsEvent:
        """Convert platform-specific event to normalized format.

        Args:
            raw_event: Platform-specific event data.

        Returns:
            Normalized RobotOpsEvent.
        """
        ...

    @abstractmethod
    def writeback(self, decision: InferenceDecision) -> None:
        """Write decision back to the platform.

        Args:
            decision: Inference decision to write back.
        """
        ...

    def health_check(self) -> bool:
        """Check if adapter is healthy.

        Returns:
            True if healthy.
        """
        return True


class AdapterConfig:
    """Configuration for adapters."""

    def __init__(
        self,
        platform_name: str,
        connection_params: dict[str, Any] | None = None,
        event_mappings: dict[str, str] | None = None,
        writeback_config: dict[str, Any] | None = None,
    ) -> None:
        """Initialize config.

        Args:
            platform_name: Platform identifier.
            connection_params: Connection parameters.
            event_mappings: Field mappings for event normalization.
            writeback_config: Configuration for writeback.
        """
        self.platform_name = platform_name
        self.connection_params = connection_params or {}
        self.event_mappings = event_mappings or {}
        self.writeback_config = writeback_config or {}

    @classmethod
    def from_yaml(cls, path: str) -> "AdapterConfig":
        """Load config from YAML file.

        Args:
            path: Path to YAML file.

        Returns:
            AdapterConfig instance.
        """
        import yaml

        with open(path) as f:
            data = yaml.safe_load(f)

        return cls(
            platform_name=data.get("platform_name", "unknown"),
            connection_params=data.get("connection", {}),
            event_mappings=data.get("event_mappings", {}),
            writeback_config=data.get("writeback", {}),
        )
