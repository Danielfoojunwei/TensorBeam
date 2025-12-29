"""OpenTelemetry instrumentation for MOAI service."""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import Any, Generator

logger = logging.getLogger(__name__)

# Metrics counters (in-memory for now, export to Prometheus)
_metrics: dict[str, Any] = {
    "jobs_submitted_total": 0,
    "jobs_completed_total": 0,
    "jobs_failed_total": 0,
    "active_sessions": 0,
    "queue_depth": 0,
    "inference_latency_sum_ms": 0.0,
    "inference_latency_count": 0,
}


def increment_counter(name: str, value: int = 1) -> None:
    """Increment a counter metric."""
    if name in _metrics:
        _metrics[name] += value


def set_gauge(name: str, value: float) -> None:
    """Set a gauge metric."""
    _metrics[name] = value


def record_latency(name: str, latency_ms: float) -> None:
    """Record a latency observation."""
    _metrics[f"{name}_sum_ms"] = _metrics.get(f"{name}_sum_ms", 0.0) + latency_ms
    _metrics[f"{name}_count"] = _metrics.get(f"{name}_count", 0) + 1


def get_metrics() -> dict[str, Any]:
    """Get all metrics."""
    return _metrics.copy()


@contextmanager
def trace_span(name: str, attributes: dict[str, str] | None = None) -> Generator[dict, None, None]:
    """Create a trace span.

    Usage:
        with trace_span("process_job", {"job_id": "123"}) as span:
            # do work
            span["result"] = "success"
    """
    span_data: dict[str, Any] = {
        "name": name,
        "start_time": time.time(),
        "attributes": attributes or {},
    }

    try:
        yield span_data
    except Exception as e:
        span_data["error"] = str(e)
        raise
    finally:
        span_data["end_time"] = time.time()
        span_data["duration_ms"] = (span_data["end_time"] - span_data["start_time"]) * 1000

        # Log span (in production, export to OTLP)
        logger.debug(
            f"Span: {name} duration={span_data['duration_ms']:.2f}ms "
            f"attrs={span_data['attributes']}"
        )


class MetricsMiddleware:
    """FastAPI middleware for metrics collection."""

    def __init__(self, app: Any) -> None:
        self.app = app

    async def __call__(self, scope: dict, receive: Any, send: Any) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        start_time = time.time()
        path = scope.get("path", "unknown")

        try:
            await self.app(scope, receive, send)
        finally:
            duration_ms = (time.time() - start_time) * 1000
            record_latency("http_request", duration_ms)


def setup_opentelemetry(
    service_name: str = "moai-service",
    otlp_endpoint: str | None = None,
) -> None:
    """Setup OpenTelemetry instrumentation.

    Args:
        service_name: Service name for traces.
        otlp_endpoint: OTLP collector endpoint (e.g., "http://localhost:4317").
    """
    try:
        from opentelemetry import trace
        from opentelemetry.sdk.trace import TracerProvider
        from opentelemetry.sdk.resources import Resource
        from opentelemetry.sdk.trace.export import BatchSpanProcessor

        resource = Resource.create({"service.name": service_name})
        provider = TracerProvider(resource=resource)

        if otlp_endpoint:
            from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter

            exporter = OTLPSpanExporter(endpoint=otlp_endpoint)
            provider.add_span_processor(BatchSpanProcessor(exporter))
            logger.info(f"OpenTelemetry configured with OTLP endpoint: {otlp_endpoint}")
        else:
            logger.info("OpenTelemetry configured (console export only)")

        trace.set_tracer_provider(provider)

    except ImportError:
        logger.warning("OpenTelemetry SDK not installed - tracing disabled")


def get_prometheus_metrics() -> str:
    """Export metrics in Prometheus format."""
    lines = [
        "# HELP moai_jobs_submitted_total Total jobs submitted",
        "# TYPE moai_jobs_submitted_total counter",
        f"moai_jobs_submitted_total {_metrics['jobs_submitted_total']}",
        "",
        "# HELP moai_jobs_completed_total Total jobs completed",
        "# TYPE moai_jobs_completed_total counter",
        f"moai_jobs_completed_total {_metrics['jobs_completed_total']}",
        "",
        "# HELP moai_jobs_failed_total Total jobs failed",
        "# TYPE moai_jobs_failed_total counter",
        f"moai_jobs_failed_total {_metrics['jobs_failed_total']}",
        "",
        "# HELP moai_active_sessions Current active sessions",
        "# TYPE moai_active_sessions gauge",
        f"moai_active_sessions {_metrics['active_sessions']}",
        "",
        "# HELP moai_queue_depth Current queue depth",
        "# TYPE moai_queue_depth gauge",
        f"moai_queue_depth {_metrics['queue_depth']}",
        "",
        "# HELP moai_inference_latency_ms Inference latency in milliseconds",
        "# TYPE moai_inference_latency_ms summary",
    ]

    count = _metrics.get("inference_latency_count", 0)
    sum_ms = _metrics.get("inference_latency_sum_ms", 0.0)
    lines.append(f'moai_inference_latency_ms_sum {sum_ms}')
    lines.append(f'moai_inference_latency_ms_count {count}')

    return "\n".join(lines) + "\n"
