"""Job queue for MOAI service.

Provides in-memory queue with optional Redis backend.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class JobStatus(Enum):
    """Job status enum."""

    UNKNOWN = 0
    QUEUED = 1
    PROCESSING = 2
    COMPLETED = 3
    FAILED = 4
    EXPIRED = 5


@dataclass
class Job:
    """Represents an FHE inference job."""

    job_id: str
    context_id: str
    model_id: str
    ciphertext_inputs: list[bytes]
    metadata: dict[str, str] = field(default_factory=dict)
    idempotency_key: str | None = None

    status: JobStatus = JobStatus.QUEUED
    error_message: str | None = None
    ciphertext_outputs: list[bytes] | None = None
    result_metadata: dict[str, Any] | None = None

    created_at: float = field(default_factory=time.time)
    started_at: float | None = None
    completed_at: float | None = None


class JobQueue:
    """In-memory job queue with thread-safe operations.

    Can be replaced with Redis backend for production.
    """

    def __init__(self, max_size: int = 1000) -> None:
        """Initialize queue.

        Args:
            max_size: Maximum queue size.
        """
        self._queue: deque[str] = deque(maxlen=max_size)
        self._jobs: dict[str, Job] = {}
        self._idempotency_cache: dict[str, str] = {}  # key -> job_id
        self._lock = threading.RLock()
        self._max_size = max_size

    def submit(self, job: Job) -> str:
        """Submit a job to the queue.

        Args:
            job: Job to submit.

        Returns:
            Job ID.

        Raises:
            ValueError: If queue is full.
        """
        with self._lock:
            # Check idempotency
            if job.idempotency_key and job.idempotency_key in self._idempotency_cache:
                existing_id = self._idempotency_cache[job.idempotency_key]
                logger.info(f"Idempotent job already exists: {existing_id}")
                return existing_id

            if len(self._queue) >= self._max_size:
                raise ValueError("Job queue is full")

            self._jobs[job.job_id] = job
            self._queue.append(job.job_id)

            if job.idempotency_key:
                self._idempotency_cache[job.idempotency_key] = job.job_id

            logger.info(f"Job submitted: {job.job_id}")
            return job.job_id

    def get_next(self) -> Job | None:
        """Get next job from queue.

        Returns:
            Next job or None if empty.
        """
        with self._lock:
            while self._queue:
                job_id = self._queue.popleft()
                job = self._jobs.get(job_id)
                if job and job.status == JobStatus.QUEUED:
                    job.status = JobStatus.PROCESSING
                    job.started_at = time.time()
                    return job
            return None

    def get_job(self, job_id: str) -> Job | None:
        """Get job by ID.

        Args:
            job_id: Job ID.

        Returns:
            Job or None.
        """
        with self._lock:
            return self._jobs.get(job_id)

    def complete_job(
        self,
        job_id: str,
        outputs: list[bytes],
        metadata: dict[str, Any],
    ) -> None:
        """Mark job as completed.

        Args:
            job_id: Job ID.
            outputs: Ciphertext outputs.
            metadata: Result metadata.
        """
        with self._lock:
            job = self._jobs.get(job_id)
            if job:
                job.status = JobStatus.COMPLETED
                job.ciphertext_outputs = outputs
                job.result_metadata = metadata
                job.completed_at = time.time()
                logger.info(f"Job completed: {job_id}")

    def fail_job(self, job_id: str, error: str) -> None:
        """Mark job as failed.

        Args:
            job_id: Job ID.
            error: Error message.
        """
        with self._lock:
            job = self._jobs.get(job_id)
            if job:
                job.status = JobStatus.FAILED
                job.error_message = error
                job.completed_at = time.time()
                logger.error(f"Job failed: {job_id} - {error}")

    def queue_depth(self) -> int:
        """Get current queue depth."""
        with self._lock:
            return len(self._queue)

    def cleanup_old_jobs(self, max_age_seconds: int = 3600) -> int:
        """Remove old completed/failed jobs.

        Args:
            max_age_seconds: Maximum age before cleanup.

        Returns:
            Number of jobs cleaned up.
        """
        with self._lock:
            now = time.time()
            to_remove = []

            for job_id, job in self._jobs.items():
                if job.status in (JobStatus.COMPLETED, JobStatus.FAILED):
                    if job.completed_at and (now - job.completed_at) > max_age_seconds:
                        to_remove.append(job_id)

            for job_id in to_remove:
                job = self._jobs.pop(job_id)
                if job.idempotency_key:
                    self._idempotency_cache.pop(job.idempotency_key, None)

            return len(to_remove)


def create_job_id() -> str:
    """Create a new job ID."""
    return f"job-{uuid.uuid4().hex[:12]}"
