"""Worker pool for processing FHE inference jobs."""

from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import TYPE_CHECKING

from moai_service.job_queue import Job, JobQueue, JobStatus

if TYPE_CHECKING:
    from moai_core.backend import FHEBackend, InferenceModel

logger = logging.getLogger(__name__)


class WorkerPool:
    """Thread pool for processing FHE inference jobs.

    Workers pull jobs from the queue, run encrypted inference,
    and store results.
    """

    def __init__(
        self,
        queue: JobQueue,
        backend: "FHEBackend",
        model_registry: dict[str, "InferenceModel"],
        context_store: dict[str, bytes],
        num_workers: int = 4,
    ) -> None:
        """Initialize worker pool.

        Args:
            queue: Job queue to pull from.
            backend: FHE backend for inference.
            model_registry: Model ID -> model mapping.
            context_store: Context ID -> context bytes mapping.
            num_workers: Number of worker threads.
        """
        self._queue = queue
        self._backend = backend
        self._model_registry = model_registry
        self._context_store = context_store
        self._num_workers = num_workers
        self._executor: ThreadPoolExecutor | None = None
        self._running = False
        self._stop_event = threading.Event()

    def start(self) -> None:
        """Start the worker pool."""
        if self._running:
            return

        self._running = True
        self._stop_event.clear()
        self._executor = ThreadPoolExecutor(
            max_workers=self._num_workers,
            thread_name_prefix="moai-worker",
        )

        # Start worker threads
        for i in range(self._num_workers):
            self._executor.submit(self._worker_loop, i)

        logger.info(f"Worker pool started with {self._num_workers} workers")

    def stop(self, wait: bool = True) -> None:
        """Stop the worker pool.

        Args:
            wait: Whether to wait for workers to finish.
        """
        self._running = False
        self._stop_event.set()

        if self._executor:
            self._executor.shutdown(wait=wait)
            self._executor = None

        logger.info("Worker pool stopped")

    def _worker_loop(self, worker_id: int) -> None:
        """Main worker loop.

        Args:
            worker_id: ID of this worker.
        """
        logger.info(f"Worker {worker_id} started")

        while self._running and not self._stop_event.is_set():
            try:
                job = self._queue.get_next()

                if job is None:
                    # No jobs, wait a bit
                    time.sleep(0.1)
                    continue

                self._process_job(job, worker_id)

            except Exception as e:
                logger.exception(f"Worker {worker_id} error: {e}")

        logger.info(f"Worker {worker_id} stopped")

    def _process_job(self, job: Job, worker_id: int) -> None:
        """Process a single job.

        Args:
            job: Job to process.
            worker_id: ID of processing worker.
        """
        start_time = time.perf_counter()
        logger.info(f"Worker {worker_id} processing job {job.job_id}")

        try:
            # Get context
            context = self._context_store.get(job.context_id)
            if context is None:
                self._queue.fail_job(job.job_id, f"Context not found: {job.context_id}")
                return

            # Get model
            model = self._model_registry.get(job.model_id)
            if model is None:
                self._queue.fail_job(job.job_id, f"Model not found: {job.model_id}")
                return

            # Process each input
            outputs = []
            total_input_bytes = 0
            total_output_bytes = 0

            for ct_input in job.ciphertext_inputs:
                total_input_bytes += len(ct_input)

                # Run encrypted inference
                ct_output = self._backend.eval(
                    context,
                    context,  # Eval keys bundled in context for TenSEAL
                    ct_input,
                    model,
                )

                outputs.append(ct_output)
                total_output_bytes += len(ct_output)

            elapsed_ms = (time.perf_counter() - start_time) * 1000

            metadata = {
                "model_id": job.model_id,
                "latency_ms": elapsed_ms,
                "input_bytes": total_input_bytes,
                "output_bytes": total_output_bytes,
                "worker_id": worker_id,
            }

            self._queue.complete_job(job.job_id, outputs, metadata)
            logger.info(f"Job {job.job_id} completed in {elapsed_ms:.2f}ms")

        except Exception as e:
            self._queue.fail_job(job.job_id, str(e))
            logger.error(f"Job {job.job_id} failed: {e}")


class MetricsCollector:
    """Collect worker pool metrics."""

    def __init__(self, queue: JobQueue) -> None:
        self._queue = queue
        self._jobs_processed = 0
        self._jobs_failed = 0
        self._total_latency_ms = 0.0
        self._lock = threading.Lock()

    def record_completion(self, latency_ms: float) -> None:
        """Record a completed job."""
        with self._lock:
            self._jobs_processed += 1
            self._total_latency_ms += latency_ms

    def record_failure(self) -> None:
        """Record a failed job."""
        with self._lock:
            self._jobs_failed += 1

    def get_metrics(self) -> dict:
        """Get current metrics."""
        with self._lock:
            avg_latency = (
                self._total_latency_ms / self._jobs_processed
                if self._jobs_processed > 0
                else 0.0
            )

            return {
                "queue_depth": self._queue.queue_depth(),
                "jobs_processed": self._jobs_processed,
                "jobs_failed": self._jobs_failed,
                "avg_latency_ms": avg_latency,
            }
