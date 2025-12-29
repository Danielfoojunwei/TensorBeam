"""MOAI gRPC Service Implementation."""

from __future__ import annotations

import logging
import time
import uuid
from concurrent import futures
from typing import Any

import grpc

logger = logging.getLogger(__name__)

# Note: In production, generate Python stubs from moai.proto using:
# python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. proto/moai.proto


class MOAIServicer:
    """gRPC servicer implementation for MOAI."""

    def __init__(
        self,
        backend: Any,
        model_registry: dict[str, Any],
        session_store: Any,
        job_queue: Any,
    ) -> None:
        """Initialize servicer.

        Args:
            backend: FHE backend.
            model_registry: Model ID -> model mapping.
            session_store: Session/context store.
            job_queue: Job queue.
        """
        self._backend = backend
        self._model_registry = model_registry
        self._session_store = session_store
        self._job_queue = job_queue

    def RegisterContext(self, request: Any, context: grpc.ServicerContext) -> Any:
        """Register HE context with server."""
        try:
            context_id = str(uuid.uuid4())
            self._session_store.register(
                context_id=context_id,
                tenant_id=request.tenant_id,
                context_bytes=request.context_bytes,
                scheme=request.scheme,
            )
            logger.info(f"Context registered: {context_id}")
            return {"context_id": context_id, "status": "registered"}
        except Exception as e:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return None

    def RegisterEvalKeys(self, request: Any, context: grpc.ServicerContext) -> Any:
        """Register evaluation keys."""
        try:
            self._session_store.add_eval_keys(
                context_id=request.context_id,
                eval_keys=request.eval_keys_bytes,
                ttl_seconds=request.ttl_seconds or 3600,
            )
            expires_at = int(time.time()) + (request.ttl_seconds or 3600)
            logger.info(f"Eval keys registered for context: {request.context_id}")
            return {"status": "registered", "expires_at_unix": expires_at}
        except KeyError:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Context not found")
            return None

    def SubmitJob(self, request: Any, context: grpc.ServicerContext) -> Any:
        """Submit inference job."""
        from moai_service.job_queue import Job, JobStatus, create_job_id

        try:
            job = Job(
                job_id=create_job_id(),
                context_id=request.context_id,
                model_id=request.model_id,
                ciphertext_inputs=list(request.ciphertext_inputs),
                metadata=dict(request.metadata),
                idempotency_key=request.idempotency_key or None,
            )
            self._job_queue.submit(job)
            logger.info(f"Job submitted: {job.job_id}")
            return {"job_id": job.job_id, "status": JobStatus.QUEUED.value}
        except ValueError as e:
            context.set_code(grpc.StatusCode.RESOURCE_EXHAUSTED)
            context.set_details(str(e))
            return None

    def GetJobStatus(self, request: Any, context: grpc.ServicerContext) -> Any:
        """Get job status."""
        job = self._job_queue.get_job(request.job_id)
        if job is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Job not found")
            return None

        return {
            "job_id": job.job_id,
            "status": job.status.value,
            "error_message": job.error_message or "",
            "created_at_unix": int(job.created_at),
            "started_at_unix": int(job.started_at) if job.started_at else 0,
            "completed_at_unix": int(job.completed_at) if job.completed_at else 0,
        }

    def FetchResult(self, request: Any, context: grpc.ServicerContext) -> Any:
        """Fetch job result."""
        from moai_service.job_queue import JobStatus

        job = self._job_queue.get_job(request.job_id)
        if job is None:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Job not found")
            return None

        if job.status != JobStatus.COMPLETED:
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            context.set_details(f"Job not completed: {job.status.name}")
            return None

        return {
            "job_id": job.job_id,
            "ciphertext_outputs": job.ciphertext_outputs or [],
            "metadata": job.result_metadata or {},
        }

    def Health(self, request: Any, context: grpc.ServicerContext) -> Any:
        """Health check."""
        return {
            "status": "healthy",
            "version": "0.1.0",
            "backend": self._backend.name,
        }

    def ListModels(self, request: Any, context: grpc.ServicerContext) -> Any:
        """List available models."""
        models = []
        for model_id, model in self._model_registry.items():
            models.append({
                "model_id": model_id,
                "input_dim": model.input_dim,
                "output_dim": model.output_dim,
                "scheme": "ckks",
                "description": getattr(model, "description", ""),
                "model_type": type(model).__name__,
            })
        return {"models": models}


def create_grpc_server(
    backend: Any,
    model_registry: dict[str, Any],
    session_store: Any,
    job_queue: Any,
    port: int = 50051,
    max_workers: int = 10,
    use_tls: bool = False,
    server_cert: bytes | None = None,
    server_key: bytes | None = None,
    ca_cert: bytes | None = None,
) -> grpc.Server:
    """Create gRPC server.

    Args:
        backend: FHE backend.
        model_registry: Models.
        session_store: Session store.
        job_queue: Job queue.
        port: Port to bind.
        max_workers: Thread pool size.
        use_tls: Enable TLS.
        server_cert: Server certificate (PEM).
        server_key: Server private key (PEM).
        ca_cert: CA certificate for mTLS (PEM).

    Returns:
        Configured gRPC server.
    """
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))

    servicer = MOAIServicer(
        backend=backend,
        model_registry=model_registry,
        session_store=session_store,
        job_queue=job_queue,
    )

    # Note: In production, add the generated servicer:
    # moai_pb2_grpc.add_MOAIServiceServicer_to_server(servicer, server)

    if use_tls and server_cert and server_key:
        if ca_cert:
            # mTLS - require client certificate
            credentials = grpc.ssl_server_credentials(
                [(server_key, server_cert)],
                root_certificates=ca_cert,
                require_client_auth=True,
            )
        else:
            # TLS only
            credentials = grpc.ssl_server_credentials(
                [(server_key, server_cert)]
            )
        server.add_secure_port(f"[::]:{port}", credentials)
        logger.info(f"gRPC server with {'mTLS' if ca_cert else 'TLS'} on port {port}")
    else:
        server.add_insecure_port(f"[::]:{port}")
        logger.warning(f"gRPC server INSECURE on port {port}")

    return server


def run_grpc_server(
    port: int = 50051,
    block: bool = True,
) -> grpc.Server:
    """Run the gRPC server with default configuration.

    Args:
        port: Port to bind.
        block: Whether to block until shutdown.

    Returns:
        Running server.
    """
    from moai_core.backends.tenseal_ckks import TenSEALCKKSBackend
    from moai_core.model_registry import get_global_registry, register_default_models
    from moai_service.job_queue import JobQueue
    from fhe_inference_service.server.session_store import SessionStore

    # Initialize components
    backend = TenSEALCKKSBackend()
    register_default_models()
    registry = get_global_registry()
    session_store = SessionStore()
    job_queue = JobQueue()

    server = create_grpc_server(
        backend=backend,
        model_registry=registry._models,
        session_store=session_store,
        job_queue=job_queue,
        port=port,
    )

    server.start()
    logger.info(f"MOAI gRPC server started on port {port}")

    if block:
        server.wait_for_termination()

    return server


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    run_grpc_server()
