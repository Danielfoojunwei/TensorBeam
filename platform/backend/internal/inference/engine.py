"""Inference Interface (Adapter Pattern)."""

import abc
import time
import logging

logger = logging.getLogger(__name__)

class InferenceEngine(abc.ABC):
    @abc.abstractmethod
    def load_model(self, model_path: str):
        pass
        
    @abc.abstractmethod
    def register_keys(self, tenant_id: str, key_path: str):
        pass
        
    @abc.abstractmethod
    def submit_batch(self, tenant_id: str, model_id: str, input_data: bytes) -> bytes:
        pass

class MockInferenceEngine(InferenceEngine):
    """Mock engine using CPU simulation or simple delay."""
    
    def load_model(self, model_path: str):
        logger.info(f"MOCK: Loaded model from {model_path}")
        return "mock_model_handle"

    def register_keys(self, tenant_id: str, key_path: str):
        logger.info(f"MOCK: Registered keys for {tenant_id}")
        return True

    def submit_batch(self, tenant_id: str, model_id: str, input_data: bytes) -> bytes:
        logger.info(f"MOCK: Processing batch for {tenant_id} (size={len(input_data)})")
        time.sleep(0.5) # Simulate latency
        # Return dummy ciphertext output (same size as input for simplicity)
        return b"MOCK_OUTPUT_" + input_data[:32]

# TODO: Add NativeInferenceEngine using ctypes for libmoai_gpu.so
