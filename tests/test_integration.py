"""Integration tests for end-to-end FHE inference pipeline.

Tests the full flow: client encrypt → server infer → client decrypt.
"""

import numpy as np
import pytest
from fastapi.testclient import TestClient

from fhe_inference_service.client.sdk import FHEClient
from fhe_inference_service.core.backend import ParamsConfig
from fhe_inference_service.core.backends.tenseal_ckks import TenSEALCKKSBackend
from fhe_inference_service.core.model_registry import register_default_models
from fhe_inference_service.core.models.linear_model import create_demo_linear_classifier
from fhe_inference_service.server.main import app


class TestEndToEndPipeline:
    """Test full encrypt → infer → decrypt pipeline."""

    @pytest.fixture(autouse=True)
    def setup_models(self) -> None:
        """Register models before tests."""
        register_default_models()

    @pytest.fixture
    def test_client(self) -> TestClient:
        """Create FastAPI test client."""
        return TestClient(app)

    def test_health_check(self, test_client: TestClient) -> None:
        """Test health endpoint."""
        response = test_client.get("/v1/health")
        assert response.status_code == 200

        data = response.json()
        assert data["status"] == "healthy"
        assert "version" in data
        assert data["backend"] == "TenSEAL-CKKS"

    def test_list_models(self, test_client: TestClient) -> None:
        """Test models listing endpoint."""
        response = test_client.get("/v1/models")
        assert response.status_code == 200

        data = response.json()
        assert "models" in data
        assert len(data["models"]) >= 2  # At least linear and MLP

        # Check model structure
        model = data["models"][0]
        assert "model_id" in model
        assert "input_dim" in model
        assert "output_dim" in model
        assert "scheme" in model

    def test_full_inference_pipeline(self, test_client: TestClient) -> None:
        """Test complete FHE inference pipeline.

        This is the main integration test:
        1. Client generates keys
        2. Client registers eval keys with server
        3. Client encrypts input
        4. Server runs encrypted inference
        5. Client decrypts output
        6. Verify output matches plaintext inference
        """
        # Setup backend and model for comparison
        backend = TenSEALCKKSBackend()
        config = ParamsConfig()
        model = create_demo_linear_classifier()

        # Generate keys
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # Register keys with server
        import base64

        context_b64 = base64.b64encode(keys.public_key).decode("ascii")
        eval_keys_b64 = base64.b64encode(keys.eval_keys).decode("ascii")

        register_response = test_client.post(
            "/v1/keys/register",
            json={
                "session_id": "test-session-123",
                "fhe_scheme": "ckks",
                "params": {},
                "context_b64": context_b64,
                "eval_keys_b64": eval_keys_b64,
            },
        )
        assert register_response.status_code == 201

        # Create input embedding
        embedding = np.random.randn(64).astype(np.float64)

        # Get plaintext reference
        plaintext_logits = model.forward_plaintext(embedding)

        # Encrypt input
        ciphertext = backend.encrypt(context, keys.secret_key, embedding)
        ct_b64 = base64.b64encode(ciphertext).decode("ascii")

        # Run inference
        infer_response = test_client.post(
            "/v1/infer",
            json={
                "session_id": "test-session-123",
                "model_id": "demo-linear-classifier",
                "ciphertext_input_b64": ct_b64,
            },
        )
        assert infer_response.status_code == 200

        data = infer_response.json()
        assert "ciphertext_output_b64" in data
        assert "metadata" in data
        assert data["metadata"]["model_id"] == "demo-linear-classifier"

        # Decrypt output
        ct_output = base64.b64decode(data["ciphertext_output_b64"])
        encrypted_logits = backend.decrypt(context, keys.secret_key, ct_output, 3)

        # Verify output matches plaintext
        np.testing.assert_allclose(
            encrypted_logits, plaintext_logits, rtol=1e-2, atol=1e-3
        )

        # Verify predicted class matches
        assert np.argmax(encrypted_logits) == np.argmax(plaintext_logits)

    def test_session_not_found(self, test_client: TestClient) -> None:
        """Test error when session doesn't exist."""
        response = test_client.post(
            "/v1/infer",
            json={
                "session_id": "nonexistent-session",
                "model_id": "demo-linear-classifier",
                "ciphertext_input_b64": "dGVzdA==",  # Just "test" in base64
            },
        )
        assert response.status_code == 404
        assert "Session not found" in response.json()["detail"]

    def test_model_not_found(self, test_client: TestClient) -> None:
        """Test error when model doesn't exist."""
        # First register a valid session
        backend = TenSEALCKKSBackend()
        config = ParamsConfig()
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        import base64

        test_client.post(
            "/v1/keys/register",
            json={
                "session_id": "test-model-error",
                "fhe_scheme": "ckks",
                "params": {},
                "context_b64": base64.b64encode(keys.public_key).decode("ascii"),
                "eval_keys_b64": base64.b64encode(keys.eval_keys).decode("ascii"),
            },
        )

        # Try to infer with nonexistent model
        response = test_client.post(
            "/v1/infer",
            json={
                "session_id": "test-model-error",
                "model_id": "nonexistent-model",
                "ciphertext_input_b64": "dGVzdA==",
            },
        )
        assert response.status_code == 404
        assert "Model not found" in response.json()["detail"]


class TestClientSDK:
    """Test client SDK functionality (without server)."""

    def test_client_setup(self) -> None:
        """Test client key generation."""
        client = FHEClient.__new__(FHEClient)
        client.backend = TenSEALCKKSBackend()
        client.params_config = ParamsConfig()
        client._key_bundle = None
        client._context = None

        # Simulate setup
        client._context = client.backend.setup_context(client.params_config)
        client._key_bundle = client.backend.keygen(client._context)

        assert client._key_bundle is not None
        assert client._context is not None
        assert len(client._key_bundle.secret_key) > 0

    def test_client_encrypt_decrypt(self) -> None:
        """Test client-side encrypt/decrypt."""
        backend = TenSEALCKKSBackend()
        config = ParamsConfig()
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # Create mock client state
        plaintext = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float64)

        # Encrypt
        ciphertext = backend.encrypt(context, keys.secret_key, plaintext)
        assert len(ciphertext) > 0

        # Decrypt
        decrypted = backend.decrypt(context, keys.secret_key, ciphertext, len(plaintext))
        np.testing.assert_allclose(decrypted, plaintext, rtol=1e-3, atol=1e-5)
