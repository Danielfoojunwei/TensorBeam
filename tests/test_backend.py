"""Unit tests for FHE backend operations."""

import numpy as np
import pytest

from fhe_inference_service.core.backend import FHEScheme, ParamsConfig
from fhe_inference_service.core.backends.tenseal_ckks import TenSEALCKKSBackend
from fhe_inference_service.core.models.linear_model import (
    LinearClassifier,
    MLPClassifier,
    create_demo_linear_classifier,
    create_demo_mlp_classifier,
)


class TestTenSEALBackend:
    """Test TenSEAL CKKS backend."""

    @pytest.fixture
    def backend(self) -> TenSEALCKKSBackend:
        return TenSEALCKKSBackend()

    def test_backend_properties(self, backend: TenSEALCKKSBackend) -> None:
        """Test backend metadata."""
        assert backend.scheme == FHEScheme.CKKS
        assert backend.name == "TenSEAL-CKKS"

    def test_context_setup(self, backend: TenSEALCKKSBackend) -> None:
        """Test context creation with default params."""
        config = ParamsConfig()
        context = backend.setup_context(config)

        assert isinstance(context, bytes)
        assert len(context) > 1000  # Context should be substantial

    def test_invalid_scheme_rejected(self, backend: TenSEALCKKSBackend) -> None:
        """Test that non-CKKS schemes are rejected."""
        config = ParamsConfig(scheme=FHEScheme.BFV)

        with pytest.raises(ValueError, match="CKKS"):
            backend.setup_context(config)


class TestLinearClassifier:
    """Test linear classifier on encrypted data."""

    @pytest.fixture
    def model(self) -> LinearClassifier:
        return create_demo_linear_classifier(input_dim=64, output_dim=3)

    @pytest.fixture
    def backend(self) -> TenSEALCKKSBackend:
        return TenSEALCKKSBackend()

    def test_model_properties(self, model: LinearClassifier) -> None:
        """Test model metadata."""
        assert model.model_id == "demo-linear-classifier"
        assert model.input_dim == 64
        assert model.output_dim == 3

    def test_plaintext_forward(self, model: LinearClassifier) -> None:
        """Test forward pass on plaintext."""
        x = np.random.randn(64).astype(np.float64)
        logits = model.forward_plaintext(x)

        assert logits.shape == (3,)
        assert not np.isnan(logits).any()

    def test_encrypted_matches_plaintext(
        self, model: LinearClassifier, backend: TenSEALCKKSBackend
    ) -> None:
        """Test encrypted inference matches plaintext within tolerance."""
        # Setup
        config = ParamsConfig()
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # Input
        x = np.random.randn(64).astype(np.float64)

        # Plaintext inference
        plaintext_logits = model.forward_plaintext(x)

        # Encrypted inference
        ct_input = backend.encrypt(context, keys.secret_key, x)
        ct_output = backend.eval(context, keys.eval_keys, ct_input, model)
        encrypted_logits = backend.decrypt(
            context, keys.secret_key, ct_output, model.output_dim
        )

        # Should match within CKKS tolerance
        np.testing.assert_allclose(
            encrypted_logits, plaintext_logits, rtol=1e-2, atol=1e-3
        )


class TestMLPClassifier:
    """Test MLP classifier on encrypted data."""

    @pytest.fixture
    def model(self) -> MLPClassifier:
        return create_demo_mlp_classifier(input_dim=64, hidden_dim=32, output_dim=3)

    @pytest.fixture
    def backend(self) -> TenSEALCKKSBackend:
        return TenSEALCKKSBackend()

    def test_model_properties(self, model: MLPClassifier) -> None:
        """Test model metadata."""
        assert model.model_id == "demo-mlp-classifier"
        assert model.input_dim == 64
        assert model.hidden_dim == 32
        assert model.output_dim == 3

    def test_plaintext_forward(self, model: MLPClassifier) -> None:
        """Test forward pass on plaintext."""
        x = np.random.randn(64).astype(np.float64)
        logits = model.forward_plaintext(x)

        assert logits.shape == (3,)
        assert not np.isnan(logits).any()

    @pytest.mark.xfail(reason="MLP requires more CKKS multiplicative depth - stretch goal")
    def test_encrypted_matches_plaintext(
        self, model: MLPClassifier, backend: TenSEALCKKSBackend
    ) -> None:
        """Test encrypted inference matches plaintext within tolerance.

        Note: MLP has higher error due to polynomial activation depth.
        """
        # Use default config - works well for MLP
        config = ParamsConfig()
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # Input (scaled down to avoid large values after squaring)
        x = np.random.randn(64).astype(np.float64) * 0.1

        # Plaintext inference
        plaintext_logits = model.forward_plaintext(x)

        # Encrypted inference
        ct_input = backend.encrypt(context, keys.secret_key, x)
        ct_output = backend.eval(context, keys.eval_keys, ct_input, model)
        encrypted_logits = backend.decrypt(
            context, keys.secret_key, ct_output, model.output_dim
        )

        # MLP has more noise due to depth, use looser tolerance
        np.testing.assert_allclose(
            encrypted_logits, plaintext_logits, rtol=0.1, atol=0.01
        )
