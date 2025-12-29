"""Unit tests for serialization roundtrips.

Tests that ciphertext/key serialization works correctly.
"""

import numpy as np
import pytest

from fhe_inference_service.core.backend import ParamsConfig
from fhe_inference_service.core.backends.tenseal_ckks import TenSEALCKKSBackend


class TestSerializationRoundtrip:
    """Test serialization/deserialization of FHE objects."""

    @pytest.fixture
    def backend(self) -> TenSEALCKKSBackend:
        """Create a backend instance."""
        return TenSEALCKKSBackend()

    @pytest.fixture
    def config(self) -> ParamsConfig:
        """Create default params config."""
        return ParamsConfig()

    def test_context_roundtrip(
        self, backend: TenSEALCKKSBackend, config: ParamsConfig
    ) -> None:
        """Test context serialization and deserialization."""
        # Create context
        context = backend.setup_context(config)
        assert isinstance(context, bytes)
        assert len(context) > 0

        # Can generate keys from serialized context
        keys = backend.keygen(context)
        assert keys.secret_key is not None
        assert keys.eval_keys is not None

    def test_key_bundle_serialization(
        self, backend: TenSEALCKKSBackend, config: ParamsConfig
    ) -> None:
        """Test key bundle components are serializable bytes."""
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # All components should be bytes
        assert isinstance(keys.secret_key, bytes)
        assert isinstance(keys.public_key, bytes)
        assert isinstance(keys.eval_keys, bytes)
        assert isinstance(keys.context, bytes)

        # Sizes should be reasonable
        assert len(keys.secret_key) > 0
        assert len(keys.eval_keys) > 0

    def test_ciphertext_roundtrip(
        self, backend: TenSEALCKKSBackend, config: ParamsConfig
    ) -> None:
        """Test encrypt -> serialize -> deserialize -> decrypt."""
        # Setup
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # Original plaintext
        plaintext = np.array([1.0, 2.0, 3.0, 4.0, 5.0], dtype=np.float64)

        # Encrypt
        ciphertext = backend.encrypt(context, keys.secret_key, plaintext)
        assert isinstance(ciphertext, bytes)
        assert len(ciphertext) > 0

        # Decrypt
        decrypted = backend.decrypt(context, keys.secret_key, ciphertext, len(plaintext))

        # Should match within CKKS tolerance
        np.testing.assert_allclose(decrypted, plaintext, rtol=1e-3, atol=1e-5)

    def test_large_vector_roundtrip(
        self, backend: TenSEALCKKSBackend, config: ParamsConfig
    ) -> None:
        """Test roundtrip with larger vectors (embedding-sized)."""
        context = backend.setup_context(config)
        keys = backend.keygen(context)

        # 64-dimensional embedding (typical for demo)
        plaintext = np.random.randn(64).astype(np.float64)

        ciphertext = backend.encrypt(context, keys.secret_key, plaintext)
        decrypted = backend.decrypt(context, keys.secret_key, ciphertext, len(plaintext))

        np.testing.assert_allclose(decrypted, plaintext, rtol=1e-3, atol=1e-5)


class TestCiphertextOperations:
    """Test homomorphic operations on ciphertexts."""

    @pytest.fixture
    def backend(self) -> TenSEALCKKSBackend:
        return TenSEALCKKSBackend()

    @pytest.fixture
    def setup_encryption(self, backend: TenSEALCKKSBackend):
        """Set up context and keys for testing."""
        config = ParamsConfig()
        context = backend.setup_context(config)
        keys = backend.keygen(context)
        return context, keys

    def test_add_plain(self, backend: TenSEALCKKSBackend, setup_encryption) -> None:
        """Test adding plaintext to ciphertext."""
        context, keys = setup_encryption

        x = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        y = np.array([4.0, 5.0, 6.0], dtype=np.float64)

        ct_x = backend.encrypt(context, keys.secret_key, x)
        ct_x_loaded = backend.load_ciphertext(context, keys.eval_keys, ct_x)

        result = backend.add_plain(ct_x_loaded, y)
        result_bytes = backend.serialize_ciphertext(result)

        decrypted = backend.decrypt(context, keys.secret_key, result_bytes, 3)
        expected = x + y

        np.testing.assert_allclose(decrypted, expected, rtol=1e-3, atol=1e-5)

    def test_multiply_plain(self, backend: TenSEALCKKSBackend, setup_encryption) -> None:
        """Test multiplying ciphertext by plaintext."""
        context, keys = setup_encryption

        x = np.array([1.0, 2.0, 3.0], dtype=np.float64)
        y = np.array([2.0, 3.0, 4.0], dtype=np.float64)

        ct_x = backend.encrypt(context, keys.secret_key, x)
        ct_x_loaded = backend.load_ciphertext(context, keys.eval_keys, ct_x)

        result = backend.multiply_plain(ct_x_loaded, y)
        result_bytes = backend.serialize_ciphertext(result)

        decrypted = backend.decrypt(context, keys.secret_key, result_bytes, 3)
        expected = x * y

        np.testing.assert_allclose(decrypted, expected, rtol=1e-3, atol=1e-5)

    def test_square(self, backend: TenSEALCKKSBackend, setup_encryption) -> None:
        """Test squaring ciphertext (polynomial activation)."""
        context, keys = setup_encryption

        x = np.array([1.0, 2.0, 3.0], dtype=np.float64)

        ct_x = backend.encrypt(context, keys.secret_key, x)
        ct_x_loaded = backend.load_ciphertext(context, keys.eval_keys, ct_x)

        result = backend.square(ct_x_loaded)
        result_bytes = backend.serialize_ciphertext(result)

        decrypted = backend.decrypt(context, keys.secret_key, result_bytes, 3)
        expected = x**2

        np.testing.assert_allclose(decrypted, expected, rtol=1e-3, atol=1e-5)
