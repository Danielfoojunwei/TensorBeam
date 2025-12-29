# Key Management Guide

## Overview

MOAI uses FHE (Fully Homomorphic Encryption) with CKKS scheme.
The **secret key** NEVER leaves the client. The server only receives evaluation keys.

## Key Types

| Key | Location | Purpose | Sensitivity |
|-----|----------|---------|-------------|
| Secret Key | Client only | Encrypt/Decrypt | CRITICAL |
| Public Key | Server | Encrypt (not used) | Low |
| Evaluation Keys | Server | Enable HE ops | Medium |
| Context | Both | FHE parameters | Low |

## Lifecycle

```
┌──────────────────────────────────────────────────────────────────┐
│                        KEY LIFECYCLE                              │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  1. GENERATION (Client)                                          │
│     client.setup()                                                │
│     ├─ Create CKKS context                                       │
│     ├─ Generate secret key (STAYS LOCAL)                         │
│     └─ Generate eval keys                                        │
│                                                                   │
│  2. REGISTRATION (Client → Server)                               │
│     client.register_keys()                                        │
│     ├─ Send: context + eval keys                                 │
│     ├─ DO NOT send: secret key                                   │
│     └─ Server stores with TTL                                    │
│                                                                   │
│  3. USAGE (During session)                                       │
│     ├─ Client encrypts with secret key                           │
│     ├─ Server computes on ciphertexts                            │
│     └─ Client decrypts with secret key                           │
│                                                                   │
│  4. EXPIRATION (Automatic)                                       │
│     ├─ Server deletes eval keys after TTL                        │
│     └─ Client generates fresh keys for new sessions              │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

## Configuration

### Session TTL
```yaml
# Helm values
moai:
  sessionTtlSeconds: 3600  # 1 hour default
```

Recommendations:
- General use: 1 hour
- Sensitive data: 15 minutes
- Long-running jobs: 4 hours (max recommended)

### Key Parameters
```python
from moai_core.backend import ParamsConfig

# Default parameters
config = ParamsConfig(
    poly_modulus_degree=8192,     # Security/performance tradeoff
    coeff_mod_bit_sizes=[60, 40, 40, 60],  # Multiplicative depth
    global_scale=2**40,           # Precision
)
```

| Parameter | Low Security | Standard | High Security |
|-----------|-------------|----------|---------------|
| poly_modulus_degree | 4096 | 8192 | 16384 |
| Security Level | ~80-bit | ~128-bit | ~192-bit |
| Performance | Fast | Balanced | Slow |

## Key Rotation

### Per-Session (Recommended)
```python
# Generate fresh keys per task
client = MOAIClient(server_url)
client.setup()  # New keys
client.register_keys()

# ... perform inference ...

# Keys expire automatically
```

### Explicit Rotation
```python
# Force new session
client.session_id = str(uuid.uuid4())
client.setup()
client.register_keys()
```

## Tenant Isolation

Each tenant has isolated:
- Session ID
- Evaluation keys
- Context parameters

Server cannot:
- Mix keys between tenants
- Access secret keys
- Decrypt any ciphertexts

## Secure Storage (Client-Side)

### Robot/Edge Device
```python
# Option 1: Environment variable (dev only)
import os
secret_key_b64 = os.environ["MOAI_SECRET_KEY"]

# Option 2: Secure enclave (production)
from secure_enclave import SecureStorage
storage = SecureStorage()
secret_key = storage.get("moai_secret_key")

# Option 3: TPM (recommended for robots)
from tpm_client import TPMClient
tpm = TPMClient()
secret_key = tpm.unseal("moai_sk_handle")
```

### Never Do
- ❌ Log secret keys
- ❌ Transmit secret keys over network
- ❌ Store in plain text on disk
- ❌ Include in crash dumps

## Cloud KMS Integration (Optional)

For envelope encryption of client-side secrets:

```python
# Wrap secret key with KMS
from cloud_kms import KMSClient

kms = KMSClient(key_id="projects/xxx/keys/moai-key")
wrapped_sk = kms.encrypt(secret_key)

# Store wrapped_sk (encrypted)
# Unwrap when needed
secret_key = kms.decrypt(wrapped_sk)
```

## Audit Logging

Server logs should include:
- Session ID (yes)
- Context ID (yes)
- Model ID (yes)
- Timestamps (yes)
- Key material (NO)
- Ciphertext content (NO)

```python
# Good log
logger.info(f"Session registered: {session_id}, TTL: {ttl}")

# Bad log - NEVER DO THIS
logger.info(f"Keys: {eval_keys[:100]}")  # ❌
```

## Emergency Procedures

### Suspected Key Compromise
1. **Eval keys only**: No decryption possible, but regenerate keys
2. **Secret key**: Rotate all affected client keys immediately

### Server Breach
- Eval keys cannot decrypt
- Attacker could compute on existing ciphertexts (limited damage)
- Rotate all client sessions
- Audit what was accessible

## Best Practices

1. **Ephemeral sessions**: New keys per logical task
2. **Short TTL**: Minimize exposure window
3. **Audit trail**: Log session lifecycle
4. **Secure client storage**: Use HSM/TPM when available
5. **Network security**: Always use mTLS in production
