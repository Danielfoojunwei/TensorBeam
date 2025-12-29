# FHE Inference Service - API Specification

**Version**: v1  
**Base URL**: `http://localhost:8000`

---

## Overview

This API enables **encrypted inference** on sensitive data. The client encrypts inputs locally, the server runs inference on ciphertexts, and the client decrypts results. The server **never sees plaintext**.

---

## Authentication

> [!NOTE]
> v1 uses session-based key registration. Future versions will add API keys and OAuth.

---

## Endpoints

### Health Check

```
GET /v1/health
```

**Response** (200 OK):
```json
{
  "status": "healthy",
  "version": "0.1.0",
  "backend": "TenSEAL-CKKS"
}
```

---

### List Models

```
GET /v1/models
```

**Response** (200 OK):
```json
{
  "models": [
    {
      "model_id": "demo-linear-classifier",
      "input_dim": 64,
      "output_dim": 3,
      "scheme": "ckks",
      "description": "Demo linear classifier",
      "model_type": "linear"
    }
  ]
}
```

---

### Register Evaluation Keys

```
POST /v1/keys/register
```

Registers evaluation keys for a session. The **secret key must NEVER be sent**.

**Request Body**:
```json
{
  "session_id": "uuid-string",
  "fhe_scheme": "ckks",
  "params": {
    "poly_modulus_degree": 8192,
    "security_level": 128
  },
  "context_b64": "<base64-encoded-public-context>",
  "eval_keys_b64": "<base64-encoded-eval-keys>"
}
```

**Response** (201 Created):
```json
{
  "session_id": "uuid-string",
  "status": "registered",
  "message": "Evaluation keys registered successfully"
}
```

**Errors**:
- 400: Invalid base64 encoding or unsupported scheme

---

### Run Encrypted Inference

```
POST /v1/infer
```

Runs inference on encrypted input. Requires prior key registration.

**Request Body**:
```json
{
  "session_id": "uuid-string",
  "model_id": "demo-linear-classifier",
  "ciphertext_input_b64": "<base64-encoded-ciphertext>"
}
```

**Response** (200 OK):
```json
{
  "ciphertext_output_b64": "<base64-encoded-result>",
  "metadata": {
    "latency_ms": 245.67,
    "ct_input_bytes": 131072,
    "ct_output_bytes": 65536,
    "model_id": "demo-linear-classifier"
  }
}
```

**Errors**:
- 404: Session not found or expired
- 404: Model not found
- 400: Invalid base64 encoding
- 500: Inference failed

---

### Delete Session

```
DELETE /v1/keys/{session_id}
```

Deletes a session and its evaluation keys.

**Response** (200 OK):
```json
{
  "status": "deleted",
  "session_id": "uuid-string"
}
```

**Errors**:
- 404: Session not found

---

## Data Formats

### Ciphertext Encoding

All ciphertexts are transmitted as **base64-encoded bytes**.

```python
import base64

# Encode
ct_b64 = base64.b64encode(ciphertext_bytes).decode("ascii")

# Decode
ciphertext_bytes = base64.b64decode(ct_b64)
```

### Session IDs

Session IDs should be UUIDs (v4 recommended):

```python
import uuid
session_id = str(uuid.uuid4())
```

---

## Rate Limits

| Endpoint | Limit |
|----------|-------|
| Health check | Unlimited |
| List models | 100/min |
| Register keys | 10/min per IP |
| Inference | 60/min per session |

> [!NOTE]
> Rate limits are not enforced in v1. Production deployments should add rate limiting.

---

## Error Responses

All errors follow this format:

```json
{
  "detail": "Human-readable error message"
}
```

| Status | Meaning |
|--------|---------|
| 400 | Bad request (invalid input) |
| 404 | Resource not found |
| 422 | Validation error |
| 500 | Server error |

---

## Example Flow

```python
from fhe_inference_service.client import FHEClient

# 1. Create client
client = FHEClient("http://localhost:8000")

# 2. Generate keys locally
client.setup()

# 3. Register eval keys (secret key stays local)
client.register_keys()

# 4. Run encrypted inference
result = client.infer("demo-linear-classifier", embedding)

# 5. Use decrypted result
print(result.predicted_class)
```

---

## Versioning

API versions are prefixed: `/v1/`, `/v2/`, etc.

Breaking changes will increment the major version.
