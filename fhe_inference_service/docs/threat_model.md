# Threat Model for FHE Inference Service

> [!CAUTION]
> **DO NOT use for real secrets unless you fully understand FHE limitations.**
> This service provides input confidentiality under specific assumptions.
> Review this threat model carefully before deploying with sensitive data.

## 1. Security Model

### 1.1 Adversary Model

The FHE Inference Service assumes an **honest-but-curious** (semi-honest) adversary:

| Actor | Trust Level | Capabilities |
|-------|-------------|--------------|
| **Client** | Trusted | Holds secret key, sees plaintext inputs/outputs |
| **Server/CSP** | Untrusted (curious) | Sees ciphertexts, eval keys, access patterns |
| **Network** | Untrusted | HTTPS protects transport; ciphertexts are encrypted regardless |

### 1.2 What IS Protected

| Asset | Protection | Notes |
|-------|-----------|-------|
| **Input embeddings** | Encrypted (CKKS) | Server only sees ciphertext bytes |
| **Output logits** | Encrypted (CKKS) | Decrypted only by client's secret key |
| **Secret key** | Never leaves client | Not transmitted, not stored server-side |

### 1.3 What is NOT Protected

> [!WARNING]
> The following are explicitly **not protected** by this design:

| Asset | Exposure | Mitigation |
|-------|----------|------------|
| **Model weights** | Plaintext on server | Model is not confidential to CSP |
| **Access patterns** | Visible to server | Session ID, timing, request frequency |
| **Timing information** | Visible | Inference latency reveals input complexity |
| **Ciphertext sizes** | Visible | May leak information about input structure |
| **Evaluation keys** | Transmitted to server | Contains no secret material, but see §2 |
| **Model architecture** | Known to CSP | Input/output dimensions are public |

---

## 2. Evaluation Key Exposure

### 2.1 What Evaluation Keys Contain

Evaluation keys enable homomorphic operations without the secret key:

- **Relinearization keys**: Required after ciphertext multiplication
- **Galois keys**: Required for rotations (slot permutations)

These keys are derived from the secret key but do **not** allow decryption.

### 2.2 Security of Evaluation Keys

| Property | Status |
|----------|--------|
| Can decrypt ciphertexts? | **No** |
| Can forge valid ciphertexts? | **No** |
| Leak any plaintext information? | **No** (under RLWE assumption) |
| Enable IND-CPA security? | Yes (standard CKKS security) |

### 2.3 Risks of Key Mishandling

| Risk | Impact | Mitigation |
|------|--------|------------|
| Key reuse across sessions | Correlation attacks possible | Per-session ephemeral keys |
| Long-lived keys | Increased exposure window | TTL-based expiration (1 hour default) |
| Key leakage to third party | Others could compute on your ciphertexts | Secure transmission (HTTPS) |

---

## 3. Session Security

### 3.1 Session Lifecycle

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Client      │     │ Network     │     │ Server      │
│ generates   │────▶│ HTTPS       │────▶│ stores      │
│ keys        │     │ transport   │     │ eval keys   │
└─────────────┘     └─────────────┘     └─────────────┘
                                               │
                                               ▼
                                        ┌─────────────┐
                                        │ TTL expiry  │
                                        │ (1 hour)    │
                                        └─────────────┘
```

### 3.2 Session Recommendations

1. **Ephemeral sessions**: Generate new keys per logical task
2. **Short TTL**: Default 1 hour; reduce for high-security contexts
3. **Explicit deletion**: Call `DELETE /v1/keys/{session_id}` when done
4. **No key persistence**: Current implementation is in-memory only

---

## 4. FHE Security Guarantees

### 4.1 CKKS Scheme Security

| Parameter | Value | Security Level |
|-----------|-------|----------------|
| `poly_modulus_degree` | 8192 | ~128-bit |
| Coefficient modulus | [60, 40, 40, 60] | Standard |
| Attack complexity | 2^128 operations | Secure under RLWE |

### 4.2 Known FHE Limitations

> [!IMPORTANT]
> FHE does not provide:

1. **Malicious security**: A malicious server could return garbage
2. **Integrity verification**: Client cannot verify computation correctness
3. **Pattern hiding**: Access patterns (when, how often) are visible
4. **Timing resistance**: Computation time may leak information

---

## 5. Attack Surface Analysis

### 5.1 Input/Output Channel

| Attack Vector | Risk | Status |
|---------------|------|--------|
| Ciphertext injection | Server returns wrong ciphertext | **Not mitigated** (honest-but-curious assumption) |
| Replay attacks | Old ciphertexts replayed | Session-scoped (mitigated) |
| Size-based inference | Large ciphertext = complex input | **Not mitigated** |

### 5.2 Side Channels

| Channel | Risk | Mitigation |
|---------|------|------------|
| Timing | Inference time varies | None (constant-time FHE not implemented) |
| Memory access patterns | Cache attacks | Depends on TenSEAL/SEAL |
| Power analysis | Physical access | Out of scope (remote service) |

---

## 6. Operational Security

### 6.1 Deployment Recommendations

1. **Network security**: Always use HTTPS in production
2. **Key isolation**: Never log or store secret keys
3. **Audit logging**: Log session IDs, not ciphertext contents
4. **Rate limiting**: Prevent DoS and enumeration attacks
5. **Input validation**: Validate ciphertext sizes before processing

### 6.2 Data Retention

| Data Type | Default Retention | Recommendation |
|-----------|-------------------|----------------|
| Ciphertext inputs | Not stored | Do not persist |
| Ciphertext outputs | Not stored | Do not persist |
| Evaluation keys | Session lifetime (1h) | Reduce as needed |
| Audit logs | Configurable | Retain session IDs only |

---

## 7. Compliance Considerations

### 7.1 Regulatory Notes

- **GDPR**: Encrypted data may still be "personal data" under GDPR
- **HIPAA**: FHE alone does not guarantee HIPAA compliance
- **PCI-DSS**: Consult security assessor for cryptographic controls

### 7.2 Audit Trail

The service logs:
- Session registration/deletion (no key material)
- Inference requests (session_id, model_id, latency)
- Errors (no plaintext content)

---

## 8. Future Security Enhancements

| Enhancement | Status | Description |
|-------------|--------|-------------|
| Zero-knowledge proofs | Planned | Verify computation correctness |
| Verifiable FHE | Research | Cryptographic result verification |
| Key rotation | v2 | Automatic key renewal |
| HSM integration | v2 | Hardware-backed key storage |
| Malicious security | Research | Resist active attackers |

---

## 9. Summary

### Protected
- ✅ Input embeddings (encrypted)
- ✅ Output logits (encrypted)
- ✅ Secret key (never transmitted)

### Not Protected
- ❌ Model weights (plaintext on server)
- ❌ Access patterns (visible)
- ❌ Timing information (visible)
- ❌ Computation integrity (honest-but-curious only)

> [!CAUTION]
> This service is designed for **input confidentiality** in an honest-but-curious setting.
> It is NOT a complete privacy solution. Evaluate your threat model carefully.
