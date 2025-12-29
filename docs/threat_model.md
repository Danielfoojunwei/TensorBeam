# MOAI FHE System - Threat Model

> [!CAUTION]
> **DO NOT use for real secrets unless you fully understand FHE limitations.**

## 1. Security Model

### Adversary Model
- **Honest-but-curious** cloud operator (CSP)
- Network eavesdropper (mitigated by TLS)
- Malicious security NOT provided

### Trust Boundaries
| Entity | Trust Level |
|--------|-------------|
| Robot/Edge | Trusted (has secret key) |
| Edge Agent | Trusted |
| Network | Untrusted (use mTLS) |
| MOAI Service | Untrusted (curious CSP) |
| Model Weights | Plaintext on server |

## 2. What IS Protected

| Asset | Protection |
|-------|-----------|
| Input embeddings | CKKS encryption |
| Output logits | CKKS encryption |
| Secret key | Never leaves client |
| Inference decisions | Decrypted only on client |

## 3. What is NOT Protected

> [!WARNING]
> The following are explicitly NOT protected:

| Asset | Status |
|-------|--------|
| Model weights | Plaintext on server |
| Access patterns | Visible (timing, frequency) |
| Ciphertext sizes | May leak structure info |
| Model architecture | Known to server |
| Computation integrity | No verification |

## 4. Key Management

### Secret Key
- Generated client-side
- **Never transmitted**
- Stored in robot's secure enclave (if available)

### Evaluation Keys
- Transmitted to server
- Enable homomorphic ops without decryption
- Per-session, TTL-based expiration
- No decrypt capability

### Key Rotation
- Recommended: new session per task
- Default TTL: 1 hour
- Explicit deletion supported

## 5. Session Security

```
Session Lifecycle:
┌─────────┐  keygen   ┌─────────┐  register  ┌─────────┐
│ Created │ ────────▶ │  Active │ ─────────▶ │ In Use  │
└─────────┘           └─────────┘            └─────────┘
                                                  │
                           TTL expiry / delete    │
                                                  ▼
                                            ┌─────────┐
                                            │ Expired │
                                            └─────────┘
```

## 6. Network Security

- **mTLS** required for production
- Per-tenant certificates
- Rate limiting per tenant

## 7. Attack Surface

| Vector | Risk | Mitigation |
|--------|------|------------|
| Key theft | High | Never transmit SK |
| Replay attack | Medium | Session scoping |
| Timing attack | Low | Constant-time not implemented |
| DoS | Medium | Rate limiting |

## 8. Compliance Notes

- GDPR: Encrypted data may still be "personal data"
- Not a substitute for legal review
- Document data flows for audit

## 9. Recommendations

1. **Per-task sessions**: New keys per logical task
2. **Short TTL**: Reduce to 15 min for sensitive data
3. **Audit logging**: Log session IDs, not content
4. **Network isolation**: mTLS + private network if possible
