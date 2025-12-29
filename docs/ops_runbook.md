# MOAI Ops Runbook

## Overview
Operational guide for running MOAI FHE Inference Service in production.

## Deployment

### Prerequisites
- Kubernetes cluster (1.24+)
- Helm 3.x
- Container registry access
- TLS certificates (for production)

### Quick Deploy
```bash
# Build and push images
docker build -f deploy/docker/Dockerfile.moai-service -t moai/moai-service:0.1.0 .
docker push moai/moai-service:0.1.0

# Deploy with Helm
helm install moai-service deploy/helm/moai-service \
  --namespace moai \
  --create-namespace
```

### Production Deploy
```bash
helm install moai-service deploy/helm/moai-service \
  --namespace moai \
  --create-namespace \
  --set replicaCount=3 \
  --set moai.mtlsEnabled=true \
  --set resources.limits.cpu=4000m \
  --set resources.limits.memory=8Gi
```

## Scaling

### Horizontal Scaling
```bash
# Manual scaling
kubectl scale deployment moai-service -n moai --replicas=5

# Enable autoscaling
helm upgrade moai-service deploy/helm/moai-service \
  --set autoscaling.enabled=true \
  --set autoscaling.minReplicas=2 \
  --set autoscaling.maxReplicas=10
```

### Vertical Scaling
Increase resources for FHE-intensive workloads:
```yaml
resources:
  limits:
    cpu: 8000m
    memory: 16Gi
```

## Monitoring

### Key Metrics
| Metric | Alert Threshold | Description |
|--------|----------------|-------------|
| `moai_queue_depth` | > 100 | Jobs waiting |
| `moai_job_latency_p99_ms` | > 5000 | Slow inference |
| `moai_job_failed_total` | > 10/min | Error rate |
| `moai_session_count` | > 1000 | Active sessions |

### Prometheus Queries
```promql
# Queue depth
sum(moai_queue_depth)

# Job latency P99
histogram_quantile(0.99, sum(rate(moai_job_latency_bucket[5m])) by (le))

# Error rate
sum(rate(moai_job_failed_total[5m]))
```

### Grafana Dashboard
Import: `deploy/grafana/moai-dashboard.json` (if available)

## Troubleshooting

### Job Failures
```bash
# Check logs
kubectl logs -n moai -l app=moai-service --tail=100

# Common causes:
# - Ciphertext size mismatch
# - Context ID not found
# - Model not found
```

### High Latency
1. Check queue depth: `moai_queue_depth`
2. Scale workers: increase `moai.workerPoolSize`
3. Add replicas: increase `replicaCount`

### Session Expiry
- Default TTL: 1 hour
- Increase via: `moai.sessionTtlSeconds`
- Clean up: sessions auto-expire

### Memory Issues
FHE operations are memory-intensive:
- Minimum: 2GB per worker
- Recommended: 4GB per worker
- For batching: 8GB+ per worker

## Incident Response

### Service Down
1. Check pod status: `kubectl get pods -n moai`
2. Check events: `kubectl describe pod <pod> -n moai`
3. Rollback if needed: `helm rollback moai-service`

### Data Breach (Eval Keys)
Eval keys alone cannot decrypt data. However:
1. Invalidate affected sessions
2. Rotate client keys
3. Audit access logs

### Performance Degradation
1. Check resource utilization
2. Scale horizontally
3. Enable Redis queue for persistence
4. Consider batching adjustments

## Backup and Recovery

### Stateless Service
MOAI service is stateless - no backup needed.
Sessions are ephemeral (in-memory or Redis).

### Redis Queue (if enabled)
```bash
# Backup Redis
kubectl exec -n moai redis-0 -- redis-cli BGSAVE

# Restore
kubectl cp backup.rdb redis-0:/data/dump.rdb -n moai
```

## Security Checklist

- [ ] mTLS enabled for gRPC
- [ ] Network policies configured
- [ ] Pod security standards enforced
- [ ] Secrets in K8s Secrets or KMS
- [ ] Audit logging enabled
- [ ] Rate limiting configured
- [ ] No secret keys stored server-side
