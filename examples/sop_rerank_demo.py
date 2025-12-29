#!/usr/bin/env python3
"""SOP Rerank Demo.

Demonstrates encrypted reranking of SOPs (Standard Operating Procedures).

Flow:
1. Query: "robot collision in zone A"
2. Candidate SOPs (retrieved from mock knowledge base)
3. Score each (query, sop) pair using MOAI
4. Return ranked list

Note: This demo uses the linear classifier as a scoring proxy.
Production would use a cross-encoder reranker.
"""

from __future__ import annotations

import uuid
from datetime import datetime

import numpy as np

from moai_client_sdk import MOAIClient
from robops_edge_agent import EdgeAgent, RobotOpsEvent
from robops_edge_agent.events import EventType

# Mock SOP knowledge base
MOCK_SOPS = [
    {
        "id": "SOP-001",
        "title": "Collision Recovery Protocol",
        "content": "When a collision is detected: 1) Verify e-stop engaged. "
                   "2) Inspect impact zone for damage. 3) Run diagnostics. "
                   "4) Clear zone before restart.",
    },
    {
        "id": "SOP-002",
        "title": "Zone A Safety Guidelines",
        "content": "Zone A is a high-traffic area. Maintain 2m clearance from "
                   "robot arms. Alert operators before any manual intervention.",
    },
    {
        "id": "SOP-003",
        "title": "Battery Replacement Procedure",
        "content": "For battery replacement: 1) Power down robot. 2) Disconnect "
                   "battery terminals. 3) Remove old battery. 4) Install new battery.",
    },
    {
        "id": "SOP-004",
        "title": "Emergency Stop Procedures",
        "content": "In case of emergency: 1) Press nearest e-stop. 2) Alert floor "
                   "supervisor. 3) Clear area. 4) Document incident.",
    },
]


def compute_relevance_embedding(query: str, sop_text: str) -> np.ndarray:
    """Compute a deterministic embedding for (query, sop) pair.

    In production, use a proper text encoder (e.g., sentence-transformers).
    """
    combined = f"{query} [SEP] {sop_text}"
    np.random.seed(hash(combined) % (2**32))
    embedding = np.random.randn(64).astype(np.float64)
    return embedding / np.linalg.norm(embedding)


def main() -> None:
    """Run SOP rerank demo."""
    print("=" * 70)
    print("MOAI SOP RERANK DEMO")
    print("=" * 70)

    server_url = "http://localhost:8000"
    query = "robot collision in zone A need recovery steps"

    print(f"\nQuery: \"{query}\"")
    print(f"\nCandidate SOPs: {len(MOCK_SOPS)}")
    for sop in MOCK_SOPS:
        print(f"  - {sop['id']}: {sop['title']}")

    print(f"\n[1/4] Connecting to MOAI service...")
    client = MOAIClient(server_url=server_url)

    try:
        health = client.health_check()
        print(f"       Server status: {health['status']}")
    except Exception as e:
        print(f"\n❌ Server not available: {e}")
        return

    # Setup
    print("\n[2/4] Setting up FHE keys...")
    client.setup()
    client.register_keys()
    print(f"       Session: {client.session_id}")

    # Score each SOP
    print("\n[3/4] Scoring SOPs (encrypted)...")
    scores = []

    for sop in MOCK_SOPS:
        # Combine query and SOP for scoring
        embedding = compute_relevance_embedding(query, sop["content"])

        # Run encrypted inference
        result = client.infer(
            model_id="demo-linear-classifier",
            embedding=embedding,
            output_dim=3,
        )

        # Use the max logit as relevance score (simplified scoring)
        score = float(np.max(result.logits))
        scores.append((sop, score))
        print(f"       {sop['id']}: score={score:.4f}")

    # Rank by score
    print("\n[4/4] Ranking results...")
    ranked = sorted(scores, key=lambda x: x[1], reverse=True)

    print("\n" + "=" * 70)
    print("RANKED SOPs")
    print("=" * 70)

    for i, (sop, score) in enumerate(ranked, 1):
        print(f"\n#{i} {sop['id']}: {sop['title']}")
        print(f"   Score: {score:.4f}")
        print(f"   Preview: {sop['content'][:80]}...")

    print("\n" + "=" * 70)
    print("🔐 All scoring done on encrypted embeddings - query text protected")
    print("=" * 70)


if __name__ == "__main__":
    main()
