#!/usr/bin/env python3
"""Incident Triage Demo.

Demonstrates the full MOAI FHE inference pipeline:
1. Simulate an incident event
2. Edge agent encrypts and submits to MOAI
3. Decrypt and display classification result

This is an acceptance test for the MOAI system.
"""

from __future__ import annotations

import uuid
from datetime import datetime

import numpy as np

from moai_client_sdk import MOAIClient
from robops_edge_agent import EdgeAgent, RobotOpsEvent
from robops_edge_agent.events import EventType


def main() -> None:
    """Run incident triage demo."""
    print("=" * 70)
    print("MOAI INCIDENT TRIAGE DEMO")
    print("=" * 70)

    server_url = "http://localhost:8000"
    print(f"\n[1/5] Connecting to MOAI service at {server_url}")

    # Create MOAI client
    client = MOAIClient(server_url=server_url)

    try:
        health = client.health_check()
        print(f"       Server status: {health['status']}")
    except Exception as e:
        print(f"\n❌ ERROR: Cannot connect to server")
        print(f"   {e}")
        print("\n   Make sure the server is running:")
        print("   $ python -m uvicorn fhe_inference_service.server.main:app --port 8000")
        return

    # Setup keys
    print("\n[2/5] Generating FHE keys locally...")
    client.setup()
    print("       ✓ Secret key stays on client (NEVER sent to server)")

    # Register eval keys
    print("\n[3/5] Registering evaluation keys with server...")
    client.register_keys()
    print(f"       Session ID: {client.session_id}")

    # Create edge agent
    print("\n[4/5] Setting up Edge Agent...")
    agent = EdgeAgent(moai_client=client, default_model_id="demo-linear-classifier")

    # Simulate incident event
    print("\n[5/5] Processing simulated incident event...")

    incident_event = RobotOpsEvent(
        event_id=str(uuid.uuid4()),
        event_type=EventType.INCIDENT,
        timestamp=datetime.now(),
        source_platform="demo",
        robot_id="robot-001",
        text_content="Robot arm collision detected during pick operation in zone A. "
                     "Emergency stop triggered. Operator intervention required.",
        metadata={
            "zone": "A",
            "operation": "pick",
            "trigger": "collision_sensor",
        },
    )

    print(f"       Event ID: {incident_event.event_id}")
    print(f"       Robot: {incident_event.robot_id}")
    print(f"       Content: {incident_event.text_content[:50]}...")

    # Process through MOAI
    decision = agent.process_event(incident_event)

    # Display results
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)

    print(f"\n📊 Classification: {decision.class_label}")
    print(f"🎯 Predicted Class: {decision.predicted_class}")
    print(f"💪 Confidence: {decision.confidence:.2%}")

    if decision.probabilities:
        print("\n📈 All Probabilities:")
        labels = ["low", "medium", "high"]
        for i, prob in enumerate(decision.probabilities[:3]):
            label = labels[i] if i < len(labels) else f"class_{i}"
            print(f"    {label}: {prob:.2%}")

    if decision.latency_ms:
        print(f"\n⏱️  Server latency: {decision.latency_ms:.2f}ms")

    print("\n🔐 Security:")
    print("    ✓ Secret key: Never left client")
    print("    ✓ Incident text: Never sent to server (only encrypted embedding)")
    print("    ✓ Classification: Decrypted only on client")

    print("\n" + "=" * 70)
    print("Demo complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
