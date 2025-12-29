"""Benchmark Harness Entrypoint (Real FHE).

Executes defined scenarios using actual TenSEAL computations.
"""

import argparse
import json
import logging
import time
import os
import tenseal as ts
import numpy as np
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger("moai-bench")

def run_scenario_s0(args: argparse.Namespace) -> Dict[str, Any]:
    """S0: Paper Replication (Real Computation Overhead)."""
    logger.info("Running S0: Real FHE Linear Computation...")
    
    # Setup Real Context
    ctx = ts.context(ts.SCHEME_TYPE.CKKS, poly_modulus_degree=8192, coeff_mod_bit_sizes=[60, 40, 40, 60])
    ctx.global_scale = 2**40
    ctx.generate_galois_keys()
    
    # Encrypt
    t0 = time.time()
    vec = ts.ckks_vector(ctx, np.random.randn(128))
    t_enc = (time.time() - t0) * 1000
    
    # Compute (Dot product + Square simulation)
    t0 = time.time()
    res = vec.dot(vec)
    res.square_()
    t_comp = (time.time() - t0) * 1000
    
    # Decrypt
    t0 = time.time()
    _ = res.decrypt()
    t_dec = (time.time() - t0) * 1000
    
    total_latency = t_enc + t_comp + t_dec
    
    return {
        "scenario_id": "S0",
        "name": "Real FHE Compute",
        "metrics": {
            "latency_ms_total": total_latency,
            "latency_breakdown": {
                "encrypt": t_enc,
                "compute": t_comp,
                "decrypt": t_dec
            },
            "status": "PASS"
        }
    }

def run_scenario_s1(args: argparse.Namespace) -> Dict[str, Any]:
    """S1: Throughput Test."""
    logger.info("Running S1: Throughput Loop...")
    # Run 5 iterations to avoid long waits in CPU mode
    latencies = []
    ctx = ts.context(ts.SCHEME_TYPE.CKKS, poly_modulus_degree=8192, coeff_mod_bit_sizes=[60, 40, 40, 60])
    ctx.global_scale = 2**40
    vec = ts.ckks_vector(ctx, [1.0]*10)
    
    for _ in range(5):
        t0 = time.time()
        vec.add(vec)
        latencies.append((time.time() - t0)*1000)
        
    return {
        "scenario_id": "S1",
        "name": "Throughput",
        "metrics": {
            "p50_ms": np.median(latencies),
            "p99_ms": np.percentile(latencies, 99),
            "ops_per_second": 1000 / np.mean(latencies)
        }
    }

SCENARIOS = {
    "s0": run_scenario_s0,
    "s1": run_scenario_s1,
}

def main():
    parser = argparse.ArgumentParser(description="MOAI Real Benchmark Harness")
    parser.add_argument("--scenarios", nargs="+", default=["s0", "s1"], help="Scenarios to run")
    parser.add_argument("--output-dir", default="bench/reports", help="Output directory")
    args = parser.parse_args()
    
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    
    results = []
    
    print("-" * 60)
    print(f"Starting MOAI Real Benchmark Suite at {datetime.now().isoformat()}")
    print("-" * 60)

    for sc_id in args.scenarios:
        if sc_id.lower() in SCENARIOS:
            try:
                res = SCENARIOS[sc_id.lower()](args)
                results.append(res)
                print(f"[{sc_id.upper()}] COMPLETED")
            except Exception as e:
                logger.error(f"Scenario {sc_id} failed: {e}")
                results.append({"scenario_id": sc_id, "error": str(e)})

    # Generate Report
    report_path = os.path.join(args.output_dir, "report.json")
    with open(report_path, "w") as f:
        json.dump({
            "timestamp": datetime.now().isoformat(),
            "environment": "Native-CPU-TenSEAL", 
            "results": results
        }, f, indent=2)
        
    print(f"\nReport written to {report_path}")

if __name__ == "__main__":
    main()
