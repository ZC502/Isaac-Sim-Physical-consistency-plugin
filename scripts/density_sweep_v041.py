"""
density_sweep_v041.py
v0.4.1 — with Noise Baseline Calibration (auto)

Features
--------
- automatic noise floor estimation
- density sweep experiment
- scheduler-friendly entrypoint
- minimal dependencies
"""

import os
import json
import time
import numpy as np

# ============================================================
# 🔧 Utility
# ============================================================

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def timestamp():
    return time.strftime("%Y%m%d_%H%M%S")


# ============================================================
# ⭐ Noise Baseline Calibration (NEW in v0.4.1)
# ============================================================

def run_noise_baseline_calibration(
    sim_fn,
    samples: int = 20,
    seed: int = 42,
    save_path: str | None = None,
):
    """
    Estimate simulator noise floor.

    Parameters
    ----------
    sim_fn : callable
        Function returning a scalar metric.
        Called repeatedly under identical conditions.
    samples : int
        Number of repeated evaluations.
    seed : int
        RNG seed for reproducibility.
    save_path : str | None
        Optional JSON output.

    Returns
    -------
    dict
        {
            "noise_mean": float,
            "noise_std": float,
            "samples": int
        }
    """

    rng = np.random.default_rng(seed)

    values = []

    for i in range(samples):
        # Fixed conditions, multiple measurements
        val = sim_fn(rng)
        values.append(val)

    values = np.array(values)

    result = {
        "noise_mean": float(values.mean()),
        "noise_std": float(values.std(ddof=1)),
        "samples": samples,
    }

    print("\n[NoiseBaseline]")
    print(json.dumps(result, indent=2))

    if save_path is not None:
        ensure_dir(os.path.dirname(save_path))
        with open(save_path, "w") as f:
            json.dump(result, f, indent=2)

    return result


# ============================================================
# 🧪 Example simulator hook (YOU will replace later)
# ============================================================

def example_simulation(rng: np.random.Generator, density: float = 1.0):
    """
    Minimal stand-in for your MuJoCo / Isaac experiment.

    Replace this with your real rollout.

    Returns a scalar metric.
    """

    # ---- deterministic signal ----
    signal = density * 10.0

    # ---- stochastic noise (Simulate real physical noise) ----
    noise = rng.normal(0.0, 0.05)

    return signal + noise


# ============================================================
# 🚀 Density Sweep (v0.4.1 core)
# ============================================================

def run_density_sweep(
    densities,
    noise_floor: dict | None,
    seed: int = 123,
    save_path: str | None = None,
):
    """
    Run density sweep experiment.

    Parameters
    ----------
    densities : iterable[float]
    noise_floor : dict | None
        Output of noise calibration.
    """

    rng = np.random.default_rng(seed)

    results = []

    for d in densities:
        val = example_simulation(rng, density=d)

        entry = {
            "density": float(d),
            "metric": float(val),
        }

        # ⭐ Optional: Signal-to-Noise Ratio
        if noise_floor is not None and noise_floor["noise_std"] > 0:
            entry["snr"] = float(
                (val - noise_floor["noise_mean"])
                / noise_floor["noise_std"]
            )

        results.append(entry)

        print(f"[Sweep] density={d:.3f} metric={val:.6f}")

    if save_path is not None:
        ensure_dir(os.path.dirname(save_path))
        with open(save_path, "w") as f:
            json.dump(results, f, indent=2)

    return results


# ============================================================
# 🎯 Scheduler Entry (IMPORTANT)
# ============================================================

def run_experiment_v041(
    output_dir: str = "outputs_v041",
    do_noise_calibration: bool = True,
):
    """
    ⭐ Scheduler-friendly single entrypoint
    """

    ensure_dir(output_dir)

    # --------------------------------------------------------
    # 1️⃣ Noise calibration
    # --------------------------------------------------------
    noise_floor = None

    if do_noise_calibration:
        noise_floor = run_noise_baseline_calibration(
            sim_fn=lambda rng: example_simulation(rng, density=1.0),
            samples=20,
            save_path=os.path.join(
                output_dir, f"noise_floor_{timestamp()}.json"
            ),
        )

    # --------------------------------------------------------
    # 2️⃣ Density sweep
    # --------------------------------------------------------
    densities = np.linspace(0.5, 2.0, 8)

    results = run_density_sweep(
        densities=densities,
        noise_floor=noise_floor,
        save_path=os.path.join(
            output_dir, f"density_sweep_{timestamp()}.json"
        ),
    )

    return {
        "noise_floor": noise_floor,
        "results": results,
    }


# ============================================================
# 🧪 CLI
# ============================================================

if __name__ == "__main__":
    run_experiment_v041()
