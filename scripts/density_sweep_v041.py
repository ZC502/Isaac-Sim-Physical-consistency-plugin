"""
density_sweep_v041.py
v0.4.2 — with Noise Baseline Calibration (auto)

Features
--------
- automatic noise floor estimation
- density × damping grid
- scheduler-friendly entrypoint
- simulator-agnostic adapter
"""

import os
import json
import time
import numpy as np


# ============================================================
# 🔌 Physical rollout adapter (v0.4.2)
# ============================================================

def physical_rollout_adapter(
    rng: np.random.Generator,
    density: float,
    joint_damping: float,
) -> float:
    """
    Unified hook point for real simulator backends.

    Default behavior:
        Falls back to example_simulation() so the script
        remains runnable out-of-the-box.

    Auditor integration:
        Replace the body of this function with an actual
        Isaac Sim / MuJoCo rollout.

    Returns
    -------
    float
        Scalar instability / drift metric.
    """
    return example_simulation(rng, density=density)


# ============================================================
# 🔧 Utility
# ============================================================

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def timestamp():
    return time.strftime("%Y%m%d_%H%M%S")


# ============================================================
# 🧪 Example simulator hook (fallback only)
# ============================================================

def example_simulation(rng: np.random.Generator, density: float) -> float:
    """
    Lightweight stochastic fallback simulator.

    NOTE:
        This is NOT the physical benchmark.
        It exists only to keep the sweep runnable when no
        simulator backend is attached.
    """
    signal = density * 10.0
    noise = rng.normal(0.0, 0.05)
    return signal + noise


# ============================================================
# ⭐ Noise Baseline Calibration
# ============================================================

def run_noise_baseline_calibration(
    sim_fn,
    samples: int = 20,
    seed: int = 42,
    save_path: str | None = None,
):
    """
    Estimate simulator noise floor.
    """

    rng = np.random.default_rng(seed)
    values = []

    for _ in range(samples):
        val = sim_fn(rng)
        values.append(val)

    values = np.array(values)

    noise_std = float(values.std(ddof=1)) if len(values) > 1 else 0.0

    result = {
        "noise_mean": float(values.mean()),
        "noise_std": noise_std,
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
# 🧪 Joint Damping × Density Grid (v0.4.2 core)
# ============================================================

def run_damping_density_grid(
    densities,
    dampings,
    noise_floor: dict | None,
    seeds=(0, 1, 2),
    early_stop_snr: float | None = 0.5,
    save_path: str | None = None,
):
    """
    2D grid experiment:
        density × joint damping
    """

    results = []

    for d in densities:
        for damping in dampings:

            seed_metrics = []

            for s in seeds:
                rng = np.random.default_rng(s)

                val = physical_rollout_adapter(
                    rng,
                    density=d,
                    joint_damping=damping,
                )

                seed_metrics.append(val)

            seed_metrics = np.array(seed_metrics)

            mean_val = float(seed_metrics.mean())
            std_val = (
                float(seed_metrics.std(ddof=1))
                if len(seed_metrics) > 1
                else 0.0
            )

            entry = {
                "density": float(d),
                "joint_damping": float(damping),
                "mean_metric": mean_val,
                "std_metric": std_val,
                "num_seeds": len(seeds),
            }

            # ⭐ Signal-to-noise ratio
            snr = None
            if noise_floor is not None and noise_floor["noise_std"] > 0:
                snr = (
                    mean_val - noise_floor["noise_mean"]
                ) / max(noise_floor["noise_std"], 1e-12)
                entry["snr"] = float(snr)

            print(
                f"[Grid] density={d:.3f} damping={damping:.4f} "
                f"mean={mean_val:.6f} std={std_val:.6f} "
                + (f"snr={snr:.3f}" if snr is not None else "")
            )

            results.append(entry)

            # 🛑 Early stop
            if (
                early_stop_snr is not None
                and snr is not None
                and snr < early_stop_snr
            ):
                print(
                    "[EarlyStop] Effect below noise floor, "
                    "stopping higher damping sweep for this density."
                )
                break

    if save_path is not None:
        ensure_dir(os.path.dirname(save_path))
        with open(save_path, "w") as f:
            json.dump(results, f, indent=2)

    return results


# ============================================================
# 🎯 Unified experiment entry
# ============================================================

def run_experiment_v042(
    output_dir: str = "outputs_v042",
    do_noise_calibration: bool = True,
):
    """
    v0.4.2 unified experiment entry
    """

    ensure_dir(output_dir)

    noise_floor = None

    if do_noise_calibration:
        noise_floor = run_noise_baseline_calibration(
            sim_fn=lambda rng: physical_rollout_adapter(
                rng,
                density=1.0,
                joint_damping=0.1,
            ),
            samples=20,
            save_path=os.path.join(
                output_dir,
                f"noise_floor_{timestamp()}.json",
            ),
        )

    densities = np.linspace(0.5, 2.0, 6)

    dampings = np.array([
        0.01,
        0.05,
        0.1,
        0.2,
        0.4,
    ])

    grid_results = run_damping_density_grid(
        densities=densities,
        dampings=dampings,
        noise_floor=noise_floor,
        seeds=(0, 1, 2),
        early_stop_snr=0.3,
        save_path=os.path.join(
            output_dir,
            f"damping_density_grid_{timestamp()}.json",
        ),
    )

    return {
        "noise_floor": noise_floor,
        "grid_results": grid_results,
    }


# ============================================================
# 🧪 CLI
# ============================================================

if __name__ == "__main__":
    run_experiment_v042()
