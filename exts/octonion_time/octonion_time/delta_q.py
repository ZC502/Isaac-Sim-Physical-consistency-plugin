from .octonion import Octonion
import numpy as np


def compute_delta_q(dt, control, omega_norm):
    """
    Compute local octonion increment Δq representing
    temporal-semantic sub-step on manifold.
    """

    dq = Octonion()

    # ---- Temporal semantic core ----
    # Instead of r = dt, encode *rate-modulated time flow*
    # r ≈ 1 + α·dt   ensures identity when dt → 0
    alpha = 0.5 * omega_norm
    dq.r = 1.0 + alpha * dt

    # ---- Control-induced spatial drift (placeholder) ----
    dq.i[0:3] = control * dt

    # ---- Adaptive compute density / coupling dimension ----
    dq.i[6] = omega_norm * dt

    # ---- Numerical safety ----
    dq.normalize()
    return dq
