from .octonion import Octonion
import numpy as np

def compute_delta_q(dt, control, omega_norm):
    dq = Octonion()
    dq.r = dt

    # 控制输入编码进虚部 (Control the input to be encoded into the imaginary part)
    dq.i[0:3] = control * dt

    # e6 = coupling / compute density
    dq.i[6] = omega_norm * dt

    dq.normalize()
    return dq
