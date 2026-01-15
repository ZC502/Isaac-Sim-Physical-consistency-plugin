import numpy as np

class Octonion:
    def __init__(self, r=1.0, i=None):
        self.r = float(r)
        self.i = np.zeros(7, dtype=np.float64) if i is None else np.array(i, dtype=np.float64)

    def copy(self):
        return Octonion(self.r, self.i.copy())

    def normalize(self):
        n = np.sqrt(self.r * self.r + np.dot(self.i, self.i))
        if n > 0:
            self.r /= n
            self.i /= n

    def __mul__(self, other):
        """
        Strict translation of your C octonion_mult()
        Non-associative, non-commutative by construction
        """
        a = self
        b = other
        c = Octonion(0.0)

        # ---------- real part ----------
        c.r = (
            a.r * b.r
            - a.i[0]*b.i[0] - a.i[1]*b.i[1] - a.i[2]*b.i[2]
            - a.i[3]*b.i[3] - a.i[4]*b.i[4] - a.i[5]*b.i[5]
            - a.i[6]*b.i[6]
        )

        # ---------- imaginary parts ----------
        # e0
        c.i[0] = (
            a.r*b.i[0] + a.i[0]*b.r
            + (a.i[1]*b.i[2] - a.i[2]*b.i[1])
            + (a.i[5]*b.i[6] - a.i[6]*b.i[5])
            - (a.i[3]*b.i[4] - a.i[4]*b.i[3])
        )

        # e1
        c.i[1] = (
            a.r*b.i[1] + a.i[1]*b.r
            + (a.i[2]*b.i[0] - a.i[0]*b.i[2])
            + (a.i[3]*b.i[5] - a.i[5]*b.i[3])
            - (a.i[4]*b.i[6] - a.i[6]*b.i[4])
        )

        # e2
        c.i[2] = (
            a.r*b.i[2] + a.i[2]*b.r
            + (a.i[0]*b.i[1] - a.i[1]*b.i[0])
            + (a.i[4]*b.i[5] - a.i[5]*b.i[4])
            - (a.i[3]*b.i[6] - a.i[6]*b.i[3])
        )

        # e3  (rotation–translation coupling)
        c.i[3] = (
            a.r*b.i[3] + a.i[3]*b.r
            + (a.i[0]*b.i[5] - a.i[5]*b.i[0])
            + (a.i[1]*b.i[4] - a.i[4]*b.i[1])
            + (a.i[6]*b.i[2] - a.i[2]*b.i[6])
            + (a.i[0]*b.i[2] - a.i[2]*b.i[0])
            - (a.i[1]*b.i[0] - a.i[0]*b.i[1])
        )

        # e4
        c.i[4] = (
            a.r*b.i[4] + a.i[4]*b.r
            + (a.i[3]*b.i[0] - a.i[0]*b.i[3])
            + (a.i[1]*b.i[6] - a.i[6]*b.i[1])
            - (a.i[2]*b.i[5] - a.i[5]*b.i[2])
        )

        # e5
        c.i[5] = (
            a.r*b.i[5] + a.i[5]*b.r
            + (a.i[6]*b.i[0] - a.i[0]*b.i[6])
            + (a.i[3]*b.i[1] - a.i[1]*b.i[3])
            + (a.i[2]*b.i[4] - a.i[4]*b.i[2])
        )

        # e6 (disturbance coupling)
        c.i[6] = (
            a.r*b.i[6] + a.i[6]*b.r
            + (a.i[0]*b.i[4] - a.i[4]*b.i[0])
            + (a.i[1]*b.i[5] - a.i[5]*b.i[1])
            + (a.i[3]*b.i[2] - a.i[2]*b.i[3])
            + (a.i[0]*b.i[1] - a.i[1]*b.i[0])
            - (a.i[2]*b.i[3] - a.i[3]*b.i[2])
        )

        return c

"""
Note:
This is an octonion multiplication specifically designed for temporal semantic coupling. 
For the specific mathematical and physical principles, please download:
《Discrete Time Steps Are Killing Physical AI》
https://github.com/ZC502/TinyOEKF/blob/master/docs/Continuous_Physics_Solver_for_AI_Wang_Liu.pdf
"""

