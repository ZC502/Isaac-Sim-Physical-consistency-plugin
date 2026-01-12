import numpy as np

class Octonion:
    def __init__(self, r=1.0, i=None):
        self.r = r
        self.i = np.zeros(7) if i is None else np.array(i)

    def normalize(self):
        n = np.sqrt(self.r**2 + np.dot(self.i, self.i))
        if n > 0:
            self.r /= n
            self.i /= n

    def __mul__(self, other):
        # 第一版：可以先写成 placeholder
        # 第二版：严格复现你 C 里的 octonion_mult
        c = Octonion()
        c.r = self.r * other.r - np.dot(self.i, other.i)
        # TODO: 完整虚部乘法（直接照抄你的 C）
        return c
