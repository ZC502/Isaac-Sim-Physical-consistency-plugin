import torch
from omni.isaac.lab.envs import DirectRLEnv
from omni.isaac.lab.utils.math import tensor_clamp

from octonion_time.octonion import Octonion

class OctonionPendulumEnv(DirectRLEnv):

    def __init__(self, cfg):
        super().__init__(cfg)
        self.q_oct = Octonion()

    def _pre_physics_step(self, actions):
        # standard torque
        self.actions = tensor_clamp(actions, -1.0, 1.0)

        # read angular velocity
        omega = self._robot.data.joint_vel[:, 0]
        omega_norm = torch.abs(omega).mean().item()

        # Δq
        dq = Octonion(
            r=self.step_dt,
            i=[omega_norm, 0, 0, 0, 0, 0, omega_norm]
        )
        dq.normalize()

        # non-commutative update
        self.q_oct = self.q_oct * dq
        self.q_oct.normalize()

    def _apply_action(self):
        self._robot.set_joint_effort_target(self.actions)

    def _get_observations(self):
        obs = torch.cat([
            self._robot.data.joint_pos,
            self._robot.data.joint_vel,
            torch.tensor(self.q_oct.i[:3], device=self.device).unsqueeze(0)
        ], dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        # penalize high associator → suppress jitter
        reward = -torch.norm(self._robot.data.joint_vel, dim=-1)
        return reward

    def _get_dones(self):
        return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
