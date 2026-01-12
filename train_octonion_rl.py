from omni.isaac.lab.app import AppLauncher
app = AppLauncher(headless=True).app

from omni.isaac.lab_tasks.utils import parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

from octonion_env import OctonionPendulumEnv

env_cfg = parse_env_cfg(
    "Pendulum",
    use_gpu=True,
    num_envs=256,
)

env = OctonionPendulumEnv(env_cfg)
env = RslRlVecEnvWrapper(env)

runner = OnPolicyRunner(
    env,
    device="cuda",
    num_steps_per_env=16,
    max_iterations=500
)

runner.learn()
