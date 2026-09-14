"""Fail fast unless MuJoCo can render through EGL inside the GPU container."""

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import numpy as np

import epuck_mujoco  # noqa: F401 - registers the environments


env = gym.make("EpuckRoomPush-v0", render_mode="rgb_array")
try:
    env.reset(seed=0)
    frame = env.render()
    if frame.shape != (256, 256, 3) or frame.dtype != np.uint8:
        raise RuntimeError(f"Unexpected render result: {frame.shape}, {frame.dtype}")
    print(f"MuJoCo EGL render OK: shape={frame.shape}, dtype={frame.dtype}")
finally:
    env.close()
