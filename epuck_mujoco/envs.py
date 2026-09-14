"""Gymnasium presets for direct e-puck parcel pushing."""

from __future__ import annotations

import numpy as np

from .assets import BoxParcel, EpuckPusher
from .scene import SceneSpec, build_scene

try:
    import gymnasium as gym
    from gymnasium.envs.registration import register, registry
except ImportError:  # Scene generation remains usable without runtime extras.
    gym = None
    registry = {}


class EpuckPushEnv(gym.Env if gym else object):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 20}

    def __init__(
        self,
        preset: str,
        render_mode: str | None = None,
        max_episode_steps: int = 400,
        control_dt: float = 0.05,
        success_tolerance: float = 0.08,
    ):
        if gym is None:
            raise ImportError("gymnasium is required to use the registered environments")
        if render_mode not in {None, "rgb_array"}:
            raise ValueError("render_mode must be None or 'rgb_array'")

        import mujoco

        self.preset = preset
        self.render_mode = render_mode
        self.max_episode_steps = int(max_episode_steps)
        self.control_dt = float(control_dt)
        self.success_tolerance = float(success_tolerance)
        self._pusher_asset = EpuckPusher("robot", role="corridor" if preset == "two_room_corridor" else "room")
        self._parcel_asset = BoxParcel("parcel")
        self.xml, self.manifest = build_scene(
            SceneSpec(map=preset, pushers=(self._pusher_asset,), parcels=(self._parcel_asset,))
        )
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.renderer = None
        self.camera = None

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32
        )
        self._steps = 0
        self._distance = np.inf

    @property
    def pusher_name(self) -> str:
        return next(iter(self.manifest.pushers))

    @property
    def parcel_name(self) -> str:
        return next(iter(self.manifest.parcels))

    def _observation(self) -> np.ndarray:
        robot_id = self.model.body(self.manifest.pushers[self.pusher_name]["body"]).id
        parcel_id = self.model.body(self.manifest.parcels[self.parcel_name]["body"]).id
        robot_xy = self.data.xpos[robot_id, :2]
        parcel_xy = self.data.xpos[parcel_id, :2]
        yaw = np.arctan2(self.data.xmat[robot_id, 3], self.data.xmat[robot_id, 0])
        spatial_velocity = self.data.cvel[robot_id]
        goal = np.asarray(self.manifest.goal, dtype=np.float64)
        return np.asarray(
            [
                robot_xy[0],
                robot_xy[1],
                np.sin(yaw),
                np.cos(yaw),
                spatial_velocity[3],
                spatial_velocity[4],
                spatial_velocity[2],
                *(parcel_xy - robot_xy),
                *(goal - parcel_xy),
            ],
            dtype=np.float32,
        )

    def _sample_xy(self, bounds) -> np.ndarray:
        x_min, x_max, y_min, y_max = bounds
        return np.asarray(
            [self.np_random.uniform(x_min, x_max), self.np_random.uniform(y_min, y_max)]
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.mujoco.mj_resetData(self.model, self.data)

        robot_xy = self._sample_xy(self.manifest.robot_spawn)
        parcel_xy = self._sample_xy(self.manifest.parcel_spawn)
        for _ in range(100):
            if np.linalg.norm(parcel_xy - robot_xy) >= 0.14:
                break
            parcel_xy = self._sample_xy(self.manifest.parcel_spawn)
        yaw = float(self.np_random.uniform(-np.pi, np.pi))

        robot_joint = self.manifest.pushers[self.pusher_name]["freejoint"]
        robot_qpos = int(self.model.joint(robot_joint).qposadr[0])
        self.data.qpos[robot_qpos : robot_qpos + 3] = (*robot_xy, self._pusher_asset.pos[2])
        self.data.qpos[robot_qpos + 3 : robot_qpos + 7] = (
            np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)
        )

        parcel_joint = self.manifest.parcels[self.parcel_name]["freejoint"]
        parcel_qpos = int(self.model.joint(parcel_joint).qposadr[0])
        self.data.qpos[parcel_qpos : parcel_qpos + 3] = (*parcel_xy, 0.04)
        self.data.qpos[parcel_qpos + 3 : parcel_qpos + 7] = (1.0, 0.0, 0.0, 0.0)

        self.mujoco.mj_forward(self.model, self.data)
        self._steps = 0
        self._distance = self._parcel_goal_distance()
        observation = self._observation()
        return observation, {"success": False, "distance": self._distance}

    def _parcel_goal_distance(self) -> float:
        body = self.manifest.parcels[self.parcel_name]["body"]
        parcel_xy = self.data.xpos[self.model.body(body).id, :2]
        return float(np.linalg.norm(parcel_xy - self.manifest.goal))

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (2,):
            raise ValueError("action must have shape (2,): normalized [linear, angular]")
        action = np.clip(action, -1.0, 1.0)

        linear_velocity = float(action[0]) * 0.13
        angular_velocity = float(action[1]) * 2.5
        radius = self._pusher_asset.wheel_radius
        half_track = self._pusher_asset.half_track
        max_speed = self._pusher_asset.max_wheel_speed
        left = np.clip(
            (linear_velocity - angular_velocity * half_track) / radius,
            -max_speed,
            max_speed,
        )
        right = np.clip(
            (linear_velocity + angular_velocity * half_track) / radius,
            -max_speed,
            max_speed,
        )
        actuator_names = self.manifest.pushers[self.pusher_name]["actuators"]
        self.data.ctrl[self.model.actuator(actuator_names[0]).id] = left
        self.data.ctrl[self.model.actuator(actuator_names[1]).id] = right

        physics_steps = max(1, round(self.control_dt / self.model.opt.timestep))
        for _ in range(physics_steps):
            self.mujoco.mj_step(self.model, self.data)

        self._steps += 1
        distance = self._parcel_goal_distance()
        success = distance <= self.success_tolerance
        reward = self._distance - distance - 0.001 * self.control_dt
        if success:
            reward += 1.0
        self._distance = distance
        truncated = self._steps >= self.max_episode_steps and not success
        observation = self._observation()
        return observation, float(reward), success, truncated, {
            "success": success,
            "distance": distance,
        }

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self.renderer is None:
            self.renderer = self.mujoco.Renderer(self.model, height=256, width=256)
            self.camera = self.mujoco.MjvCamera()
            self.camera.type = self.mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.lookat[:] = self.manifest.camera[:2] + (0.0,)
            self.camera.distance = self.manifest.camera[2]
            self.camera.azimuth = 90.0
            self.camera.elevation = -89.0
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()

    def close(self):
        if self.renderer is not None:
            if hasattr(self.renderer, "close"):
                self.renderer.close()
            self.renderer = None
            self.camera = None


class EpuckRoomPushEnv(EpuckPushEnv):
    def __init__(self, **kwargs):
        super().__init__(preset="single_room", **kwargs)


class EpuckCorridorPushEnv(EpuckPushEnv):
    def __init__(self, **kwargs):
        super().__init__(preset="two_room_corridor", **kwargs)


def register_envs() -> None:
    if gym is None:
        return
    entries = {
        "EpuckRoomPush-v0": "epuck_mujoco.envs:EpuckRoomPushEnv",
        "EpuckCorridorPush-v0": "epuck_mujoco.envs:EpuckCorridorPushEnv",
    }
    for env_id, entry_point in entries.items():
        if env_id not in registry:
            register(id=env_id, entry_point=entry_point)
