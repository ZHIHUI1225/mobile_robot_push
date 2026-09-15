"""Gymnasium presets for direct e-puck parcel pushing."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass, replace
from math import pi

import numpy as np

from .assets import BoxParcel, EpuckPusher
from .scene import SceneSpec, build_scene

try:
    import gymnasium as gym
    from gymnasium.envs.registration import register, registry
except ImportError:  # Scene generation remains usable without runtime extras.
    gym = None
    registry = {}


@dataclass(frozen=True)
class ResetConfig:
    """Spawn distribution and safety constraints for environment resets."""

    robot_spawn: tuple[float, float, float, float] | None = None
    parcel_spawn: tuple[float, float, float, float] | None = None
    robot_yaw_range: tuple[float, float] = (-pi, pi)
    min_robot_parcel_distance: float = 0.14
    min_parcel_goal_distance: float = 0.16
    obstacle_clearance: float = 0.005
    max_attempts: int = 1000


class EpuckPushEnv(gym.Env if gym else object):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 20}

    def __init__(
        self,
        preset: str,
        render_mode: str | None = None,
        max_episode_steps: int = 400,
        control_dt: float = 0.05,
        success_tolerance: float = 0.08,
        reset_config: ResetConfig | Mapping | None = None,
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
        if reset_config is None:
            self.reset_config = ResetConfig()
        elif isinstance(reset_config, ResetConfig):
            self.reset_config = reset_config
        elif isinstance(reset_config, Mapping):
            self.reset_config = ResetConfig(**reset_config)
        else:
            raise TypeError("reset_config must be ResetConfig, a mapping, or None")
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

        self._robot_geom_ids = tuple(
            self.model.geom(name).id
            for name in self.manifest.pushers[self.pusher_name]["collision_geoms"]
        )
        self._parcel_geom_id = self.model.geom(
            self.manifest.parcels[self.parcel_name]["collision_geom"]
        ).id
        floor_id = self.model.geom("floor").id
        self._static_geom_ids = tuple(
            geom_id
            for geom_id in range(self.model.ngeom)
            if self.model.geom_bodyid[geom_id] == 0 and geom_id != floor_id
        )

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

    @staticmethod
    def _validate_bounds(name, bounds) -> tuple[float, float, float, float]:
        values = np.asarray(bounds, dtype=np.float64)
        if values.shape != (4,) or not np.isfinite(values).all():
            raise ValueError(f"{name} must contain four finite values")
        if values[0] > values[1] or values[2] > values[3]:
            raise ValueError(f"{name} must be ordered as (x_min, x_max, y_min, y_max)")
        return tuple(float(value) for value in values)

    def _resolved_reset_config(self, options) -> tuple[ResetConfig, dict]:
        options = {} if options is None else dict(options)
        exact_keys = {"robot_xy", "parcel_xy", "robot_yaw"}
        config_keys = set(asdict(self.reset_config))
        unknown = set(options) - exact_keys - config_keys
        if unknown:
            raise ValueError(f"Unknown reset options: {sorted(unknown)}")
        overrides = {key: options.pop(key) for key in tuple(options) if key in config_keys}
        config = replace(self.reset_config, **overrides)
        robot_spawn = self._validate_bounds(
            "robot_spawn", config.robot_spawn or self.manifest.robot_spawn
        )
        parcel_spawn = self._validate_bounds(
            "parcel_spawn", config.parcel_spawn or self.manifest.parcel_spawn
        )
        yaw_range = np.asarray(config.robot_yaw_range, dtype=np.float64)
        if yaw_range.shape != (2,) or not np.isfinite(yaw_range).all() or yaw_range[0] > yaw_range[1]:
            raise ValueError("robot_yaw_range must contain two ordered finite values")
        for name in (
            "min_robot_parcel_distance",
            "min_parcel_goal_distance",
            "obstacle_clearance",
        ):
            if not np.isfinite(getattr(config, name)) or getattr(config, name) < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if config.min_parcel_goal_distance < self.success_tolerance:
            raise ValueError("min_parcel_goal_distance must be at least success_tolerance")
        if not isinstance(config.max_attempts, int) or config.max_attempts < 1:
            raise ValueError("max_attempts must be a positive integer")
        return replace(
            config,
            robot_spawn=robot_spawn,
            parcel_spawn=parcel_spawn,
            robot_yaw_range=(float(yaw_range[0]), float(yaw_range[1])),
        ), options

    @staticmethod
    def _exact_value(options, name, shape):
        if name not in options:
            return None
        value = np.asarray(options[name], dtype=np.float64)
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError(f"{name} must have shape {shape} and contain finite values")
        return value

    def _set_spawn_state(self, robot_xy, parcel_xy, yaw) -> None:
        robot_joint = self.manifest.pushers[self.pusher_name]["freejoint"]
        robot_qpos = int(self.model.joint(robot_joint).qposadr[0])
        self.data.qpos[robot_qpos : robot_qpos + 3] = (*robot_xy, self._pusher_asset.pos[2])
        self.data.qpos[robot_qpos + 3 : robot_qpos + 7] = (
            np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)
        )
        parcel_joint = self.manifest.parcels[self.parcel_name]["freejoint"]
        parcel_qpos = int(self.model.joint(parcel_joint).qposadr[0])
        self.data.qpos[parcel_qpos : parcel_qpos + 3] = (*parcel_xy, self._parcel_asset.pos[2])
        self.data.qpos[parcel_qpos + 3 : parcel_qpos + 7] = (1.0, 0.0, 0.0, 0.0)
        self.mujoco.mj_forward(self.model, self.data)

    def _geom_distance(self, first: int, second: int, limit: float) -> float:
        return float(
            self.mujoco.mj_geomDistance(
                self.model, self.data, first, second, max(limit, 1e-9), np.zeros(6)
            )
        )

    def _spawn_is_safe(self, robot_xy, parcel_xy, config: ResetConfig) -> bool:
        if np.linalg.norm(parcel_xy - robot_xy) < config.min_robot_parcel_distance:
            return False
        if np.linalg.norm(parcel_xy - self.manifest.goal) < config.min_parcel_goal_distance:
            return False
        for dynamic_id in (*self._robot_geom_ids, self._parcel_geom_id):
            for static_id in self._static_geom_ids:
                if self._geom_distance(dynamic_id, static_id, config.obstacle_clearance) < config.obstacle_clearance:
                    return False
        for robot_id in self._robot_geom_ids:
            if self._geom_distance(robot_id, self._parcel_geom_id, config.obstacle_clearance) < config.obstacle_clearance:
                return False
        return True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.mujoco.mj_resetData(self.model, self.data)
        config, exact = self._resolved_reset_config(options)
        exact_robot = self._exact_value(exact, "robot_xy", (2,))
        exact_parcel = self._exact_value(exact, "parcel_xy", (2,))
        exact_yaw = self._exact_value(exact, "robot_yaw", ())
        for attempt in range(1, config.max_attempts + 1):
            robot_xy = exact_robot if exact_robot is not None else self._sample_xy(config.robot_spawn)
            parcel_xy = exact_parcel if exact_parcel is not None else self._sample_xy(config.parcel_spawn)
            yaw = float(exact_yaw) if exact_yaw is not None else float(
                self.np_random.uniform(*config.robot_yaw_range)
            )
            self._set_spawn_state(robot_xy, parcel_xy, yaw)
            if self._spawn_is_safe(robot_xy, parcel_xy, config):
                break
            if exact_robot is not None and exact_parcel is not None and exact_yaw is not None:
                raise ValueError("The requested exact reset state is not collision-safe")
        else:
            raise RuntimeError(
                f"Could not sample a collision-safe {self.preset} reset in "
                f"{config.max_attempts} attempts"
            )
        self._steps = 0
        self._distance = self._parcel_goal_distance()
        observation = self._observation()
        return observation, {
            "success": False,
            "distance": self._distance,
            "reset_attempts": attempt,
            "robot_xy": robot_xy.copy(),
            "parcel_xy": parcel_xy.copy(),
            "robot_yaw": yaw,
        }

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
