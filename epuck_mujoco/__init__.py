from .assets import BoxParcel, Cable, EpuckPusher
from .scene import SceneManifest, SceneSpec, build_scene

__all__ = ["BoxParcel", "Cable", "EpuckPusher", "SceneManifest", "SceneSpec", "build_scene"]

try:
    from .envs import register_envs
    register_envs()
except ImportError:
    # Keep scene generation importable for source inspection without extras.
    pass
