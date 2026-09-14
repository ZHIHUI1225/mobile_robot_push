import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")
mujoco = pytest.importorskip("mujoco")

import epuck_mujoco
from epuck_mujoco import Cable, EpuckPusher, SceneSpec, build_scene


@pytest.mark.parametrize("map_name", ["single_room", "two_room_corridor"])
def test_map_asset_compositions_compile(map_name):
    xml, manifest = build_scene(SceneSpec(map=map_name))
    model = mujoco.MjModel.from_xml_string(xml)
    assert model.nbody > 1
    assert manifest.pushers["pusher"]["attach_site"] == "pusher_attach"
    assert manifest.parcels["parcel"]["freejoint"] == "parcel_free"


def test_cable_attaches_two_pushers_and_simulates():
    left = EpuckPusher("left", pos=(-0.175, 0.0, 0.022))
    right = EpuckPusher("right", pos=(0.175, 0.0, 0.022))
    xml, manifest = build_scene(SceneSpec(map="two_room_corridor", pushers=(left, right), cable=Cable(), cable_sites=("left_attach", "right_attach")))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    for _ in range(20):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert manifest.cable["pusher_sites"] == ("left_attach", "right_attach")
    assert model.neq == 2


@pytest.mark.parametrize("env_id", ["EpuckRoomPush-v0", "EpuckCorridorPush-v0"])
def test_presets_reset_step_and_render(env_id):
    env = gym.make(env_id, render_mode="rgb_array")
    first, _ = env.reset(seed=4)
    second, _ = env.reset(seed=4)
    np.testing.assert_array_equal(first, second)
    assert env.observation_space.contains(first)
    for _ in range(10):
        observation, reward, terminated, truncated, info = env.step([0.5, 0.1])
        assert env.observation_space.contains(observation)
        assert np.isfinite(reward)
        assert np.isfinite(info["distance"])
        if terminated or truncated:
            break
    frame = env.render()
    assert frame.shape == (256, 256, 3)
    assert frame.dtype == np.uint8
    env.close()


def test_forward_command_moves_robot():
    env = gym.make("EpuckRoomPush-v0")
    env.reset(seed=9)
    body_id = env.unwrapped.model.body("robot").id
    start = env.unwrapped.data.xpos[body_id, :2].copy()
    for _ in range(20):
        env.step([1.0, 0.0])
    displacement = np.linalg.norm(env.unwrapped.data.xpos[body_id, :2] - start)
    env.close()
    assert displacement > 0.001
