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


def test_corridor_preserves_research_layout():
    xml, _ = build_scene(SceneSpec(map="two_room_corridor"))
    model = mujoco.MjModel.from_xml_string(xml)
    expected = {
        "room_right_upper": ((-0.3, 0.6, 0.1), (0.025, 0.07, 0.1)),
        "room_right_lower": ((-0.3, 0.1, 0.1), (0.025, 0.19, 0.1)),
        "room_bottom_left": ((-1.085, -0.1, 0.1), (0.16, 0.025, 0.1)),
        "room_bottom_right": ((-0.515, -0.1, 0.1), (0.21, 0.025, 0.1)),
        "room2_left_upper": ((0.2, -0.12, 0.1), (0.025, 0.29, 0.1)),
        "room2_left_lower": ((0.2, -0.63, 0.1), (0.025, 0.04, 0.1)),
        "room2_top_left": ((0.33, 0.2, 0.1), (0.13, 0.025, 0.1)),
        "room2_top_right": ((0.72, 0.2, 0.1), (0.08, 0.025, 0.1)),
        "obstacle_1": ((-0.9, 0.35, 0.1), (0.06, 0.06, 0.1)),
        "obstacle_2": ((-0.55, 0.1, 0.1), (0.06, 0.06, 0.1)),
        "obstacle_3": ((0.4, -0.05, 0.1), (0.06, 0.06, 0.1)),
        "obstacle_4": ((0.6, -0.5, 0.1), (0.06, 0.06, 0.1)),
    }
    for name, (position, size) in expected.items():
        geom_id = model.geom(name).id
        np.testing.assert_allclose(model.geom_pos[geom_id], position)
        np.testing.assert_allclose(model.geom_size[geom_id], size)


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
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_board_front") == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_board_front") == -1
    assert manifest.pushers["left"]["collision_geoms"] == ("left_chassis",)


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
