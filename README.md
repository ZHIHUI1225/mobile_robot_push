# epuck-mujoco

A small, composable MuJoCo environment for e-puck scale parcel pushing. Version 0.1 keeps the public surface narrow: three assets, two maps, and two ready-to-run Gymnasium presets.

## Install and run

The reproducible path uses Docker:

```bash
docker compose build
docker compose run --rm test
docker compose run --rm room
```

Start a persistent development container with the repository mounted at `/workspace`:

```bash
docker compose up -d dev
docker compose exec dev bash
```

Edits under `/workspace` inside the container and edits in the host repository are the same files. Stop the development container with `docker compose stop dev`; start it again with `docker compose start dev`.

On an NVIDIA Linux server, use the checked GPU launcher. It verifies the host GPU, container GPU access, source mount, and a MuJoCo EGL render before reporting success:

```bash
./scripts/start_gpu_dev.sh
# Or start and enter the shell:
./scripts/start_gpu_dev.sh --shell
```

The GPU overlay passes all NVIDIA devices, enables the `graphics` driver capability, forwards `$DISPLAY`, and mounts `/tmp/.X11-unix`. EGL works without a desktop session; an available X server additionally supports interactive viewer windows. The overlay stays separate so CPU-only Docker and GitHub Actions continue to use the portable base `compose.yaml`.

For interactive development, open the repository in VS Code and select **Dev Containers: Reopen in Container**. The same image can also provide a mounted shell:

```bash
docker compose run --rm shell
```

Native Python installation remains available:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
python examples/room.py
python -m pytest -q
```

```python
import gymnasium as gym
import epuck_mujoco

env = gym.make("EpuckRoomPush-v0", render_mode="rgb_array")
obs, info = env.reset(seed=7)
obs, reward, terminated, truncated, info = env.step([0.2, 0.0])
frame = env.render()
env.close()
```

## Included components

| Layer | Public names | Purpose |
|---|---|---|
| Assets | `EpuckPusher`, `BoxParcel`, `Cable` | Reusable robot, box, and articulated cable |
| Maps | `single_room`, `two_room_corridor` | Static geometry, goal, spawn bounds, and camera |
| Presets | `EpuckRoomPush-v0`, `EpuckCorridorPush-v0` | One e-puck and one box in a configured task |

Maps contain only static workspace geometry. `SceneSpec` composes named robot and parcel assets into either map, and `build_scene` returns both MJCF and a semantic manifest. Task code uses the manifest instead of fixed MuJoCo IDs.

```python
from epuck_mujoco import BoxParcel, EpuckPusher, SceneSpec, build_scene

xml, manifest = build_scene(SceneSpec(
    map="single_room",
    pushers=(EpuckPusher("carrier"),),
    parcels=(BoxParcel("package", size=(0.03, 0.02, 0.04)),),
))
```

Assets can be matched with either map. The builder accepts multiple named pushers and parcels. The two Gymnasium presets retain one pusher and one parcel so their observation and action contracts stay simple.

## Scene viewer

Use one command to inspect different map and asset combinations:

```bash
# Interactive X11 viewer
python examples/view_scene.py --map single_room
python examples/view_scene.py --map two_room_corridor --pushers 2 --parcels 2
python examples/view_scene.py --map two_room_corridor --pushers 2 --cable

# GPU EGL rendering without an X11 desktop
python examples/view_scene.py --map single_room --mode egl --output room.png
python examples/view_scene.py --map two_room_corridor --pushers 2 --cable \
  --mode egl --output cable_corridor.png
```

`--mode check` compiles a composition without opening or rendering it. Cable mode requires exactly two pushers because its endpoints attach to both robots.

## Cable attachment

`Cable` is an articulated asset connected between two `EpuckPusher` attachment sites. Its endpoint distance must match its configured length. It is available for custom scenes and is not a third registered task in v0.1.

```python
from epuck_mujoco import Cable, EpuckPusher, SceneSpec, build_scene

left = EpuckPusher("left", pos=(-0.175, 0.0, 0.022))
right = EpuckPusher("right", pos=(0.175, 0.0, 0.022))
xml, manifest = build_scene(SceneSpec(
    map="two_room_corridor",
    pushers=(left, right),
    cable=Cable(),
    cable_sites=("left_attach", "right_attach"),
))
```

## Environment contract

Actions are normalized linear/angular commands `[v, omega]` and are converted to differential wheel speeds. Observations contain robot position and heading, planar velocity, parcel-relative position, and parcel-to-goal displacement. The reward is distance progress minus a small time cost, with a success bonus when the parcel is within the goal tolerance.

Seeded resets are deterministic. Episodes terminate when the parcel reaches the goal and truncate at the time limit. Rendering supports `rgb_array`. The single-room map is a closed rectangular room with an east doorway and exterior goal. The corridor map contains two room areas joined by a central corridor.

Historical training checkpoints are not v0.1-compatible yet because observation ordering and model names have changed. Physical dimensions and actuator limits follow the research environment: 35 mm chassis radius, 20.5 mm wheel radius, 52 mm wheel track, 25 x 25 x 40 mm parcel half extents, and wheel speed limits of 6.34 rad/s.
