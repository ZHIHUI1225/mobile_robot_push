"""Open or render any supported map/asset composition.

Examples:
  python examples/view_scene.py --map single_room
  python examples/view_scene.py --map two_room_corridor --pushers 2 --parcels 2
  python examples/view_scene.py --map two_room_corridor --pushers 2 --cable
  python examples/view_scene.py --map single_room --mode egl --output room.png
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from epuck_mujoco import BoxParcel, Cable, EpuckPusher, SceneSpec, build_scene


def make_spec(map_name: str, pusher_count: int, parcel_count: int, cable: bool) -> SceneSpec:
    if cable and pusher_count != 2:
        raise ValueError("--cable requires --pushers 2")

    center_x = -0.30 if map_name == "single_room" else -1.025
    pusher_y = 0.0 if map_name == "single_room" else -0.60
    if pusher_count == 1:
        pusher_x = (center_x,)
    else:
        # The default cable has 14 links x 0.025 m = 0.35 m.
        pusher_x = (center_x - 0.175, center_x + 0.175)
    pushers = tuple(
        EpuckPusher(f"robot_{index}", pos=(x, pusher_y, 0.022))
        for index, x in enumerate(pusher_x)
    )

    parcel_origin = 0.20 if map_name == "single_room" else -1.10
    parcel_y = 0.10 if map_name == "single_room" else -0.35
    parcels = tuple(
        BoxParcel(
            f"parcel_{index}",
            pos=(parcel_origin + 0.10 * (index // 2), parcel_y + 0.10 * (index % 2), 0.04),
        )
        for index in range(parcel_count)
    )

    cable_asset = Cable() if cable else None
    cable_sites = ("robot_0_attach", "robot_1_attach") if cable else None
    return SceneSpec(
        map=map_name,
        pushers=pushers,
        parcels=parcels,
        cable=cable_asset,
        cable_sites=cable_sites,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", choices=("single_room", "two_room_corridor"), default="single_room")
    parser.add_argument("--pushers", type=int, choices=(1, 2), default=1)
    parser.add_argument("--parcels", type=int, choices=range(1, 5), default=1)
    parser.add_argument("--cable", action="store_true")
    parser.add_argument("--mode", choices=("viewer", "egl", "check"), default="viewer")
    parser.add_argument("--output", type=Path, default=Path("scene.png"))
    parser.add_argument("--duration", type=float, default=0.0, help="Viewer seconds; 0 waits until closed")
    args = parser.parse_args()

    os.environ["MUJOCO_GL"] = "glfw" if args.mode == "viewer" else "egl"
    if args.mode == "viewer" and os.environ.get("EPUCK_VIEWER_GL") == "mesa":
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
        os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "mesa"
    import mujoco

    spec = make_spec(args.map, args.pushers, args.parcels, args.cable)
    xml, manifest = build_scene(spec)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(
        f"Loaded map={args.map}, pushers={args.pushers}, parcels={args.parcels}, "
        f"cable={args.cable}, bodies={model.nbody}"
    )

    if args.mode == "check":
        return
    if args.mode == "egl":
        from PIL import Image

        renderer = mujoco.Renderer(model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = (*manifest.camera[:2], 0.0)
        camera.distance = manifest.camera[2]
        camera.azimuth = 90.0
        camera.elevation = -89.0
        renderer.update_scene(data, camera=camera)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(args.output)
        if hasattr(renderer, "close"):
            renderer.close()
        print(f"Saved EGL render to {args.output.resolve()}")
        return

    import mujoco.viewer

    started = time.monotonic()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = (*manifest.camera[:2], 0.0)
        viewer.cam.distance = manifest.camera[2]
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -70.0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            if args.duration > 0 and time.monotonic() - started >= args.duration:
                break
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
