"""Scene composition and semantic model manifest."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, sin

from .assets import BoxParcel, Cable, EpuckPusher
from .maps import MAPS


@dataclass(frozen=True)
class SceneSpec:
    map: str = "single_room"
    pushers: tuple[EpuckPusher, ...] = field(
        default_factory=lambda: (EpuckPusher("pusher"),)
    )
    parcels: tuple[BoxParcel, ...] = field(
        default_factory=lambda: (BoxParcel("parcel"),)
    )
    cable: Cable | None = None
    cable_sites: tuple[str, str] | None = None


@dataclass(frozen=True)
class SceneManifest:
    pushers: dict[str, dict[str, object]]
    parcels: dict[str, dict[str, object]]
    cable: dict[str, object] | None
    goal: tuple[float, float]
    robot_spawn: tuple[float, float, float, float]
    parcel_spawn: tuple[float, float, float, float]
    camera: tuple[float, float, float]


def _attachment_world_position(pusher: EpuckPusher) -> tuple[float, float, float]:
    ox, oy, oz = pusher.attachment_offset
    x = pusher.pos[0] + ox * cos(pusher.yaw) - oy * sin(pusher.yaw)
    y = pusher.pos[1] + ox * sin(pusher.yaw) + oy * cos(pusher.yaw)
    return x, y, pusher.pos[2] + oz


def build_scene(spec: SceneSpec) -> tuple[str, SceneManifest]:
    if spec.map not in MAPS:
        raise ValueError(f"Unknown map: {spec.map}")
    if not spec.pushers or not spec.parcels:
        raise ValueError("A scene requires at least one pusher and one parcel")

    component_names = [item.name for item in (*spec.pushers, *spec.parcels)]
    if len(component_names) != len(set(component_names)):
        raise ValueError("Component names must be unique")

    cable_xml = ""
    equality_xml = ""
    cable_manifest = None
    if spec.cable is not None:
        if spec.cable_sites is None or len(set(spec.cable_sites)) != 2:
            raise ValueError("Cable requires two distinct pusher attachment sites")
        pushers_by_site = {f"{p.name}_attach": p for p in spec.pushers}
        if any(site not in pushers_by_site for site in spec.cable_sites):
            raise ValueError("Cable endpoints must reference existing pusher attachment sites")
        first, second = (pushers_by_site[site] for site in spec.cable_sites)
        cable_xml, equality_xml, cable_joints = spec.cable.xml(
            first.name,
            second.name,
            first.attachment_offset,
            second.attachment_offset,
            _attachment_world_position(first),
            _attachment_world_position(second),
        )
        cable_manifest = {
            "bodies": tuple(
                f"{spec.cable.name}_node_{index}" for index in range(spec.cable.nodes)
            ),
            "joints": cable_joints,
            "end_sites": (f"{spec.cable.name}_end_a", f"{spec.cable.name}_end_b"),
            "pusher_sites": spec.cable_sites,
        }

    map_spec = MAPS[spec.map]
    assets_xml = """
<asset>
  <material name="floor" rgba="0.14 0.18 0.22 1"/>
  <material name="wall" rgba="0.45 0.48 0.52 1"/>
  <material name="epuck" rgba="0.15 0.35 0.85 1"/>
  <material name="board" rgba="0.85 0.85 0.85 1"/>
  <material name="wheel" rgba="0.06 0.06 0.06 1"/>
  <material name="parcel" rgba="0.85 0.20 0.16 1"/>
  <material name="cable" rgba="0.70 0.65 0.20 1"/>
</asset>
"""
    xml = f"""
<mujoco model="epuck_scene">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit"/>
  <default>
    <joint damping="0.1"/>
    <geom solref="0.006 1" solimp="0.9 0.95 0.001" friction="0.8 0.1 0.001"/>
  </default>
  {assets_xml}
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    {map_spec['xml']}
    {''.join(pusher.xml() for pusher in spec.pushers)}
    {''.join(parcel.xml() for parcel in spec.parcels)}
    {cable_xml}
  </worldbody>
  <actuator>{''.join(pusher.actuators() for pusher in spec.pushers)}</actuator>
  <equality>{equality_xml}</equality>
</mujoco>
"""

    manifest = SceneManifest(
        pushers={
            pusher.name: {
                "body": pusher.name,
                "freejoint": f"{pusher.name}_free",
                "actuators": (
                    f"{pusher.name}_wheel_left_velocity",
                    f"{pusher.name}_wheel_right_velocity",
                ),
                "attach_site": f"{pusher.name}_attach",
                "collision_geoms": (
                    f"{pusher.name}_chassis",
                    f"{pusher.name}_board_front",
                    f"{pusher.name}_board_left",
                    f"{pusher.name}_board_right",
                ),
            }
            for pusher in spec.pushers
        },
        parcels={
            parcel.name: {
                "body": parcel.name,
                "freejoint": f"{parcel.name}_free",
                "collision_geom": f"{parcel.name}_geom",
            }
            for parcel in spec.parcels
        },
        cable=cable_manifest,
        goal=map_spec["goal"],
        robot_spawn=map_spec["robot_spawn"],
        parcel_spawn=map_spec["parcel_spawn"],
        camera=map_spec["camera"],
    )
    return xml, manifest
