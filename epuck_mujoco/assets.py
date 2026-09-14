"""Reusable MuJoCo components for e-puck manipulation scenes."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot


@dataclass(frozen=True)
class EpuckPusher:
    """Planar differential-drive e-puck with a front pushing board."""

    name: str
    pos: tuple[float, float, float] = (0.0, 0.0, 0.022)
    yaw: float = 0.0
    role: str = "room"
    wheel_radius: float = 0.0205
    half_track: float = 0.026
    max_wheel_speed: float = 6.34
    attachment_offset: tuple[float, float, float] = (0.0, 0.0, 0.0335)

    def xml(self) -> str:
        name = self.name
        x, y, z = self.pos
        ax, ay, az = self.attachment_offset
        return f"""
<body name="{name}" pos="{x} {y} {z}" euler="0 0 {self.yaw}">
  <freejoint name="{name}_free"/>
  <inertial pos="0 0 -0.01" mass="0.15" diaginertia="0.00009 0.00009 0.00014"/>
  <geom name="{name}_chassis" type="cylinder" size="0.035 0.025" pos="0 0 0.005"
        material="epuck" mass="0.15" condim="3" friction="1.2 0.1 0.001"/>
  <body name="{name}_board" pos="0.035 0 0.0025">
    <geom name="{name}_board_front" type="box" size="0.003 0.035 0.0215"
          material="board" mass="0.01" condim="3" friction="1.8 0.2 0.001"/>
    <geom name="{name}_board_left" type="box" size="0.01 0.0015 0.0215"
          pos="0.013 0.0365 0" material="board" mass="0.002"/>
    <geom name="{name}_board_right" type="box" size="0.01 0.0015 0.0215"
          pos="0.013 -0.0365 0" material="board" mass="0.002"/>
  </body>
  <site name="{name}_attach" pos="{ax} {ay} {az}" size="0.004" rgba="0.2 0.8 1 1"/>
  <body name="{name}_wheel_left" pos="0 0.026 0">
    <joint name="{name}_wheel_left_joint" type="hinge" axis="0 1 0" damping="0.001" armature="0.0001"/>
    <geom name="{name}_wheel_left_geom" type="cylinder" size="0.0205 0.005"
          euler="1.5708 0 0" material="wheel" friction="2 0.005 0.0001"/>
  </body>
  <body name="{name}_wheel_right" pos="0 -0.026 0">
    <joint name="{name}_wheel_right_joint" type="hinge" axis="0 1 0" damping="0.001" armature="0.0001"/>
    <geom name="{name}_wheel_right_geom" type="cylinder" size="0.0205 0.005"
          euler="1.5708 0 0" material="wheel" friction="2 0.005 0.0001"/>
  </body>
</body>
"""

    def actuators(self) -> str:
        name = self.name
        return f"""
<velocity name="{name}_wheel_left_velocity" joint="{name}_wheel_left_joint"
          kv="1" ctrllimited="true" ctrlrange="-6.34 6.34"/>
<velocity name="{name}_wheel_right_velocity" joint="{name}_wheel_right_joint"
          kv="1" ctrllimited="true" ctrlrange="-6.34 6.34"/>
"""


@dataclass(frozen=True)
class BoxParcel:
    """A parameterized box parcel; size values are MuJoCo half extents."""

    name: str
    pos: tuple[float, float, float] = (0.0, 0.0, 0.04)
    size: tuple[float, float, float] = (0.025, 0.025, 0.04)
    mass: float = 0.05

    def xml(self) -> str:
        x, y, z = self.pos
        sx, sy, sz = self.size
        return f"""
<body name="{self.name}" pos="{x} {y} {z}">
  <freejoint name="{self.name}_free"/>
  <geom name="{self.name}_geom" type="box" size="{sx} {sy} {sz}"
        material="parcel" mass="{self.mass}" condim="3" friction="2.0 0.5 0.005"/>
</body>
"""


@dataclass(frozen=True)
class Cable:
    """Planar articulated cable attached to two pusher sites."""

    name: str = "cable"
    nodes: int = 15
    link_length: float = 0.025
    radius: float = 0.012

    @property
    def length(self) -> float:
        return (self.nodes - 1) * self.link_length

    def xml(self, body_a, body_b, anchor_a, anchor_b, start, end):
        if self.nodes < 2:
            raise ValueError("Cable requires at least two nodes")
        distance = hypot(end[0] - start[0], end[1] - start[1])
        if abs(distance - self.length) > 0.005:
            raise ValueError(
                f"Cable length {self.length:.3f} m does not match attachment distance {distance:.3f} m"
            )
        yaw = atan2(end[1] - start[1], end[0] - start[0])

        child = f'<site name="{self.name}_end_b" pos="0 0 0" size="0.003"/>'
        internal_joints = []
        for index in reversed(range(1, self.nodes)):
            joint = f"{self.name}_joint_{index}"
            internal_joints.append(joint)
            geom = ""
            if index < self.nodes - 1:
                geom = (
                    f'<geom name="{self.name}_link_{index}" type="capsule" '
                    f'fromto="0 0 0 {self.link_length} 0 0" size="{self.radius}" '
                    'material="cable" mass="0.002"/>'
                )
            child = (
                f'<body name="{self.name}_node_{index}" pos="{self.link_length} 0 0">'
                f'<joint name="{joint}" type="hinge" axis="0 0 1" damping="0.005" '
                f'stiffness="0.00003"/><inertial pos="0 0 0" mass="0.001" '
                f'diaginertia="0.0000001 0.0000001 0.0000001"/>{geom}{child}</body>'
            )

        root_joints = (f"{self.name}_x", f"{self.name}_y", f"{self.name}_yaw")
        cable_xml = f"""
<body name="{self.name}_node_0" pos="{start[0]} {start[1]} {start[2]}" euler="0 0 {yaw}">
  <joint name="{root_joints[0]}" type="slide" axis="1 0 0"/>
  <joint name="{root_joints[1]}" type="slide" axis="0 1 0"/>
  <joint name="{root_joints[2]}" type="hinge" axis="0 0 1" damping="0.001"/>
  <site name="{self.name}_end_a" pos="0 0 0" size="0.003"/>
  <geom name="{self.name}_link_0" type="capsule" fromto="0 0 0 {self.link_length} 0 0"
        size="{self.radius}" material="cable" mass="0.002"/>
  {child}
</body>
"""
        equality_xml = f"""
<connect name="{self.name}_attachment_a" body1="{body_a}" body2="{self.name}_node_0"
         anchor="{anchor_a[0]} {anchor_a[1]} {anchor_a[2]}"/>
<connect name="{self.name}_attachment_b" body1="{body_b}" body2="{self.name}_node_{self.nodes - 1}"
         anchor="{anchor_b[0]} {anchor_b[1]} {anchor_b[2]}"/>
"""
        joints = root_joints + tuple(reversed(internal_joints))
        return cable_xml, equality_xml, joints
