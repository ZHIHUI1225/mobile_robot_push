"""Static layouts. Robots and parcels are deliberately not embedded here."""

MAPS = {
    "single_room": {
        "goal": (0.95, 0.0),
        "robot_spawn": (-0.62, -0.42, -0.30, 0.30),
        "parcel_spawn": (-0.15, 0.28, -0.28, 0.28),
        "camera": (0.05, 0.0, 2.5),
        "xml": """
<geom name="floor" type="plane" size="1.4 1.1 0.1" material="floor"/>
<geom name="west_wall" type="box" pos="-0.8 0 0.1" size="0.025 0.65 0.1" material="wall"/>
<geom name="north_wall" type="box" pos="0 0.65 0.1" size="0.8 0.025 0.1" material="wall"/>
<geom name="south_wall" type="box" pos="0 -0.65 0.1" size="0.8 0.025 0.1" material="wall"/>
<geom name="east_wall_north" type="box" pos="0.8 0.37 0.1" size="0.025 0.28 0.1" material="wall"/>
<geom name="east_wall_south" type="box" pos="0.8 -0.37 0.1" size="0.025 0.28 0.1" material="wall"/>
<site name="goal" pos="0.95 0 0.004" type="cylinder" size="0.08 0.002" rgba="0.1 0.9 0.2 0.45"/>
""",
    },
    "two_room_corridor": {
        "goal": (1.20, -0.30),
        "robot_spawn": (0.90, 1.18, -0.64, -0.52),
        "parcel_spawn": (0.92, 1.15, -0.46, -0.26),
        "camera": (0.0, 0.0, 3.1),
        "xml": """
<geom name="floor" type="plane" size="1.5 1.0 0.1" material="floor" condim="3" friction="0.8 0.005 0.0001" contype="5" conaffinity="11"/>

<geom name="wall_north" type="box" pos="0 0.725 0.1" size="1.3 0.025 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="wall_south" type="box" pos="0 -0.725 0.1" size="1.3 0.025 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="wall_east" type="box" pos="1.325 0 0.1" size="0.025 0.7 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="wall_west" type="box" pos="-1.325 0 0.1" size="0.025 0.7 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>

<!-- Upper-left room. Gaps between the split east and south walls are doors. -->
<geom name="room_right_upper" type="box" pos="-0.3 0.6 0.1" size="0.025 0.07 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room_right_lower" type="box" pos="-0.3 0.10 0.1" size="0.025 0.19 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room_bottom_left" type="box" pos="-1.085 -0.1 0.1" size="0.16 0.025 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room_bottom_right" type="box" pos="-0.515 -0.1 0.1" size="0.21 0.025 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="obstacle_1" type="box" pos="-0.90 0.35 0.1" size="0.06 0.06 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="obstacle_2" type="box" pos="-0.55 0.1 0.1" size="0.06 0.06 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>

<!-- Lower-right room. Gaps between the split west and north walls are doors. -->
<geom name="room2_left_upper" type="box" pos="0.2 -0.12 0.1" size="0.025 0.29 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room2_left_lower" type="box" pos="0.2 -0.63 0.1" size="0.025 0.04 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room2_right" type="box" pos="0.8 -0.25 0.1" size="0.025 0.45 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room2_top_left" type="box" pos="0.33 0.2 0.1" size="0.13 0.025 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="room2_top_right" type="box" pos="0.72 0.2 0.1" size="0.08 0.025 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="obstacle_3" type="box" pos="0.4 -0.05 0.1" size="0.06 0.06 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>
<geom name="obstacle_4" type="box" pos="0.6 -0.5 0.1" size="0.06 0.06 0.1" material="wall" condim="3" friction="0.8 0.1 0.001" contype="3" conaffinity="3"/>

<site name="goal" pos="1.2 -0.3 0.004" type="cylinder" size="0.08 0.002" rgba="0.1 0.9 0.2 0.45"/>
""",
    },
}
