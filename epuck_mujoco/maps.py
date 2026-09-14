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
        "goal": (1.30, 0.0),
        "robot_spawn": (-0.44, -0.28, -0.12, 0.12),
        "parcel_spawn": (-0.05, 0.18, -0.10, 0.10),
        "camera": (0.0, 0.0, 4.0),
        "xml": """
<geom name="floor" type="plane" size="2.1 1.3 0.1" material="floor"/>
<geom name="outer_north" type="box" pos="0 1.0 0.1" size="1.8 0.025 0.1" material="wall"/>
<geom name="outer_south" type="box" pos="0 -1.0 0.1" size="1.8 0.025 0.1" material="wall"/>
<geom name="outer_west" type="box" pos="-1.8 0 0.1" size="0.025 1.0 0.1" material="wall"/>
<geom name="outer_east" type="box" pos="1.8 0 0.1" size="0.025 1.0 0.1" material="wall"/>
<geom name="room1_east_north" type="box" pos="-0.5 0.625 0.1" size="0.025 0.375 0.1" material="wall"/>
<geom name="room1_east_south" type="box" pos="-0.5 -0.625 0.1" size="0.025 0.375 0.1" material="wall"/>
<geom name="room2_west_north" type="box" pos="0.5 0.625 0.1" size="0.025 0.375 0.1" material="wall"/>
<geom name="room2_west_south" type="box" pos="0.5 -0.625 0.1" size="0.025 0.375 0.1" material="wall"/>
<geom name="corridor_north" type="box" pos="0 0.25 0.1" size="0.5 0.025 0.1" material="wall"/>
<geom name="corridor_south" type="box" pos="0 -0.25 0.1" size="0.5 0.025 0.1" material="wall"/>
<site name="goal" pos="1.30 0 0.004" type="cylinder" size="0.08 0.002" rgba="0.1 0.9 0.2 0.45"/>
""",
    },
}
