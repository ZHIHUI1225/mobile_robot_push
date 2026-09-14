from epuck_mujoco import Cable, EpuckPusher, SceneSpec, build_scene

left = EpuckPusher("left", pos=(-0.175, 0.0, 0.022))
right = EpuckPusher("right", pos=(0.175, 0.0, 0.022))
xml, manifest = build_scene(SceneSpec(map="two_room_corridor", pushers=(left, right), cable=Cable(), cable_sites=("left_attach", "right_attach")))
print(f"MJCF characters: {len(xml)}")
print(f"Cable endpoints: {manifest.cable['pusher_sites']}")
