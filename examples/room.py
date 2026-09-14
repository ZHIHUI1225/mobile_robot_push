import gymnasium as gym
import epuck_mujoco
env = gym.make("EpuckRoomPush-v0", render_mode="rgb_array")
observation, info = env.reset(seed=0)
for _ in range(10):
    observation, reward, terminated, truncated, info = env.step([0.5, 0.0])
    if terminated or truncated:
        break
frame = env.render()
print(observation.shape, frame.shape, info)
env.close()
