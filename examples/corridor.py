import gymnasium as gym
import epuck_mujoco
env = gym.make("EpuckCorridorPush-v0", render_mode="rgb_array")
observation, info = env.reset(seed=0)
observation, reward, terminated, truncated, info = env.step([0.2, 0.1])
print(observation.shape, reward, info)
env.close()
