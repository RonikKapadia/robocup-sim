from stable_baselines3 import PPO
from tqdm import tqdm
from source.ENVIRONMENT import RoboCupEnv


if __name__ == '__main__':

    # creating gym environment
    env = RoboCupEnv()

    # test the agent in the environment with random sampling
    obs = env.reset()
    env.render_init(show_debug=True)
    for s in tqdm(range(env.max_steps)):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()  
        if done:
            break
    env.render_close()

    # train a model on the environment using stable_baselines3
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=100_000)

    # save the trained model
    model.save('models/PPO_1')

    # load the trained model
    model = PPO.load('models/PPO_1', env)

    # test the agent in the environment with trained model
    obs = env.reset()
    env.render_init()
    for s in tqdm(range(env.max_steps)):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()  
        if done:
            break
    env.render_close()