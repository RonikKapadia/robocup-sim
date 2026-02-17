from .ROBOT import Robot
from .ARENA import Arena
from .BALL import Ball
import gymnasium
from gymnasium import spaces
import numpy as np
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import cv2


class RoboCupEnv(gymnasium.Env):
    def __init__(self):
        super(RoboCupEnv, self).__init__()
        self.deltaTime = 1/10
        self.max_steps = 400
        self.action_space = spaces.Box(
            low=-0.0, high=1.0, shape=(4,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(12,), dtype=np.float32
        )
        self.reset()

    def reset(self):
        self.steps = 0
        self.robot = Robot(x=0.75, y=1.21, size=0.11, rotation=0, mass=2.2, elasticity=0.5)
        self.arena = Arena()
        self.ball = Ball(x=1.215, y=0.91, size=0.0325, mass=0.07, drag=0.1, elasticity=0.5)

        action = [0.0, 0.0, 0.0, 0.0]
        obs = self.robot.step(action=action, deltaTime=self.deltaTime, arena=self.arena, ball=self.ball, robots=[self.robot])
        return obs


    def step(self, action):
        self.steps += 1
        
        obs = self.robot.step(action=action, deltaTime=self.deltaTime, arena=self.arena, ball=self.ball, robots=[self.robot])
        self.ball.step(arena=self.arena,robots=[self.robot], deltaTime=self.deltaTime)
        rewards, dones = self.arena.step(ball=self.ball, robots=[self.robot])
        reward = rewards[0]
        done = dones[0]

        if reward > 0:
            reward *= 1000
        if reward < 0:
            reward *= 100

        reward += 3.0*(1/((self.ball.x-self.robot.x)**2 + (self.ball.y-self.robot.y)**2)**(1/2)) - 5
        reward += 10.0*(1/((self.ball.x-self.arena.goal2.innerBox.x1)**2 + (self.ball.y-self.arena.goal2.innerBox.y1)**2)**(1/2)) - 10

        if self.steps >= self.max_steps:
            done = True
        
        return obs, reward, done, {}

    def render_init(self, scale=300, fps=10, show_debug=False):
        self.render_scale = scale
        self.render_fps = fps
        self.show_debug = show_debug

        # Init pygame
        pygame.init()
        self.render_win = pygame.display.set_mode((int(2.43*self.render_scale), int(1.82*self.render_scale)))
        pygame.display.set_caption("RoboCup Simulator")
        self.render_clock = pygame.time.Clock()

        # Create Image
        self.render_image = np.zeros((int(1.82*self.render_scale), int(2.43*self.render_scale), 3), np.uint8)

    def render(self):
        # Clock
        deltaTime = self.render_clock.tick(self.render_fps)
        deltaTime /= 1000

        # Draw image
        self.arena.draw(self.render_image, self.render_scale)
        self.ball.draw(self.render_image, self.render_scale)
        self.robot.draw(self.render_image, self.render_scale)
        if self.show_debug: self.robot.draw_sensors(self.render_image, self.render_scale)
        cv2.putText(self.render_image, 'FPS: ' + str(round(self.render_clock.get_fps())), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA, False)

        # Draw to pygame
        self.render_win.fill((0,0,0))
        img = cv2.rotate(self.render_image, cv2.ROTATE_90_CLOCKWISE)
        img = cv2.flip(img, 1)
        img = pygame.surfarray.make_surface(np.flip(img, 2)).convert()
        self.render_win.blit(img, (0,0))

        # Update image
        pygame.display.update() 

    def render_close(self):
        # quit pygame
        pygame.quit()

    



        

