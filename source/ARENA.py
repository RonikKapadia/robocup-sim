from .GOAL import Goal
from .FIELD import Field
from .BOUNDARY import Boundary


class Arena:
    '''An Arena object holds all static objects the robot or ball may interact with.'''
    def __init__(self):
        self.goal1 = Goal(0.226, 0.30, 1.21, 0.61, thickness=0.02, reward=-1)
        self.goal2 = Goal(2.204, 2.13, 1.21, 0.61, thickness=0.02, reward=+1)
        self.field = Field(0.30, 2.13, 0.30, 1.52, thickness=0.02, reward=-1)
        self.boundary = Boundary(0, 2.43, 0, 1.82)


    def step(self, ball, robots):
        '''
        Gets reward and environment state for each robot. 

        Parameters:
            Type:   Name:           Unit:
            (Ball)  ball            N/A
            (list)  robots          N/A

        ''' 
        rewards = []
        dones = []
        for robot in robots:
            reward = 0 
            reward += self.goal1.step(ball)
            reward += self.goal2.step(ball)
            reward += self.field.step(robot)
            
            done = False
            if reward != 0:
                done = True
            
            rewards.append(reward)
            dones.append(done)

        return rewards, dones


    def draw(self, image, scale=100):
        '''
        Draws object to image

        Parameters:
            Type:           Name:   Unit:
            (np.ndarray)    image   N/A
            (float)         scale   N/A

        '''
        self.boundary.draw(image, scale)
        self.field.draw(image, scale)
        self.goal1.draw(image, scale)
        self.goal2.draw(image, scale)