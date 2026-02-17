import cv2 
import numpy as np
import math
from .EXTRA import Point, XLine, YLine, Box


class Field:
    '''
    A Field object holds the in-bounds dimension where the robots reside.

    Parameters:
        Type:   Name:       Unit:
        (float) x1          m
        (float) x2          m
        (float) y1          m
        (float) y2          m
        (float) thickness   m
        (float) reward      N/A
    
    '''
    def __init__(self, x1, x2, y1, y2, thickness, reward):
        self.xUp = max(x1, x2)
        self.xDown = min(x1,x2)
        self.yUp = max(y1, y2)
        self.yDown = min(y1, y2)
        self.thickness = thickness
        self.reward = reward
        ...
        t = thickness/2
        self.outerBox = Box(self.xUp+t, self.xDown-t, self.yUp+t, self.yDown-t)
        self.innerBox = Box(self.xUp-t, self.xDown+t, self.yUp-t, self.yDown+t)

    def step(self, robot):
        ''' 
        Get reward if robot goes out of the field.

        Parameters:
            Type:   Name:   Unit:
            (Robot) robot   N/A

        Returns:
            Type:   Name:   Unit:
            (float) reward  N/S
        
        '''
        reward = 0

        if self.outerBox.contains(robot.x, robot.y) == False:
            reward = self.reward

        return reward
        ...

    def draw(self, image, scale=100, color=(255,255,255)):
        '''
        Draws object to image.

        Parameters:
            Type:           Name:   Unit:
            (np.ndarray)    image   N/A
            (float)         scale   N/A
            (tuple)         color   (r,g,b)

        '''
        cv2.circle(image, (int(scale*0.75), int(scale*1.21)), int(0.01*scale), (0,0,0), -1, cv2.LINE_AA)
        cv2.circle(image, (int(scale*0.75), int(scale*0.61)), int(0.01*scale), (0,0,0), -1, cv2.LINE_AA)
        cv2.circle(image, (int(scale*(2.43-0.75)), int(scale*1.21)), int(0.01*scale), (0,0,0), -1, cv2.LINE_AA)
        cv2.circle(image, (int(scale*(2.43-0.75)), int(scale*0.61)), int(0.01*scale), (0,0,0), -1, cv2.LINE_AA)

        cv2.rectangle(image, (int(0.30*scale), int(0.56*scale)), (int(0.6*scale), int(1.26*scale)), (160,160,160), int(scale*0.01), cv2.LINE_AA)
        cv2.rectangle(image, (int((2.43-0.30)*scale), int(0.56*scale)), (int((2.43-0.6)*scale), int(1.26*scale)), (160,160,160), int(scale*0.01), cv2.LINE_AA)

        cv2.circle(image, (int(scale*(self.xUp+self.xDown)/2), int(scale*(self.yUp+self.yDown)/2)), int(0.3*scale), (0,0,0), int(0.001*scale), cv2.LINE_AA)
        cv2.rectangle(image, (int(self.xDown*scale),int(self.yDown*scale)), (int(self.xUp*scale),int(self.yUp*scale)), color, int(self.thickness*scale), cv2.LINE_AA)

