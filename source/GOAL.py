import cv2 
import numpy as np
import math
from .EXTRA import Point, XLine, YLine, Box


class Goal:
    '''
    A Goal object holds the dimensions of the goals.

    Parameters:
        Type:   Name:       Unit:
        (float) x           m
        (float) xOpen       m
        (float) y1          m
        (float) y2          m
        (float) thickness   m
        (float) reward      N/A
    
    '''
    def __init__(self, x, xOpen, y1, y2, thickness, reward):
        t = thickness/2
        
        self.xLines = [XLine(x + t, y1, y2), XLine(x - t, y1, y2), XLine(xOpen, y1 + t, y1 - t), XLine(xOpen, y2 + t, y2 - t)]
        self.yLines = [YLine(y1 + t, x, xOpen), YLine(y1 - t, x, xOpen), YLine(y2 + t, x, xOpen), YLine(y2 - t, x, xOpen)]
        
        self.x = x
        self.xOpen = xOpen
        self.y1 = y1
        self.y2 = y2
        self.thickness = thickness
        self.reward = reward


    def step(self, ball):
        ''' 
        Get reward if ball goes in goal.

        Parameters:
            Type:   Name:   Unit:
            (Ball)  ball    N/A

        Returns:
            Type:   Name:   Unit:
            (float) reward  N/A
        
        '''
        reward = 0
        
        yUp = self.y1
        yDown = self.y2
        if self.y1 > self.y2:
            yUp = self.y2
            yDown = self.y1

        xUp = self.x
        xDown = self.xOpen
        if self.x > self.xOpen:
            xUp = self.xOpen
            xDown = self.x
        
        t = self.thickness/2 
        self.outerBox = Box(xUp+t, xDown-t, yUp+t, yDown-t)
        self.innerBox = Box(xUp-t, xDown+t, yUp-t, yDown+t)

        if self.innerBox.contains(ball.x, ball.y) == True:
            reward = self.reward
            
        return reward


    def draw(self, image, scale=100, color=(255,255,255)):
        '''
        Draws object to image.

        Parameters:
            Type:           Name:   Unit:
            (np.ndarray)    image   N/A
            (float)         scale   N/A
            (tuple)         color   (r,g,b)

        '''

        if self.reward < 0:
            color = (255,0,0)
        if self.reward > 0:
            color = (0,255,255)

        cv2.line(image, (int(self.x*scale),int(self.y1*scale)), (int(self.x*scale),int(self.y2*scale)), color, int(self.thickness*scale), cv2.LINE_AA)
        cv2.line(image, (int(self.x*scale),int(self.y1*scale)), (int(self.xOpen*scale),int(self.y1*scale)), color, int(self.thickness*scale), cv2.LINE_AA)
        cv2.line(image, (int(self.x*scale),int(self.y2*scale)), (int(self.xOpen*scale),int(self.y2*scale)), color, int(self.thickness*scale), cv2.LINE_AA)
        




