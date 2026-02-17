import cv2 
import numpy as np
import math
from .EXTRA import Point, XLine, YLine, Box


class Ball:
    ''' 
    A Ball object is an tool the agent uses to obtain reward.
    
    Parameters:
        Type:   Name:       Unit:
        (float) x           m
        (float) y           m
        (float) size        m
        (float) mass        kg
        (float) elasticity  N/A
        (float) drag        N/A

    '''
    def __init__(self, x, y, size, mass=0.07, elasticity=0.9, drag=0.9):
        self.x = x
        self.y = y
        self.size = size
        self.velocity = 0
        self.rotation = 0
        self.mass = mass
        self.elasticity = elasticity
        self.drag = drag
    

    def step(self, arena, robots, deltaTime):
        ''' 
        Updates Ball position using inputted parameters.

        Parameters:
            Type:   Name:       Unit:
            (Arena) area        N/A
            (list)  robots      N/A
            (float) deltaTime   s
        '''

        self.x += math.sin(self.rotation) * self.velocity * deltaTime
        self.y -= math.cos(self.rotation) * self.velocity * deltaTime

        for robot in robots:
            self.collision(robot, arena)

        # linear drag
        if self.velocity > 0.001:
            self.velocity -= self.drag * deltaTime
            # self.velocity -= (pow(self.velocity, 2)*self.drag)*deltaTime
            # self.velocity -= ((pow(self.velocity, 2)*self.drag)+ 3 )*deltaTime
        else:
            self.velocity = 0

        
        
    def collision(self, robot, arena):
        '''
        Handles collision between self, and robots and arena.
        
        Parameters:
            Type:   Name:   Unit:
            (Robot) x       N/A
            (Arena) y       N/A
        
        '''
        dx = self.x - robot.x
        dy = self.y - robot.y
    
        dist = math.hypot(dx, dy)
        if dist < self.size + robot.size:
            rotation = math.atan2(dy, dx) + 0.5 * math.pi
            total_mass = self.mass + robot.mass
            
            (self.rotation, self.velocity) = self.addVectors((self.rotation, self.velocity*(self.mass-robot.mass)/total_mass), (rotation, 2*robot.velocity*robot.mass/total_mass))
            elasticity = self.elasticity * robot.elasticity
            self.velocity *= elasticity

            overlap = 0.5*(self.size + robot.size - dist)
            self.x += math.sin(rotation)*overlap
            self.y -= math.cos(rotation)*overlap
            robot.x -= math.sin(rotation)*overlap
            robot.y += math.cos(rotation)*overlap

        xLines = arena.goal1.xLines + arena.goal2.xLines + arena.boundary.xLines
        for xLine in xLines:
            distance = abs(xLine.x - self.x) 
            if distance < self.size:
                t = (self.size**2 - distance**2)**0.5
                t1 = self.y - t
                t2 = self.y + t
                if t1 < xLine.yRangeUp and t1 > xLine.yRangeDown or t2 < xLine.yRangeUp and t2 > xLine.yRangeDown:
                    if self.x > xLine.x:
                        self.x += abs(self.x-xLine.x-self.size)
                    if self.x < xLine.x:
                        self.x -= abs(self.x-xLine.x+self.size)
                    self.rotation = 2*np.pi - self.rotation

        yLines = arena.goal1.yLines + arena.goal2.yLines + arena.boundary.yLines
        for yLine in yLines:
            distance = abs(yLine.y - self.y)
            if distance < self.size:
                t = (self.size**2 - distance**2)**0.5
                t1 = self.x - t
                t2 = self.x + t
                if t1 < yLine.xRangeUp and t1 > yLine.xRangeDown or t2 < yLine.xRangeUp and t2 > yLine.xRangeDown:
                    if self.y > yLine.y:
                        self.y += abs(self.y-yLine.y-self.size)
                    if self.y < yLine.y:
                        self.y -= abs(self.y-yLine.y+self.size)
                    self.rotation = np.pi - self.rotation


    def addVectors(self, v1, v2):
        ''' 
        Returns the sum of two polar vectors.

        Parameters:
            Type:   Name:  Unit:
            (tuple) v1     (rad, m)
            (tuple) v2     (rad, m)

        Returns:
            Type:   Name:  Unit:
            (tuple) N/A    (rad, m)
        
        '''
        rotation1, length1 = v1
        rotation2, length2 = v2

        x  = math.sin(rotation1) * length1 + math.sin(rotation2) * length2
        y  = math.cos(rotation1) * length1 + math.cos(rotation2) * length2
        
        rotation  = 0.5 * math.pi - math.atan2(y, x)
        length = math.hypot(x, y)

        return (rotation, length)


    def draw(self, image, scale=100):
        '''
        Draws object to image.

        Parameters:
            Type:           Name:   Unit:
            (np.ndarray)    image   N/A
            (float)         scale   N/A

        '''
        color = (255,255,255)
        cv2.circle(image, (int(scale*self.x), int(scale*self.y)), int(self.size*scale), color, -1, cv2.LINE_AA)
        color = (0,0,0)
        cv2.circle(image, (int(scale*self.x), int(scale*self.y)), int(self.size*scale/3), color, -1, cv2.LINE_AA)

        size = self.size/4
        subdivisions = 5
        subSection = 2*np.pi/subdivisions
        for sub in range(subdivisions):
            xOff = math.cos(subSection*sub)*self.size
            yOff = math.sin(subSection*sub)*self.size
            cv2.circle(image, (int(scale*(self.x+xOff)), int(scale*(self.y+yOff))), int(size*scale), color, -1, cv2.LINE_AA)















