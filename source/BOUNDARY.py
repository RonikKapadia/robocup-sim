import cv2 
import numpy as np
import math
from .EXTRA import Point, XLine, YLine, Box


class Boundary:
    '''
    A Boundary object holds the maximum boundary dimensions.

    Parameters:
        Type:   Name:   Unit:
        (float) x1      m
        (float) x2      m
        (float) y1      m
        (float) y2      m

    '''
    def __init__(self, x1, x2, y1, y2):
        self.xLines = [XLine(x1, y1, y2), XLine(x2, y1, y2)]
        self.yLines = [YLine(y1, x1, x2), YLine(y2, x1, x2)]

        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2


    def draw(self, image, scale=100, color=(80,155,50)):
        '''
        Draws object to image.

        Parameters:
            Type:           Name:   Unit:
            (np.ndarray)    image   N/A
            (float)         scale   N/A
            (tuple)         color   (r,g,b)

        '''
        cv2.rectangle(image, (int(self.x1*scale),int(self.y1*scale)), (int(self.x2*scale),int(self.y2*scale)), color, -1, cv2.LINE_AA)