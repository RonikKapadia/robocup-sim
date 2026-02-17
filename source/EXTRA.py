class Point:
    ''' 
    A Point object holds an x and y location.

    Parameters:
        Type:   Name:  Unit:
        (float) x      m
        (float) y      m

    '''
    def __init__(self, x, y):
        self.x = x
        self.y = y


class XLine:
    ''' 
    An XLine object holds an x-axis location and its range along the y-axis (used primarily for collision).

    Parameters:
        Type:   Name:  Unit:  
        (float) x      m      
        (float) y1     m      
        (float) y2     m   

    '''
    def __init__(self, x, y1, y2):
        self.x = x
        self.yRangeUp = max([y1,y2])
        self.yRangeDown = min([y1, y2])


class YLine:
    ''' 
    An XLine object holds an y-axis location and its range along the x-axis (used primarily for collision).

    Parameters:
        Type:   Name:  Unit:  
        (float) y      m      
        (float) x1     m      
        (float) x2     m    

    '''
    def __init__(self, y, x1, x2):
        self.y = y
        self.xRangeUp = max([x1,x2])
        self.xRangeDown = min([x1, x2])


class Box:
    ''' 
    A Box object which holds x and y locations for the top left and bottom right of a rectangle.

    Parameters:
        Type:   Name:  Unit:
        (float) x1     m
        (float) x2     m
        (float) y1     m
        (float) y2     m

    '''
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

    def contains(self, x, y):
        ''' 
        Returns wheather the provided x and y location lie within the Box object.

        Parameters:
            Type:   Name:  Unit:
            (float) x      m
            (float) x      m

        Returns:
            Type:   Name:  Unit:
            (bool)  N/A    N/A
        
        '''
        xUp = max(self.x1, self.x2)
        xDown = min(self.x1, self.x2)
        yUp = max(self.y1, self.y2)
        yDown = min(self.y1, self.y2)

        if x < xUp and x > xDown and y < yUp and y > yDown:
            return True
        else:
            return False
