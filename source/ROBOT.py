import cv2 
import numpy as np
import math
from .EXTRA import Point, XLine, YLine, Box


class Robot:
    '''
    A Robot object is the agent that traverse the environment.

    Parameters:
        Type:   Name:       Unit:
        (float) x           m
        (float) y           m
        (float) size        m
        (float) rotation    rad
        (Arena) mass        kg
        (Ball)  elasticity  N/A

    '''
    def __init__(self, x, y, size, rotation, mass=2.2, elasticity=0.9, max_velocity=0.5, max_angular_velocity=1.8):
        # Robot properties
        self.x = x
        self.y = y
        self.size = size
        self.rotation = rotation
        self.mass = mass
        self.elasticity = elasticity
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity

        self.velocity = 0
        self.direction = 0
        self.vx = 0
        self.vy = 0
        
        # Sensor info
        self.compassPoints = []
        self.compassValues = []

        self.infaredPoints = []
        self.infaredValues = []
        self.infaredHitPoints = []

        self.ultrasonicPoints = []
        self.ultrasonicHitPoints = []

        self.lineVal = 0

    def step(self, action, deltaTime, arena, ball, robots):
        '''
        Updates Robot properties using inputted parameters.

        Parameters:
            Type:   Name:           Unit:
            (list)  action          N/A
            (float) deltaTime       s
            (Arena) arena           N/A
            (Ball)  ball            N/A
            (list)  robots          N/A

        Returns:
            Type:   Name:           Unit:
            (list)  observation     N/A
        ''' 
        dx, dy, v1, a1 = action

        direction = np.angle(complex(dx-0.5,dy-0.5))
        self.direction = direction

        self.vx = np.cos(direction) * v1 * self.max_velocity
        self.vy = np.sin(direction) * v1 * self.max_velocity

        self.velocity = v1
        self.x += self.vx * deltaTime
        self.y += self.vy * deltaTime
        
        self.rotation += 2*(a1-0.5) * deltaTime * self.max_angular_velocity

        self.collision(arena, robots)

        observation = self.compass() + self.infared(arena, ball, robots) + self.ultrasonic(arena, robots) + self.line(arena)
        return observation
    
    def step_human(self, velocity, direction, angularVelocity, deltaTime, arena, ball, robots):
        '''
        Updates Robot properties using inputted parameters.

        Parameters:
            Type:   Name:           Unit:
            (float) velocity        m/s
            (float) direction       rad
            (float) angularVelocity rad/s
            (float) deltaTime       s
            (Arena) arena           N/A
            (Ball)  ball            N/A
            (list)  robots          N/A

        Returns:
            Type:   Name:           Unit:
            (list)  observation     N/A
        
        ''' 
        self.vx = np.cos(direction) * velocity
        self.vy = np.sin(direction) * velocity

        self.velocity = velocity
        self.x += self.vx * deltaTime
        self.y += self.vy * deltaTime
        self.direction = direction

        self.rotation += angularVelocity * deltaTime

        self.collision(arena, robots)

        observation = self.compass() + self.infared(arena, ball, robots) + self.ultrasonic(arena, robots) + self.line(arena)
        return observation
        ...

    # Collision
    def collision(self, arena, robots):
        ''' 
        Handles Robot object collision with Arena object

        Parameters:
            Type:   Name:   Unit:
            (Arena) arena   N/A

        '''
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
                    self.vy *= 0

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
                    self.vx *= 0

        for robot in robots:
            if robot != self:
                dx = self.x - robot.x
                dy = self.y - robot.y

                dist = math.hypot(dx, dy)
                if dist < self.size + robot.size:
                    rotation = math.atan2(dy, dx) + 0.5 * math.pi

                    overlap = 0.5*(self.size + robot.size - dist)
                    self.x += math.sin(rotation)*overlap
                    self.y -= math.cos(rotation)*overlap
                    robot.x -= math.sin(rotation)*overlap
                    robot.y += math.cos(rotation)*overlap


    
    #   Sensors
    def compass(self):
        '''Compass Sensor - Detects the direction the robot is facing.'''
        observation = []

        self.compassPoints = []
        self.compassValues = []
        
        subdivisions = 3
        rotation = self.rotation%(2*np.pi)
        subSection = (2*np.pi)/subdivisions

        for sub in range(subdivisions):
            subRotation = subSection * sub

            deltaRotation = min((subRotation-rotation)%(2*np.pi),(rotation-subRotation)%(2*np.pi))

            obs = 0
            if deltaRotation < subSection:
                obs = subSection-deltaRotation
            obs = obs/(2*np.pi/3)
            
            self.compassPoints.append(Point(self.x+(obs*self.size*math.cos(subRotation)), self.y+(obs*self.size*math.sin(subRotation)) ))
            self.compassValues.append(obs)
            observation.append(obs)

        return observation


    def infared(self, arena, ball, robots):
        ''' 
        Infared Sensor - Detects the balls relative direction to the robots position.

        Parameters:
            Type:   Name:   Unit:
            (Arena) arena   N/A
            (Ball)  ball    N/A
            (list)  robots  N/A
        
        Returns:
            Type:   Name:       Unit:
            (list)  observation N/A
        
        '''
        observation = []

        self.infaredPoints = []
        self.infaredValues = []
        self.infaredHitPoints = []

        self.infaredPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi/4), self.y + self.size*np.sin(self.rotation+np.pi/4)))
        self.infaredPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi/2+np.pi/4), self.y + self.size*np.sin(self.rotation+np.pi/2+np.pi/4)))
        self.infaredPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi+np.pi/4), self.y + self.size*np.sin(self.rotation+np.pi+np.pi/4)))
        self.infaredPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi*1.5+np.pi/4), self.y + self.size*np.sin(self.rotation+np.pi*1.5+np.pi/4)))

        for infaredPoint in self.infaredPoints:
            distanceInfaredToBall = ((infaredPoint.x-ball.x)**2 + (infaredPoint.y-ball.y)**2)**(1/2)
            distanceRobotToBall = ((self.x-ball.x)**2 + (self.y-ball.y)**2)**(1/2)

            if distanceInfaredToBall < distanceRobotToBall:

                tx = []
                ty = []
                tr = []

                xLines = arena.goal1.xLines + arena.goal2.xLines + arena.boundary.xLines
                for xLine in xLines:
                    t = 0

                    t = (xLine.x - infaredPoint.x) 
                    t_ = (ball.x - infaredPoint.x)
                    if t_ == 0:
                        t_ = 0.001
                    t /= t_

                    if t < 0:
                        t = 1000
                    
                    infaredYHit = (1-t)*infaredPoint.y + ball.y*t
                    if infaredYHit < xLine.yRangeDown or infaredYHit > xLine.yRangeUp:
                        t = 1000

                    tx.append(t)

                yLines = arena.goal1.yLines + arena.goal2.yLines + arena.boundary.yLines
                for yLine in yLines:
                    t = 0

                    t = (yLine.y - infaredPoint.y) 
                    t_ = (self.y - infaredPoint.y)
                    if t_ == 0:
                        t_ = 0.001
                    t /= t_

                    if t < 0:
                        t = 1000
                    
                    infaredXHit = (1-t)*infaredPoint.x + ball.x*t
                    if infaredXHit < yLine.xRangeDown or infaredXHit > yLine.xRangeUp:
                        t = 1000

                    ty.append(t)

                for robot in robots:
                    if robot != self:
                        rx = robot.x
                        ry = robot.y
                        rr = robot.size

                        x0 = infaredPoint.x
                        x1 = ball.x
                        y0 = infaredPoint.y
                        y1 = ball.y

                        a = (x1 - x0)**2 + (y1-y0)**2
                        b = 2*(x1-x0)*(x0-rx) + 2*(y1-y0)*(y0-ry)
                        c = (x0-rx)**2 + (y0-ry)**2 - rr**2
                        
                        bac = b**2 - 4*a*c 
                        if bac < 0:
                            t = 1000
                        else:
                            t1 = (-b + math.sqrt(bac) ) / (2*a)
                            t2 = (-b - math.sqrt(bac) ) / (2*a)
                            t = min(t1,t2)
                            if t < 0:
                                t = 1000
                        tr.append(t)

                tList = ty+tx+tr
                t = min(tList)

                infaredXHit = (1-t)*infaredPoint.x + ball.x*t
                infaredYHit = (1-t)*infaredPoint.y + ball.y*t

                if t > 1:
                    observation.append(1.0)
                    self.infaredValues.append(1.0)
                    infaredXHit = ball.x
                    infaredYHit = ball.y
                else:
                    observation.append(0.0)
                    self.infaredValues.append(0.0)

                self.infaredHitPoints.append(Point(infaredXHit, infaredYHit))

                
            else:
                self.infaredHitPoints.append(infaredPoint)

                observation.append(0.0)
                self.infaredValues.append(0.0)

        return observation
    

    def ultrasonic(self, arena, robots):
        '''
        Ultrasonic Sensor - Detects the distance the robot is from surrounding arena object.

        Parameters:
            Type:   Name:   Unit:
            (Arena) arena   N/A
            (list)  robots  N/A
        
        Returns:
            Type:   Name:       Unit:
            (list)  observation N/A

        '''
        observation = []

        self.ultrasonicPoints = []
        self.ultrasonicHitPoints = []

        self.ultrasonicPoints.append(Point(self.x + self.size*np.cos(self.rotation), self.y + self.size*np.sin(self.rotation)))
        self.ultrasonicPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi/2), self.y + self.size*np.sin(self.rotation+np.pi/2)))
        self.ultrasonicPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi), self.y + self.size*np.sin(self.rotation+np.pi)))
        self.ultrasonicPoints.append(Point(self.x + self.size*np.cos(self.rotation+np.pi*1.5), self.y + self.size*np.sin(self.rotation+np.pi*1.5)))

        for ultrasonicPoint in self.ultrasonicPoints:

            tx = []
            ty = []
            tr = []

            xLines = arena.goal1.xLines + arena.goal2.xLines + arena.boundary.xLines
            for xLine in xLines:
                t = 0

                t = (xLine.x - self.x) 
                t_ = (ultrasonicPoint.x - self.x)
                if t_ == 0:
                    t_ = 0.001
                t /= t_

                if t < 0:
                    t = 1000
                
                ultrasonicYHit = (1-t)*self.y + ultrasonicPoint.y*t
                if ultrasonicYHit < xLine.yRangeDown or ultrasonicYHit > xLine.yRangeUp:
                    t = 1000

                tx.append(t)

            yLines = arena.goal1.yLines + arena.goal2.yLines + arena.boundary.yLines
            for yLine in yLines:
                t = 0

                t = (yLine.y - self.y) 
                t_ = (ultrasonicPoint.y - self.y)
                if t_ == 0:
                    t_ = 0.001
                t /= t_

                if t < 0:
                    t = 1000
                
                ultrasonicXHit = (1-t)*self.x + ultrasonicPoint.x*t
                if ultrasonicXHit < yLine.xRangeDown or ultrasonicXHit > yLine.xRangeUp:
                    t = 1000

                ty.append(t)

            for robot in robots:
                if robot != self:
                    rx = robot.x
                    ry = robot.y
                    rr = robot.size

                    x0 = self.x
                    x1 = ultrasonicPoint.x
                    y0 = self.y
                    y1 = ultrasonicPoint.y

                    a = (x1 - x0)**2 + (y1-y0)**2
                    b = 2*(x1-x0)*(x0-rx) + 2*(y1-y0)*(y0-ry)
                    c = (x0-rx)**2 + (y0-ry)**2 - rr**2
                    
                    bac = b**2 - 4*a*c 
                    if bac < 0:
                        t = 1000
                    else:
                        t1 = (-b + math.sqrt(bac) ) / (2*a)
                        t2 = (-b - math.sqrt(bac) ) / (2*a)
                        t = min(t1,t2)
                        if t < 0:
                            t = 1000
                    tr.append(t)

            tList = ty+tx+tr
            t = min(tList)

            ultrasonicXHit = (1-t)*self.x + ultrasonicPoint.x*t
            ultrasonicYHit = (1-t)*self.y + ultrasonicPoint.y*t
            ultrasonicHitPoint = Point(ultrasonicXHit, ultrasonicYHit)

            self.ultrasonicHitPoints.append(ultrasonicHitPoint)

            distanceToHit = ((ultrasonicHitPoint.x-ultrasonicPoint.x)**2+(ultrasonicHitPoint.y-ultrasonicPoint.y)**2)**(1/2)
            maxBoundaryDistance = ((arena.boundary.x1-arena.boundary.x2)**2+(arena.boundary.y1-arena.boundary.y2)**2)**(1/2)
            
            observation.append(distanceToHit/maxBoundaryDistance)

        return observation
    

    def line(self, arena):
        '''
        Line Sensor - Detects if the robot is over an arena boundary white line.
        
        Parameters:
            Type:   Name:   Unit:
            (Arena) arena   N/A
        
        Returns:
            Type:   Name:       Unit:
            (list)  observation N/A
        
        '''
        observation = []

        t = arena.field.thickness/2

        if arena.field.outerBox.contains(self.x, self.y) == True and arena.field.innerBox.contains(self.x, self.y) == False:
            observation.append(1.0)
            self.lineVal = 1
        else:
            observation.append(0.0)
            self.lineVal = 0
        
        return observation


    def draw(self, image, scale=100):
        '''
        Draws object to image

        Parameters:
            Type:           Name:   Unit:
            (np.ndarray)    image   N/A
            (float)         scale   N/A

        '''
        cv2.circle(image,(int(self.x*scale), int(self.y*scale)), int(self.size*scale), (80,80,80), -1, cv2.LINE_AA)

        frontPlacementRatio = 2/3
        frontX = self.x + (np.cos(self.rotation)*self.size*frontPlacementRatio)
        frontY = self.y + (np.sin(self.rotation)*self.size*frontPlacementRatio)
        frontSize = self.size * (1-frontPlacementRatio)
        cv2.circle(image,(int(frontX*scale), int(frontY*scale)), int(frontSize*scale), (50,50,50), -1, cv2.LINE_AA)

    def draw_sensors(self, image, scale=100, compass=True, infared=True, ultrasonic=True, line=True):
        '''
        Draws sensor info to image

        Parameters:
            Type:           Name:       Unit:
            (np.ndarray)    image       N/A
            (float)         scale       N/A
            (bool)          compass     N/A
            (bool)          ultrasonic  N/A
            (bool)          infared     N/A
            (bool)          line        N/A

        '''
        if compass == True:
            overlay = image.copy()
            for i in range(len(self.compassPoints)):
                cv2.line(overlay, (int(self.compassPoints[i].x*scale),int(self.compassPoints[i].y*scale)), (int(self.x*scale),int(self.y*scale)), (0,255,255), int(scale*self.size*0.1), cv2.LINE_AA)
            alpha = 0.8
            cv2.addWeighted(overlay, alpha, image, 1-alpha, 0, image)

        if infared == True:
            overlay = image.copy()
            for i in range(len(self.infaredPoints)):
                color = 0 
                if self.infaredValues[i] == 0:
                    color = (150,150,150)
                else:
                    color = (100,100,255)
                cv2.circle(overlay, (int(self.infaredPoints[i].x*scale),int(self.infaredPoints[i].y*scale)), int(self.size/5*scale), color, -1, cv2.LINE_AA)
                cv2.line(overlay, (int(self.infaredPoints[i].x*scale),int(self.infaredPoints[i].y*scale)), (int(self.infaredHitPoints[i].x*scale),int(self.infaredHitPoints[i].y*scale)), color, 1, cv2.LINE_AA)
                cv2.circle(overlay, (int(self.infaredHitPoints[i].x*scale),int(self.infaredHitPoints[i].y*scale)), int(self.size/7*scale), color, -1, cv2.LINE_AA)
            alpha = 0.8
            cv2.addWeighted(overlay, alpha, image, 1-alpha, 0, image)

        if ultrasonic == True: 
            overlay = image.copy()
            for i in range(len(self.ultrasonicPoints)):
                cv2.line(overlay, (int(self.ultrasonicPoints[i].x*scale),int(self.ultrasonicPoints[i].y*scale)), (int(self.ultrasonicHitPoints[i].x*scale),int(self.ultrasonicHitPoints[i].y*scale)), (255,255,0), 1, cv2.LINE_AA)
                cv2.circle(overlay, (int(self.ultrasonicHitPoints[i].x*scale),int(self.ultrasonicHitPoints[i].y*scale)), int(self.size/5*scale), (255,255,0), -1, cv2.LINE_AA)
            alpha = 0.8
            cv2.addWeighted(overlay, alpha, image, 1-alpha, 0, image)

        if line == True:
            color = 0
            if self.lineVal == 0:
                color = (100,100,100)
            else:
                color = (180,180,180)
            cv2.circle(image,(int(self.x*scale), int(self.y*scale)), int(self.size/5*scale), color, -1, cv2.LINE_AA)

        




