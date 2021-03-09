import math

length = 72
height = 54

#get the distance between two points
def getDistance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

#find the angle needed to reach a point from the origin
def find1(x, y):
    th = atan2(y, x)
    if th < 0: th = th + (2 * math.pi)
    return th

#get the angle the robot needs to acheive to head straight to the next point
#treating x1, y1, as the origin
def find2(x1, y1, x2, y2):
    #put x2, y2 into local coordinates
    x2 = x2 - x1
    y2 = y2 - y1
    
    if ((math.fabs(x2) < 1e-7) and (y2 < 0)):
        theta = 1.5 * math.pi
    elif ((x2 == 0) and (y2 > 0)):
        theta = 0.5 * math.pi
    else:
        theta = math.atan2(y2, x2)
        
    if (theta < 0):
        theta = theta + 2 * math.pi
    return math.fmod(theta, 2 * math.pi)
    