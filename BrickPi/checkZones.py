import math
#check if our point is in zone 1
def checkZone1(x, y):
    xLow = 0
    xHigh = 17.97
    yLow = 0
    yHigh = 25.5
    xCheck = (xLow <= x) and (x < xHigh)
    yCheck = (yLow <= y) and (y < yHigh)
    slopeCheck = (0.812*x + y) < 28.5           # l1
    return xCheck and yCheck and slopeCheck

#check if our point is in zone 2
def checkZone2(x, y):
    xLow = 17.97
    xHigh = 36.56
    yLow = 0
    yHigh = 19
    xCheck = (xLow <= x) and (x < xHigh)
    yCheck = (yLow <= y) and (y < yHigh)
    slope1Check = (1.37*x - y) > 10.54          # l2
    slope2Check = (0.855*x + y) < 45.86         # l3
    slope3Check = (0.0476*x + y) < 20.02        # l4
    return xCheck and yCheck and slope1Check and slope2Check and slope3Check

#check if our point is in zone 3
def checkZone3(x, y):
    xLow = 36.56
    xHigh = 72
    yLow = 0
    yHigh = 28.5
    xCheck = (xLow <= x) and (x < xHigh)
    yCheck = (yLow <= y) and (y < yHigh)
    slopeCheck = (1.215*x - y) > 29.82          # l5
    return xCheck and yCheck and slopeCheck

#check if our point is in zone 4
def checkZone4(x, y):
    xLow = 43.44
    xHigh = 72
    yLow = 28.5
    yHigh = 54
    xCheck = (xLow <= x) and (x < xHigh)
    yCheck = (yLow <= y) and (y < yHigh)
    slopeCheck = (0.855*x + y) > 69.55          # l6
    return xCheck and yCheck and slopeCheck

#check if our point is in zone 5
def checkZone5(x, y):
    xLow = 7.53
    xHigh = 43.44
    yLow = 18.5
    yHigh = 54
    xCheck = (xLow <= x) and (x < xHigh)
    yCheck = (yLow <= y) and (y < yHigh)
    slope1Check = (1.215*x - y) < 20.38         # l7
    slope2Check = (0.812*x + y) > 36.47         # l8
    slope3Check = (0.0476*x + y) > 20.02        # l4
    return xCheck and yCheck and slope1Check and slope2Check and slope3Check

#check if our point is in zone 6
def checkZone6(x, y):
    xLow = 0
    xHigh = 7.53
    yLow = 25.5
    yHigh = 54
    xCheck = (xLow <= x) and (x < xHigh)
    yCheck = (yLow <= y) and (y < yHigh)
    slopeCheck = (1.37*x - y) < -20             #l9
    return xCheck and yCheck and slopeCheck

#check all the zones and return which one our point is in
def getZone(x, y):
    if checkZone1(x, y): return '1'
    elif checkZone2(x, y): return '2'
    elif checkZone3(x, y): return '3'
    elif checkZone4(x, y): return '4'
    elif checkZone5(x, y): return '5'
    elif checkZone6(x, y): return '6'
    else: return '5'