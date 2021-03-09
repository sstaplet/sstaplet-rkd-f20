import math
import checkZones as cz
pt1 = (11, 8)
pt2 = (26.75, 8)
pt3 = (60, 8)
pt4 = (60, 43.2)
pt5 = (26.75, 43.2)
pt6 = (7, 43.2)
pts = [pt1, pt2, pt3, pt4, pt5, pt6]

#find the distance between two points
def getDistance(p1, p2):
    (x1, y1) = p1
    (x2, y2,) = p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

#get the waypoints we need to hit
def grabWaypoints(x0, y0, xGoal, yGoal):
    #grab the start and end zoens
    start = cz.getZone(x0, y0)
    end = cz.getZone(xGoal, yGoal)

    #grab our path
    (path, w, v, found) = grabPath(start, end, [], False)
    print(path)
    wp = convertPath(path)
    #add our start and end points to the waypoints
    wp = [(x0, y0)] + wp + [(xGoal, yGoal)]
    return wp

#dfs with weights to find the shortest path
def grabPath(ptCurr, ptGoal, visited, found):
    #add our current node to the visited nodes
    visited = visited + [ptCurr]
    #if we've reached our goal return with found = True
    if ptCurr == ptGoal:
        return ([ptCurr], 0, visited, True)
    #if we haven't reached our goal
    else:
        #initialize variables
        res = []
        w = 0
        newVisited = visited.copy()
        #for each neighbor to this node
        for neighbor in graph[ptCurr]:
            #if we haven't seen the neighbor in this path
            if neighbor[0] not in visited:
                #recursively find a path from this point
                (path, wCurr, thisVisited, found) = grabPath(neighbor[0], ptGoal, visited, False)
                #if this is the first path found or is less weight than before
                if found and (((wCurr + neighbor[1]) < w) or w == 0):
                    #make the result this path, weight, and visited list
                    res = [ptCurr] + path
                    w = wCurr + neighbor[1]
                    newVisited = thisVisited
        #return true if we did find a path here
        if res != []: return (res, w, newVisited, True)
        #return false otherwise
        return ([],0, visited, False)

#convert the node path to a waypoint path
def convertPath(p):
    res = []
    #go through each node and find it's coordinate
    for pt in p:
        i = int(pt) - 1
        res += [pts[i]]
    return res


#set up our point graph below
graph =	{
	      "1" : [["2", getDistance(pt1, pt2)]],
	      "2" : [["1", getDistance(pt1, pt2)], ["3", getDistance(pt2, pt3)], ["5", getDistance(pt2, pt5)]],
	      "3" : [["2", getDistance(pt3, pt2)], ["4", getDistance(pt3, pt4)]],
	      "4" : [["3", getDistance(pt4, pt3)], ["5", getDistance(pt4, pt5)]],
	      "5" : [["2", getDistance(pt5, pt2)], ["4", getDistance(pt5, pt4)], ["6", getDistance(pt5, pt6)]],
	      "6" : [["5", getDistance(pt5, pt6)]]
	      }


