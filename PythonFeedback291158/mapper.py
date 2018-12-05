import math

class Point(object):
    def __init__(self, x, y):
        self.X = x
        self.Y = y

    def __str__(self):
        return "Point(%s,%s)"%(self.X, self.Y)

    def getX(self):
        return self.X

    def getY(self):
        return self.Y

    def distance(self, other):
        dx = self.X - other.X
        dy = self.Y - other.Y
        return math.sqrt(dx**2 + dy**2)

    #shortest distance between this point and the line (infinite length) formed by two points
    def distanceToLine(self, other1, other2):
        x0 = self.X
        y0 = self.Y
        x1 = other1.X
        y1 = other1.Y
        x2 = other2.X
        y2 = other2.Y

        return abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/other1.distance(other2)

    #shortest distance between this point and the line segment (limited length) formed by two points
    def distanceToSegment(self, other1, other2):
        x0 = self.X
        y0 = self.Y
        x1 = other1.X
        x2 = other2.X
        y1 = other1.Y
        y2 = other2.Y

        #make sure point 1 is to the left (west) of point 2
        if x1 > x2:
                x1=other2.X
                x2=other1.X
                y1=other2.Y
                y2=other1.Y
                
        #vertical road case
        elif x2 == x1:
            if y1 > y2:
                y1=other2.Y
                y2=other1.Y
            if y0 <= y1:
                return self.distance(Point(x1,y1))
            elif y0 > y2:
                return self.distance(Point(x2,y2))
            else:
                return abs(x0-x1)

        #horizontal road case   
        if y1 == y2:
            if x0 < x1:
                return self.distance(Point(x1,y1))
            elif x0 > x2:
                return self.distance(Point(x2,y2))
            else:
                return abs(y0-y1)

        #slope of the line perpendicular to the inputted line
        perpSlope = (x1 - x2) / (y2 - y1)

        #perp slope positive case
        if perpSlope > 0:
            if y0 > perpSlope * (x0 - x1) + y1:
                return self.distance(Point(x1,y1))
            elif y0 < perpSlope * (x0 - x2) + y2:
                return self.distance(Point(x2,y2))
            else:
                return self.distanceToLine(other1, other2)
        
        #perp slope negative case
        if perpSlope < 0:
            if y0 < perpSlope * (x0 - x1) + y1:
                return self.distance(Point(x1,y1))
            elif y0 > perpSlope * (x0 - x2) + y2:
                return self.distance(Point(x2,y2))
            else:
                return self.distanceToLine(other1, other2)


class Road(object):
    def __init__(self, start, end, forwardSpeed, backwardSpeed):
        self.Start = start
        self.End = end
        self.ForwardSpeed = forwardSpeed
        self.BackwardSpeed = backwardSpeed
        
    def __str__(self):
        return "Road from %s to %s, speed %s and %s"%(self.Start, self.End, self.ForwardSpeed, self.BackwardSpeed)

    def getStart(self):
        return self.Start

    def getEnd(self):
        return self.End
    
    def getForwardSpeed(self):
        return self.ForwardSpeed

    def getBackwardSpeed(self):
        return self.BackwardSpeed

    #heading going from start to end
    def getDirection(self):
        deg = (90 - math.degrees(math.atan2(self.End.getY()-self.Start.getY(), self.End.getX()-self.Start.getX()))) % 360
        if deg < 0:
            deg = 360 - deg
        return deg

class Mapper(object):
    def __init__(self, nodeFile, edgeFile):
        self.NodeFile = nodeFile
        self.EdgeFile = edgeFile
        self.Dist = None
        self.CurrentRoad = None
        self.CurrentRoadName = None
        self.DistanceFromBeginning = None
        self.BeginningNode = None
        self.Direction = None
        self.RoadHeading = None

        f=open(self.NodeFile, 'r')
        lines=f.readlines()
        f.close()

        self.PointDict = {}

        for line in lines:
            line = line.replace('  ', ' ') #account for node names that are two letters long
            s = line.split(' ')
            self.PointDict[s[0]] = Point(float(s[2]), float(s[1]))


        f=open(self.EdgeFile, 'r')
        lines=f.readlines()
        f.close()

        self.RoadDict = {}
        self.PathDict = {}
        for line in lines:
            s = line.split(' ')
            start = s[0]
            end = s[1]
            forwardSpeed = float(s[2])
            backwardSpeed = float(s[3])
            degreesToMeters = 111139 #conversion factor

            #build a dictionary of roads and their speed limits
            roadKey = start + " " + end
            self.RoadDict[roadKey] = Road(self.PointDict[start], self.PointDict[end], forwardSpeed, backwardSpeed)

            #build a dictionary of paths, with a cost equal to the travel time between them (at the speed limit)
            if not self.PathDict.has_key(start) and float(forwardSpeed) != 0:
                self.PathDict[start] = [(end, self.PointDict[start].distance(self.PointDict[end]) * degreesToMeters / float(forwardSpeed))]
            else:
                if self.PathDict.has_key(start) and float(forwardSpeed) != 0:
                    self.PathDict[start].append((end, self.PointDict[start].distance(self.PointDict[end]) * degreesToMeters / float(forwardSpeed)))
            if not self.PathDict.has_key(end) and float(backwardSpeed) != 0:
                self.PathDict[end] = [(start, self.PointDict[start].distance(self.PointDict[end]) * degreesToMeters / float(backwardSpeed))]
            else:
                if self.PathDict.has_key(end) and float(backwardSpeed) != 0:
                    self.PathDict[end].append((start, self.PointDict[start].distance(self.PointDict[end]) * degreesToMeters / float(backwardSpeed)))

    def printPathDict(self):
        for key, value in self.PathDict.items():
            print "%s: %s"%(key, value)

    def printRoadDict(self):
        for key, value in self.RoadDict.items():
            print "%s: %s"%(key, value)
            
    def printPointDict(self):
        for key, value in self.PointDict.items():
            print "%s: %s"%(key, value)
            
    def findRoad(self, gpsX, gpsY, gpsH):

        gpsCurrent = Point(gpsX, gpsY)

        for n, r in self.RoadDict.iteritems():
            p1 = r.getStart()
            p2 = r.getEnd()
            currentDist = gpsCurrent.distanceToSegment(p1, p2)
            if currentDist < self.Dist or self.Dist is None:
                self.Dist = currentDist
                self.CurrentRoad = r
                self.CurrentRoadName = n

        #find direction on road ("forward" or "backward")
        self.RoadHeading = self.CurrentRoad.getDirection()
        d = gpsH-self.RoadHeading
        if d > 180:
            d -= 360
        elif d < -180:
            d += 360
        if d > 90:
            self.Direction = "backward"
        else:
            self.Direction = "forward"

        #determine the beginning node of the road
        if self.Direction == "backward":
            beginningNode = self.CurrentRoad.getEnd()
        else:
            beginningNode = self.CurrentRoad.getStart()
        self.DistanceFromBeginning=gpsCurrent.distance(beginningNode)

        #adjust road heading if you are going "backward"
        if self.Direction == "backward":
            roadHeading = (roadHeading + 180) % 360

    #return the an ordered list of the fastest route from a start node to an end node
    def findPath(self, current, end, visited, cost):
        if len(visited) == 0:
            self.CompletePaths = []
        
        newVisited = visited + [current]

        if not self.PathDict.has_key(current) or not self.PointDict.has_key(end):
            return None
        if current == end:
            if len(newVisited) == 1:
                return newVisited
            return (newVisited, cost)
        
        for value in self.PathDict[current]:

            nextNode=value[0]
            cost += value[1]

            if newVisited.count(value[0]) < 1:
                p = self.findPath(nextNode, end, newVisited, cost)
                if not p is None:
                    self.CompletePaths.append(p)
        if len(visited) == 0:
            return sorted(self.CompletePaths, key=lambda path: path[1])[0][0]

    def getDist(self):
        return self.Dist

    def getCurrentRoad(self):
        return self.CurrentRoad

    def getCurrentRoadName(self):
        return self.CurrentRoadName

    def getDistanceFromBeginning(self):
        return self.DistanceFromBeginning

    def getBeginningNode(self):
        return self.BeginningNode

    def getDirection(self):
        return self.Direction

    def getRoadHeading(self):
        return self.RoadHeading

    def getSpeedLimit(self):
        if self.Direction == 'forward':
            return self.CurrentRoad.getForwardSpeed()
        if self.Direction == 'backward':
            return self.CurrentRoad.getBackwardSpeed()

def takeImage(cam, fileName):
    cam.capture(fileName)
    return None
