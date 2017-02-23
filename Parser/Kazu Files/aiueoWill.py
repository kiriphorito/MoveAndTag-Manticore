#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""
@brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai

@license: MIT

"""

import shapely
from shapely.geometry import Polygon, LineString, Point, MultiPoint, GeometryCollection
import matplotlib.pyplot as plt
from ast import literal_eval
import threading
import time


import random
import math
import copy
def drawRobots(robots):
    for (x,y) in robots:
        plt.plot(x,y,"o")

def drawPolygonNoFill(points,color):
    polygon = plt.Polygon(points,color=color,fill=False)
    plt.gca().add_patch(polygon)

def drawPolygon(points):
    polygon = plt.Polygon(points)
    plt.gca().add_patch(polygon)

def drawPolygons(polygons):
    try:
        for xs in polygons:
            drawPolygon(xs)
    except ValueError:
        print ("no polygons specified")

def drawPolygonNoFill(points,color):
    polygon = plt.Polygon(points,color=color,fill=False)
    plt.gca().add_patch(polygon)

def drawPolygonsNoFill(polygons):
    try:
        for xs in polygons:
            drawPolygonNoFill(xs,'red')
    except ValueError:
        print ("no polygons specified")

def LineCollisionCheck(first,second, obstacleList):
    from shapely import geometry,wkt
    EPS = 1e-15
    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]
    line = geometry.LineString([(x1,y1),(x2,y2)])
    for p1 in obstacleList:
        poly = geometry.Polygon(p1)
        ips = line.intersection(poly.boundary)
        if type(ips) is Point:
            if ips.distance(poly) < EPS:
                return False
        elif type(ips) is MultiPoint:
            for i in ips:
                if (i.distance(poly) <EPS):
                    return False
        elif type(ips) is GeometryCollection:
            continue
        else:
            print (ips,type(ips))
            return False
    return True

def supersmoothie(smoothie,obstacleList):
    path = smoothie
    state = True
    counter1 = 0
    counter2 = len(path)-1
    while state:
        counter2 = len(path)-1
        if counter1 == counter2:
            state = False
            break
        coord1 = path[counter1]
        for counter in range(counter2,0,-1):
            coord2 = path[counter]
            if LineCollisionCheck(coord1,coord2,obstacleList): #if no obstacle
                del path[(counter1+1):(counter)]
                break
        counter1 += 1
    return path

class Node():
    u"""
        RRT Node
        """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,randArea,robots,expandDis=1.0,goalSampleRate=50,maxIter=500):
        u"""
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start=Node(start[0],start[1])
        self.end=Node(goal[0],goal[1])
        self.minrand = randArea[0]
        self.robots = copy.deepcopy(robots)
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter

        self.robots.remove(start)

        print self.robots

    def Planning(self,animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """
        paths = [[(self.start.x,self.start.y)]]
        robots = self.robots
        minDis = 1000000000
#        currentDestination = goal
        self.nodeList = [self.start]
        while True:
            # Random Sampling
#            minDis = 1000000000
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
#            print "H",nind

            # expand tree
            nearestNode =self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind
            path = []


            if not self.__CollisionCheck(newNode, obstacleList,nearestNode):
                continue

            self.nodeList.append(newNode)
#            print "H",newNode.x,newNode.y
            check = False
            for (gx,gy) in self.robots:
                #Find closest robot
                dx = newNode.x - gx
                dy = newNode.y - gy
                d = math.sqrt(dx * dx + dy * dy)
#                print (gx,gy)
#                print d
                if d < minDis:
                    print "MIN"
                    print d
                    minDis = d
                    self.end = Node(gx,gy)
                    print self.end.x,self.end.y

            # check goal
            gx = self.end.x
            gy = self.end.y
            dx = newNode.x - gx
            dy = newNode.y - gy
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                if not self.__CollisionCheck(newNode, obstacleList,Node(gx,gy)): #if intersects
                    continue
                else:
                    print "ok"
#                    print (gx,gy)
                    path += [[gx,gy]]
                    check = True
                    for p in paths:
                        if cmp(p[0],(self.start.x,self.start.y)) == 0:
                            p += [(gx,gy)]
                            paths += [[(gx,gy)]]
                            print paths

                    #print("Goal!!")
                    break

#            if check:
#                break
#        path=[[self.end.x,self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x,node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node,obstacleList,nearestNode):
        x1 = nearestNode.x
        y1 = nearestNode.y
        x2 = node.x
        y2 = node.y
        first = [x1,y1]
        second = [x2,y2]
        return LineCollisionCheck(first,second,obstacleList)





#=====end of RRT class

def rrtpath(obstacles,startcoord,goalcoord,randAreas,robots):
    rrt = RRT(start=startcoord, goal=goalcoord,randArea = randAreas, obstacleList=obstacles, robots=robots)
    path= rrt.Planning(animation=False)
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-r')
    smoothiePath.reverse()
    return smoothiePath

class myThread (threading.Thread):
    def __init__(self, threadID, name, ):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        # self.rand = rand
        # self.obstacleList = obstacleList
        # self.start = start
        # self.goal = goal
        # self.robots = robots
        print "OK"
    def run(self,obstacleList, start, goal, rand,robots):
        print "Y"
        rrtpath(obstacleList,start,goal,rand,robots)

robots = [(4,4),(-1,5),(7,0),(1.5,10)]



obstacleList = [[(1,6),(1,1),(5,1),(5,5),(3,5),(3,3),(4,3),(4,2),(2,2),(2,6),(6,6),(6,0),(0,0),(0,6)]]

start = (4,4)
goal = (7,5)
rand = (-1,8)

# Create new threads
thread1 = myThread(1, "Thread-1")
thread2 = myThread(2, "Thread-2")

# Start new Threads
thread1.run(obstacleList, start, goal, rand,robots)
thread2.run(obstacleList, goal, start, rand,robots)


#robots = [(4,4),(-1,5),(7,0)]

#rrtpath(obstacleList,robots,start,rand) #return closest node


drawRobots(robots)
drawPolygons(obstacleList)

plt.axis('scaled')
plt.grid(True)
plt.pause(0.01)  # Need for Mac
plt.show()
