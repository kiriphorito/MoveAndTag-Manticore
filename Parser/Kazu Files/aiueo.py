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

import random
import math
import copy

global global_unawakenRobots
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
    
    def __init__(self, start, goal, obstacleList, randArea,expandDis=1.0,goalSampleRate=5,maxIter=500):
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
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.robots = global_unawakenRobots
    
    def Planning(self,animation=True):
        u"""
        Pathplanning 

        animation: flag for animation on or off
        """
        robots = self.robots
        
        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode =self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind
            path = []
            foundNode = []
            if not self.__CollisionCheck(newNode, obstacleList,nearestNode):
                continue

            self.nodeList.append(newNode)
            check = False
            for (gx,gy) in robots:
                # check goal
                dx = newNode.x - gx
                dy = newNode.y - gy
                d = math.sqrt(dx * dx + dy * dy)
                if d <= self.expandDis:
                    if not self.__CollisionCheck(newNode, obstacleList,Node(gx,gy)): #if intersects
                        continue
                    else:
                        print "ok"
                        print (gx,gy)
                        path += [[gx,gy]]
                        foundNode.append((gx,gy))
                        check = True
                        #print("Goal!!")
                        break
        
            if animation:
                self.DrawGraph(rnd)


            if check:
                break
#        path=[[self.end.x,self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x,node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return {'path': path, 'node': foundNode}

    def DrawGraph(self,rnd=None):
        u"""
            Draw Graph
            """
        import matplotlib.pyplot as plt
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
            for node in self.nodeList:
                if node.parent is not None:
                    plt.plot([node.x, self.nodeList[node.parent].x], [node.y, self.nodeList[node.parent].y], "-g")
        
        drawRobots(robots)
        drawPolygons(obstacleList)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis('scaled')
        plt.grid(True)
        plt.pause(0.01)

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

def rrtpath(obstacles,startcoord,goalcoord,randAreas):
    rrt = RRT(start=startcoord, goal=goalcoord,randArea = randAreas, obstacleList=obstacles)
    answer = rrt.Planning(animation=True)
    path= answer['path']
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-r')
    smoothiePath.reverse()
    print smoothiePath
    return {'path': smoothiePath, 'node': answer['node']}


def rrtshortestpath(currentNode,obstacleList,previousPath,rand,numAct):
    global global_unawakenRobots
    unawakenRobots = global_unawakenRobots
    print ("left",unawakenRobots)
    print len(unawakenRobots)
    if len(unawakenRobots) == 0:
        print "0 left"
        return
    if len(unawakenRobots) == 1:
        print "final"
        print (currentNode,unawakenRobots[-1])
        a= rrtpath(obstacleList,currentNode[0],unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        print previousPath
        print a['path']
        newPath = previousPath + a['path']
        print newPath
        return
    if numAct == 1:
        print "num = 1"
        a = rrtpath(obstacleList,currentNode,unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        print ("unawaken = ",unawakenRobots)
        print previousPath
        print a['path']
        newPath = previousPath + a['path']
        print newPath
        print "finding next node"
        rrtshortestpath(foundNode[0],obstacleList,newPath,rand,2)
        return
    elif numAct == 2:
        print "num = 2"
        if(type(currentNode)== tuple):
            fixedTypeCurrentNode = currentNode
            print "tuple"
        else:
            print "list"
            fixedTypeCurrentNode = currentNode[0]
        print (currentNode,unawakenRobots[-1])
        a = rrtpath(obstacleList,fixedTypeCurrentNode,unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        print ("unawaken 1= ",unawakenRobots)
        newPath = previousPath + a['path']
        rrtshortestpath(foundNode,obstacleList,newPath,rand,2)
        return 
#        b = rrtpath(obstacleList,currentNode,unawakenRobots[-1],rand)
#        foundNode = b['node']
#        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
#        print ("unawaken 2= ",unawakenRobots)
#        newPath = [b['path']]
#        rrtshortestpath(foundNode,obstacleList,newPath,rand,2)
#        return

    return







robots = [(2,9),(4,4),(7,5)]

obstacleList = [[(1,6),(1,1),(5,1),(5,5),(3,5),(3,3),(4,3),(4,2),(2,2),(2,6),(6,6),(6,0),(0,0),(0,6)]]

start = (4,4)
goal = (7,5)
rand = (-1,12)
global_unawakenRobots = [(2,9),(7,5),(10,10)]

print global_unawakenRobots[-1]
#a = rrtpath(obstacleList,start,global_unawakenRobots[-1],rand)
rrtshortestpath(start,obstacleList,[],rand,1)

#print ("unawaken = ",unawakenRobots)
#
#a = rrtpath(obstacleList,start,goal,rand)
#print a
##print ("path = ", a['path'])
##print ("found node = ", a['node'])
#nextNode = a['node']
#print nextNode[0]
#
##ab= unawakenRobots
##unawakenRobots = [item for item in ab if item != nextNode[0]]
#unawakenRobots = [item for item in unawakenRobots if item != nextNode[0]]
#print ("unawaken = ",unawakenRobots)
##unawakenRobots.
#b = rrtpath(obstacleList,nextNode[0],goal,rand)
#print b

#rrtpath(obstacleList,robots,start,rand) #return closest node 


drawRobots(robots)
drawPolygons(obstacleList)

plt.axis('scaled')
plt.grid(True)
plt.pause(0.01)  # Need for Mac
plt.show()
