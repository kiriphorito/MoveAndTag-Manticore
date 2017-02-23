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
import datetime

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



class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,randArea,expandDis=1.0,goalSampleRate=5,maxIter=500):
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

    def Planning(self,animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """

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

            if not self.__CollisionCheck(newNode, obstacleList,nearestNode):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                if not self.__CollisionCheck(newNode, obstacleList,self.end):
                    continue
                else:

                #print("Goal!!")
                    break

            if animation:
                self.DrawGraph(rnd)


        path=[[self.end.x,self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x,node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

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
#        plt.plot([ox for (ox,oy,size) in obstacleList],[oy for (ox,oy,size) in obstacleList], "ok", ms=size * 20)
        drawPolygons(obstacleList)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis()
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


def LineCollisionCheck(first,second, obstacleList):
    from shapely import geometry,wkt
    EPS = 1.2e-16 #======= may need to change this value depending on precision
    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    line = geometry.LineString([(x1,y1),(x2,y2)])

#============ changed here =======


#    for p1 in obstacleList:
#
#        poly = geometry.Polygon(p1)
#        ips = line.intersection(poly.boundary)
##        print ips
#        if type(ips) is Point:
##            print "hello"
#            if ips.distance(poly) < EPS:
##                print "INTERSECT"
#                return False
#        elif type(ips) is MultiPoint:
#            for i in ips:
#                if (i.distance(poly) <EPS):
##                    print "INTERSECT2"
#                    return False
#        elif type(ips) is GeometryCollection:
#            continue
#        else:
#            print (ips,type(ips))
#            return False
#    return True

#============ changed here =======

    for poly in obstacleList:
        p1 = Polygon(poly)
        if p1.buffer(EPS).intersects(line):
            #                print "collision"
            return False
    #        print "safe"
    return True

#============ changed here =======

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


def rrtpath(obstacles,startcoord,goalcoord,randAreas):
    rrt = RRT(start=startcoord, goal=goalcoord,randArea = randAreas, obstacleList=obstacles)
    path= rrt.Planning(animation=False)
#    rrt.DrawGaph()
#    plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')
#    print path
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-r')
    smoothiePath.reverse()
    #print smoothiePath
    return smoothiePath

obstacleList = []
rand = (-220,220)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 399"
path = []
start = (63.72633384476626,31.422393905288004)
goal = (58.44823976670091,29.604228741788063)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 399"
path = []
start = (58.44823976670091,29.604228741788063)
goal = (53.75764436427187,22.147994313542853)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 399"
path = []
start = (53.75764436427187,22.147994313542853)
goal = (57.70496923125779,15.301591399635583)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 399"
path = []
start = (57.70496923125779,15.301591399635583)
goal = (64.21564017266815,9.569443144511695)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 399"
path = []
start = (64.21564017266815,9.569443144511695)
goal = (55.77529184632647,1.8719458852445428)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 399"
path = []
start = (55.77529184632647,1.8719458852445428)
goal = (50.944720643900865,4.118990649348177)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 399"
path = []
start = (50.944720643900865,4.118990649348177)
goal = (33.90798790289432,-8.435144401088166)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 399"
path = []
start = (33.90798790289432,-8.435144401088166)
goal = (28.630825266963512,-19.783666701140135)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 399"
path = []
start = (28.630825266963512,-19.783666701140135)
goal = (44.11484975702115,-28.603348652659264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 399"
path = []
start = (44.11484975702115,-28.603348652659264)
goal = (43.08263964478712,-40.32895249063958)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 399"
path = []
start = (43.08263964478712,-40.32895249063958)
goal = (29.95891969398491,-44.97446889481739)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 399"
path = []
start = (29.95891969398491,-44.97446889481739)
goal = (29.321279993177768,-54.937500535629795)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 399"
path = []
start = (29.321279993177768,-54.937500535629795)
goal = (15.361046710366054,-51.87987806082663)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 399"
path = []
start = (15.361046710366054,-51.87987806082663)
goal = (12.630538580421302,-39.10205295901528)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 399"
path = []
start = (12.630538580421302,-39.10205295901528)
goal = (-9.652444019821388,-44.385713627900884)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 399"
path = []
start = (-9.652444019821388,-44.385713627900884)
goal = (-18.321713229041194,-38.99353413316214)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 399"
path = []
start = (-18.321713229041194,-38.99353413316214)
goal = (-42.36973140979089,-49.14385482156516)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 399"
path = []
start = (-42.36973140979089,-49.14385482156516)
goal = (-46.823213241873674,-52.70221985789419)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 399"
path = []
start = (-46.823213241873674,-52.70221985789419)
goal = (-46.184952971630224,-61.445835535430774)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 399"
path = []
start = (-46.184952971630224,-61.445835535430774)
goal = (-36.33292295546718,-73.75050293549812)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 399"
path = []
start = (-36.33292295546718,-73.75050293549812)
goal = (-17.499085061460306,-70.34555730261599)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 399"
path = []
start = (-17.499085061460306,-70.34555730261599)
goal = (-8.573988681231327,-72.20422883226303)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 399"
path = []
start = (-8.573988681231327,-72.20422883226303)
goal = (-1.7264639158888997,-77.71961295163341)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 399"
path = []
start = (-1.7264639158888997,-77.71961295163341)
goal = (3.7098379932831165,-70.89560288379144)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 399"
path = []
start = (3.7098379932831165,-70.89560288379144)
goal = (26.618520315702995,-69.85470172454606)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 399"
path = []
start = (26.618520315702995,-69.85470172454606)
goal = (54.41258788827034,-65.92379136721064)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 399"
path = []
start = (54.41258788827034,-65.92379136721064)
goal = (57.13664149044911,-48.69152638873837)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 399"
path = []
start = (57.13664149044911,-48.69152638873837)
goal = (68.71618065598273,-43.76855717147765)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 399"
path = []
start = (68.71618065598273,-43.76855717147765)
goal = (58.046110661892556,-22.21879773081409)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 399"
path = []
start = (58.046110661892556,-22.21879773081409)
goal = (69.39176200150496,-17.82835399292844)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 399"
path = []
start = (69.39176200150496,-17.82835399292844)
goal = (71.7325010905464,-7.896859777585121)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 399"
path = []
start = (71.7325010905464,-7.896859777585121)
goal = (93.08293617644074,-21.291695502116028)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 399"
path = []
start = (93.08293617644074,-21.291695502116028)
goal = (93.75046741861684,-20.427408917766144)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 399"
path = []
start = (93.75046741861684,-20.427408917766144)
goal = (95.07224592153113,-28.757944112661136)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 399"
path = []
start = (95.07224592153113,-28.757944112661136)
goal = (101.85427326195844,-44.522033031088824)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 399"
path = []
start = (101.85427326195844,-44.522033031088824)
goal = (104.87832353455377,-63.72380533964636)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 399"
path = []
start = (104.87832353455377,-63.72380533964636)
goal = (103.88520015013603,-66.53481891761461)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 399"
path = []
start = (103.88520015013603,-66.53481891761461)
goal = (103.94125647388574,-69.62434085870544)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 399"
path = []
start = (103.94125647388574,-69.62434085870544)
goal = (107.85810993251374,-67.02538612708082)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 399"
path = []
start = (107.85810993251374,-67.02538612708082)
goal = (118.35662471023898,-60.97714994914759)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 399"
path = []
start = (118.35662471023898,-60.97714994914759)
goal = (127.99159295651776,-52.79414509550341)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 399"
path = []
start = (127.99159295651776,-52.79414509550341)
goal = (133.33381520694627,-47.11479774888207)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 399"
path = []
start = (133.33381520694627,-47.11479774888207)
goal = (135.54357068683953,-31.28595797922111)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 399"
path = []
start = (135.54357068683953,-31.28595797922111)
goal = (138.67020413378037,-28.619085361359936)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 399"
path = []
start = (138.67020413378037,-28.619085361359936)
goal = (135.02317614131,-24.993018165827863)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 399"
path = []
start = (135.02317614131,-24.993018165827863)
goal = (123.60577361905558,-32.270754358016376)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 399"
path = []
start = (123.60577361905558,-32.270754358016376)
goal = (113.91071307750752,-25.55720937406187)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 399"
path = []
start = (113.91071307750752,-25.55720937406187)
goal = (109.56226678230132,-19.107050447582992)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 399"
path = []
start = (109.56226678230132,-19.107050447582992)
goal = (119.88900964351433,-19.751021736799714)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 399"
path = []
start = (119.88900964351433,-19.751021736799714)
goal = (126.67700257513889,2.178988971034329)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 399"
path = []
start = (126.67700257513889,2.178988971034329)
goal = (129.56761743564675,4.020996677972789)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 399"
path = []
start = (129.56761743564675,4.020996677972789)
goal = (132.08494277211742,6.907846081864221)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 399"
path = []
start = (132.08494277211742,6.907846081864221)
goal = (127.09116174797799,9.232517504376688)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 399"
path = []
start = (127.09116174797799,9.232517504376688)
goal = (126.05512181404498,9.496730605426563)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 399"
path = []
start = (126.05512181404498,9.496730605426563)
goal = (124.0984266356852,9.287839600509415)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 399"
path = []
start = (124.0984266356852,9.287839600509415)
goal = (119.88803520713816,22.310708495563347)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 399"
path = []
start = (119.88803520713816,22.310708495563347)
goal = (131.28377856765485,21.566099118592717)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 399"
path = []
start = (131.28377856765485,21.566099118592717)
goal = (138.88946316013784,20.018923145962418)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 399"
path = []
start = (138.88946316013784,20.018923145962418)
goal = (143.56311122016973,29.109878082624306)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 399"
path = []
start = (143.56311122016973,29.109878082624306)
goal = (135.78441984366003,30.391617489238655)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 399"
path = []
start = (135.78441984366003,30.391617489238655)
goal = (125.74201038519277,36.48263497572671)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 399"
path = []
start = (125.74201038519277,36.48263497572671)
goal = (128.30507461675722,51.587705622741254)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 399"
path = []
start = (128.30507461675722,51.587705622741254)
goal = (126.79327078988587,59.960961438477796)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 399"
path = []
start = (126.79327078988587,59.960961438477796)
goal = (131.71347154646674,61.281188503563726)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 399"
path = []
start = (131.71347154646674,61.281188503563726)
goal = (134.24569350595834,66.77340237726901)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 399"
path = []
start = (134.24569350595834,66.77340237726901)
goal = (135.0285274833471,78.85922437959982)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 399"
path = []
start = (135.0285274833471,78.85922437959982)
goal = (126.29287015065472,82.17842945974309)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 399"
path = []
start = (126.29287015065472,82.17842945974309)
goal = (144.28310894419002,86.26611180357963)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 399"
path = []
start = (144.28310894419002,86.26611180357963)
goal = (146.89386197381305,72.3477504118963)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 399"
path = []
start = (146.89386197381305,72.3477504118963)
goal = (154.43241545040325,68.95365325432061)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 399"
path = []
start = (154.43241545040325,68.95365325432061)
goal = (158.7684551050171,75.25108676692957)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 399"
path = []
start = (158.7684551050171,75.25108676692957)
goal = (160.87171621125276,78.77560228441905)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 399"
path = []
start = (160.87171621125276,78.77560228441905)
goal = (157.29151247066238,100.6037933095655)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 399"
path = []
start = (157.29151247066238,100.6037933095655)
goal = (167.38895448207256,102.23524592272526)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 399"
path = []
start = (167.38895448207256,102.23524592272526)
goal = (168.75469852228025,108.88564906326798)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 399"
path = []
start = (168.75469852228025,108.88564906326798)
goal = (175.7473807806354,119.39644957708964)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 399"
path = []
start = (175.7473807806354,119.39644957708964)
goal = (180.59699331722243,122.23767261995289)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 399"
path = []
start = (180.59699331722243,122.23767261995289)
goal = (170.94048824776326,128.55810165655004)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 399"
path = []
start = (170.94048824776326,128.55810165655004)
goal = (157.91798938359705,143.54422953332272)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 399"
path = []
start = (157.91798938359705,143.54422953332272)
goal = (156.04580619097771,154.70197146986425)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 399"
path = []
start = (156.04580619097771,154.70197146986425)
goal = (134.16123453788657,156.6146704949581)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 399"
path = []
start = (134.16123453788657,156.6146704949581)
goal = (125.9080348767738,176.0784038939152)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 399"
path = []
start = (125.9080348767738,176.0784038939152)
goal = (90.12976681931247,180.95767323658254)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 399"
path = []
start = (90.12976681931247,180.95767323658254)
goal = (76.68295170024248,175.65815855562306)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 399"
path = []
start = (76.68295170024248,175.65815855562306)
goal = (64.44601749407724,180.22424838045652)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 399"
path = []
start = (64.44601749407724,180.22424838045652)
goal = (57.07584053792772,188.96169634192597)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 399"
path = []
start = (57.07584053792772,188.96169634192597)
goal = (45.543585070734906,197.2189782460905)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 399"
path = []
start = (45.543585070734906,197.2189782460905)
goal = (25.125179165906985,187.25282244796784)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 399"
path = []
start = (25.125179165906985,187.25282244796784)
goal = (-0.8791642518316678,188.18694004318945)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 399"
path = []
start = (-0.8791642518316678,188.18694004318945)
goal = (9.49065812698828,165.14168973772826)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 399"
path = []
start = (9.49065812698828,165.14168973772826)
goal = (-3.036832224823627,163.00033168412983)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 399"
path = []
start = (-3.036832224823627,163.00033168412983)
goal = (1.0743441702787493,139.7915582797297)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 399"
path = []
start = (1.0743441702787493,139.7915582797297)
goal = (21.915141528765673,134.0729193310367)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 399"
path = []
start = (21.915141528765673,134.0729193310367)
goal = (23.377986397257274,138.28076543077003)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 399"
path = []
start = (23.377986397257274,138.28076543077003)
goal = (21.17831083677126,127.9597675990932)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 399"
path = []
start = (21.17831083677126,127.9597675990932)
goal = (23.795024387025734,115.49749051373561)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 399"
path = []
start = (23.795024387025734,115.49749051373561)
goal = (22.17265575825786,105.37723860199122)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 399"
path = []
start = (22.17265575825786,105.37723860199122)
goal = (-3.6142199246486655,95.1032087405859)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 399"
path = []
start = (-3.6142199246486655,95.1032087405859)
goal = (-16.929252984639845,88.66847728784984)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 399"
path = []
start = (-16.929252984639845,88.66847728784984)
goal = (-28.367748264693574,99.19653806862186)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 399"
path = []
start = (-28.367748264693574,99.19653806862186)
goal = (-33.66152009438795,95.33250299412708)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 399"
path = []
start = (-33.66152009438795,95.33250299412708)
goal = (-26.671840118639295,72.51525186124275)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 399"
path = []
start = (-26.671840118639295,72.51525186124275)
goal = (-44.56233907599355,69.62123428672709)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 399"
path = []
start = (-44.56233907599355,69.62123428672709)
goal = (-60.58566745743724,69.6211391751271)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 399"
path = []
start = (-60.58566745743724,69.6211391751271)
goal = (-60.44814781578137,67.41282458798088)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 399"
path = []
start = (-60.44814781578137,67.41282458798088)
goal = (-69.56367834240771,73.00217247504361)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 399"
path = []
start = (-69.56367834240771,73.00217247504361)
goal = (-73.5749353155398,83.42365434208443)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 399"
path = []
start = (-73.5749353155398,83.42365434208443)
goal = (-79.22254273608891,82.60874767764534)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 399"
path = []
start = (-79.22254273608891,82.60874767764534)
goal = (-84.03322024582339,86.17148401427016)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 399"
path = []
start = (-84.03322024582339,86.17148401427016)
goal = (-89.56159711654874,82.30324746739899)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 399"
path = []
start = (-89.56159711654874,82.30324746739899)
goal = (-99.67594090114562,90.10973872041814)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 399"
path = []
start = (-99.67594090114562,90.10973872041814)
goal = (-115.84740482720686,87.97849023965011)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 399"
path = []
start = (-115.84740482720686,87.97849023965011)
goal = (-115.38397659809227,69.90974137704325)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 399"
path = []
start = (-115.38397659809227,69.90974137704325)
goal = (-103.82730020959916,66.50643440103971)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 399"
path = []
start = (-103.82730020959916,66.50643440103971)
goal = (-91.72306540505217,50.617703857642)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 399"
path = []
start = (-91.72306540505217,50.617703857642)
goal = (-93.26721553884644,42.28082595224083)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 399"
path = []
start = (-93.26721553884644,42.28082595224083)
goal = (-76.3753189281704,42.91781650150003)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 399"
path = []
start = (-76.3753189281704,42.91781650150003)
goal = (-95.230825706612,25.481739118799908)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 399"
path = []
start = (-95.230825706612,25.481739118799908)
goal = (-83.98701860948066,17.072349269398188)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 399"
path = []
start = (-83.98701860948066,17.072349269398188)
goal = (-60.58146320464104,22.381816153635583)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 399"
path = []
start = (-60.58146320464104,22.381816153635583)
goal = (-49.91736290088335,19.211999401044665)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 399"
path = []
start = (-49.91736290088335,19.211999401044665)
goal = (-27.1149716261788,5.310734087378535)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 399"
path = []
start = (-27.1149716261788,5.310734087378535)
goal = (-38.8483301436641,-13.306341140016599)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 124 of 399"
path = []
start = (-38.8483301436641,-13.306341140016599)
goal = (-63.14181603813057,-17.264624833971766)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 125 of 399"
path = []
start = (-63.14181603813057,-17.264624833971766)
goal = (-68.80383704318697,-28.705301225375564)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 126 of 399"
path = []
start = (-68.80383704318697,-28.705301225375564)
goal = (-73.79664773153154,-33.099835026884875)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 127 of 399"
path = []
start = (-73.79664773153154,-33.099835026884875)
goal = (-76.0186922877121,-37.16352003454281)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 128 of 399"
path = []
start = (-76.0186922877121,-37.16352003454281)
goal = (-67.9956547765411,-42.712019248818876)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 129 of 399"
path = []
start = (-67.9956547765411,-42.712019248818876)
goal = (-63.090742681458124,-35.48309682764898)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 130 of 399"
path = []
start = (-63.090742681458124,-35.48309682764898)
goal = (-63.72424967532817,-51.08923030700069)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 131 of 399"
path = []
start = (-63.72424967532817,-51.08923030700069)
goal = (-73.86518361760844,-69.90925220459641)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 132 of 399"
path = []
start = (-73.86518361760844,-69.90925220459641)
goal = (-86.7803323542566,-67.47126250971891)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 133 of 399"
path = []
start = (-86.7803323542566,-67.47126250971891)
goal = (-92.50667190653981,-79.25602605991058)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 134 of 399"
path = []
start = (-92.50667190653981,-79.25602605991058)
goal = (-90.85686046969452,-82.52487446176504)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 135 of 399"
path = []
start = (-90.85686046969452,-82.52487446176504)
goal = (-92.96555895540175,-108.037357635347)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 136 of 399"
path = []
start = (-92.96555895540175,-108.037357635347)
goal = (-84.82232219442723,-109.27470189988276)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 137 of 399"
path = []
start = (-84.82232219442723,-109.27470189988276)
goal = (-83.68733138276774,-120.95303266669326)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 138 of 399"
path = []
start = (-83.68733138276774,-120.95303266669326)
goal = (-76.45227863449749,-122.77752212076672)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 139 of 399"
path = []
start = (-76.45227863449749,-122.77752212076672)
goal = (-73.13430313038354,-138.69582866614223)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 140 of 399"
path = []
start = (-73.13430313038354,-138.69582866614223)
goal = (-60.855658076445536,-137.55692370504636)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 141 of 399"
path = []
start = (-60.855658076445536,-137.55692370504636)
goal = (-56.43051440545807,-143.22080006532175)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 142 of 399"
path = []
start = (-56.43051440545807,-143.22080006532175)
goal = (-55.27854695413507,-149.41179892346167)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 143 of 399"
path = []
start = (-55.27854695413507,-149.41179892346167)
goal = (-41.055695700550075,-149.3381113465938)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 144 of 399"
path = []
start = (-41.055695700550075,-149.3381113465938)
goal = (-45.751354447310035,-160.36957349229414)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 145 of 399"
path = []
start = (-45.751354447310035,-160.36957349229414)
goal = (-50.10365359036061,-164.39838386811786)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 146 of 399"
path = []
start = (-50.10365359036061,-164.39838386811786)
goal = (-44.04608909846749,-167.61110905315462)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 147 of 399"
path = []
start = (-44.04608909846749,-167.61110905315462)
goal = (-35.20475981841366,-184.52993741076438)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 148 of 399"
path = []
start = (-35.20475981841366,-184.52993741076438)
goal = (-57.96994740148236,-196.11048396670668)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 149 of 399"
path = []
start = (-57.96994740148236,-196.11048396670668)
goal = (-87.65575259945871,-178.3680830060588)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 150 of 399"
path = []
start = (-87.65575259945871,-178.3680830060588)
goal = (-89.61381276909952,-172.26894035378413)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 151 of 399"
path = []
start = (-89.61381276909952,-172.26894035378413)
goal = (-106.71111525048946,-172.43221346666462)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 152 of 399"
path = []
start = (-106.71111525048946,-172.43221346666462)
goal = (-115.80879991761196,-185.06417475701315)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 153 of 399"
path = []
start = (-115.80879991761196,-185.06417475701315)
goal = (-127.8303143358082,-188.70327120451543)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 154 of 399"
path = []
start = (-127.8303143358082,-188.70327120451543)
goal = (-117.78772922499483,-149.79865740425362)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 155 of 399"
path = []
start = (-117.78772922499483,-149.79865740425362)
goal = (-102.61947674108418,-146.51942759932342)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 156 of 399"
path = []
start = (-102.61947674108418,-146.51942759932342)
goal = (-99.76292266682898,-140.13510680524806)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 157 of 399"
path = []
start = (-99.76292266682898,-140.13510680524806)
goal = (-102.78988310116466,-127.95894885434964)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 158 of 399"
path = []
start = (-102.78988310116466,-127.95894885434964)
goal = (-86.72751009330742,-136.51213601848394)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 159 of 399"
path = []
start = (-86.72751009330742,-136.51213601848394)
goal = (-86.36436330072192,-152.04325481188857)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 160 of 399"
path = []
start = (-86.36436330072192,-152.04325481188857)
goal = (-122.41328556598616,-127.95615271868512)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 161 of 399"
path = []
start = (-122.41328556598616,-127.95615271868512)
goal = (-110.60461798137382,-107.10099355011221)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 162 of 399"
path = []
start = (-110.60461798137382,-107.10099355011221)
goal = (-118.47053419022112,-88.77677193185968)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 163 of 399"
path = []
start = (-118.47053419022112,-88.77677193185968)
goal = (-118.42437913571997,-79.18936559061534)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 164 of 399"
path = []
start = (-118.42437913571997,-79.18936559061534)
goal = (-123.34238049549255,-72.52427489737134)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 165 of 399"
path = []
start = (-123.34238049549255,-72.52427489737134)
goal = (-150.18808606652794,-67.4255250849721)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 166 of 399"
path = []
start = (-150.18808606652794,-67.4255250849721)
goal = (-158.7061053904192,-70.1577177983049)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 167 of 399"
path = []
start = (-158.7061053904192,-70.1577177983049)
goal = (-163.06215802061428,-80.50313912183364)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 168 of 399"
path = []
start = (-163.06215802061428,-80.50313912183364)
goal = (-161.1887711115714,-87.41044396788284)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 169 of 399"
path = []
start = (-161.1887711115714,-87.41044396788284)
goal = (-161.81329938733137,-98.05694122867395)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 170 of 399"
path = []
start = (-161.81329938733137,-98.05694122867395)
goal = (-164.34018080410482,-100.97988956708797)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 171 of 399"
path = []
start = (-164.34018080410482,-100.97988956708797)
goal = (-144.68402554516885,-99.92749423890075)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 172 of 399"
path = []
start = (-144.68402554516885,-99.92749423890075)
goal = (-152.09010578933243,-81.14813775154927)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 173 of 399"
path = []
start = (-152.09010578933243,-81.14813775154927)
goal = (-184.83100841068548,-83.62097645757368)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 174 of 399"
path = []
start = (-184.83100841068548,-83.62097645757368)
goal = (-189.38984061907325,-93.93631431125682)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 175 of 399"
path = []
start = (-189.38984061907325,-93.93631431125682)
goal = (-193.86838424543797,-97.21410408873679)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 176 of 399"
path = []
start = (-193.86838424543797,-97.21410408873679)
goal = (-172.13535860178987,-131.24518286878973)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 177 of 399"
path = []
start = (-172.13535860178987,-131.24518286878973)
goal = (-170.34419941272154,-190.847630934433)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 178 of 399"
path = []
start = (-170.34419941272154,-190.847630934433)
goal = (-186.02306019696204,-192.99876872604952)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 179 of 399"
path = []
start = (-186.02306019696204,-192.99876872604952)
goal = (-48.65176212717418,-129.45319487198836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 180 of 399"
path = []
start = (-48.65176212717418,-129.45319487198836)
goal = (-46.87975939877842,-131.56836750387083)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 181 of 399"
path = []
start = (-46.87975939877842,-131.56836750387083)
goal = (-19.57203553147471,-134.35531634040876)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 182 of 399"
path = []
start = (-19.57203553147471,-134.35531634040876)
goal = (-20.819842840168207,-143.02004743445974)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 183 of 399"
path = []
start = (-20.819842840168207,-143.02004743445974)
goal = (-3.0287506614716335,-146.7353372968854)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 184 of 399"
path = []
start = (-3.0287506614716335,-146.7353372968854)
goal = (4.590585118497216,-144.66109546710047)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 185 of 399"
path = []
start = (4.590585118497216,-144.66109546710047)
goal = (9.434363971877474,-148.7386411061956)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 186 of 399"
path = []
start = (9.434363971877474,-148.7386411061956)
goal = (14.211422140701956,-151.34706323846743)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 187 of 399"
path = []
start = (14.211422140701956,-151.34706323846743)
goal = (17.118303771088307,-136.30458088710893)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 188 of 399"
path = []
start = (17.118303771088307,-136.30458088710893)
goal = (13.924907777488158,-127.87550954853914)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 189 of 399"
path = []
start = (13.924907777488158,-127.87550954853914)
goal = (14.959587187223576,-127.09484230065975)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 190 of 399"
path = []
start = (14.959587187223576,-127.09484230065975)
goal = (22.245780656914604,-121.89998950561782)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 191 of 399"
path = []
start = (22.245780656914604,-121.89998950561782)
goal = (20.40960017395176,-111.14486436258724)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 192 of 399"
path = []
start = (20.40960017395176,-111.14486436258724)
goal = (26.41856562470778,-102.7478745244768)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 193 of 399"
path = []
start = (26.41856562470778,-102.7478745244768)
goal = (14.275132896530835,-102.58060053042568)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 194 of 399"
path = []
start = (14.275132896530835,-102.58060053042568)
goal = (1.7988086853734728,-113.88931262788647)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 195 of 399"
path = []
start = (1.7988086853734728,-113.88931262788647)
goal = (-20.86792069581702,-101.97362789463975)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 196 of 399"
path = []
start = (-20.86792069581702,-101.97362789463975)
goal = (-25.520403040086308,-101.85972556520588)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 197 of 399"
path = []
start = (-25.520403040086308,-101.85972556520588)
goal = (-1.742191210631404,-159.46761751354148)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 198 of 399"
path = []
start = (-1.742191210631404,-159.46761751354148)
goal = (16.530215611830755,-167.63684308211094)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 199 of 399"
path = []
start = (16.530215611830755,-167.63684308211094)
goal = (21.23334725024199,-172.72784210421284)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 200 of 399"
path = []
start = (21.23334725024199,-172.72784210421284)
goal = (17.51154445400448,-180.39192271131236)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 201 of 399"
path = []
start = (17.51154445400448,-180.39192271131236)
goal = (47.34298078815519,-196.9523635142317)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 202 of 399"
path = []
start = (47.34298078815519,-196.9523635142317)
goal = (66.15897447984844,-170.7567520885686)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 203 of 399"
path = []
start = (66.15897447984844,-170.7567520885686)
goal = (72.95724811574081,-173.44031660565292)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 204 of 399"
path = []
start = (72.95724811574081,-173.44031660565292)
goal = (57.94006704799716,-164.32342358136162)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 205 of 399"
path = []
start = (57.94006704799716,-164.32342358136162)
goal = (40.56713883869486,-154.0396903542106)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 206 of 399"
path = []
start = (40.56713883869486,-154.0396903542106)
goal = (51.26701521122854,-134.70763182096226)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 207 of 399"
path = []
start = (51.26701521122854,-134.70763182096226)
goal = (53.03710687414829,-134.28265787450258)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 208 of 399"
path = []
start = (53.03710687414829,-134.28265787450258)
goal = (53.318242354747525,-135.6667056888386)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 209 of 399"
path = []
start = (53.318242354747525,-135.6667056888386)
goal = (63.34483223411604,-130.85291854962736)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 210 of 399"
path = []
start = (63.34483223411604,-130.85291854962736)
goal = (68.68476923008006,-123.51576082533641)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 211 of 399"
path = []
start = (68.68476923008006,-123.51576082533641)
goal = (60.66029271507887,-119.35115748015552)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 212 of 399"
path = []
start = (60.66029271507887,-119.35115748015552)
goal = (42.027794408444805,-102.60204840610996)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 213 of 399"
path = []
start = (42.027794408444805,-102.60204840610996)
goal = (30.336314540130843,-132.1230931122793)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 214 of 399"
path = []
start = (30.336314540130843,-132.1230931122793)
goal = (80.73833360902063,-157.11744319846878)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 215 of 399"
path = []
start = (80.73833360902063,-157.11744319846878)
goal = (99.03345550101886,-146.14778850854933)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 216 of 399"
path = []
start = (99.03345550101886,-146.14778850854933)
goal = (124.11156352707798,-135.08787203196744)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 217 of 399"
path = []
start = (124.11156352707798,-135.08787203196744)
goal = (133.05723968011648,-147.54402502897088)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 218 of 399"
path = []
start = (133.05723968011648,-147.54402502897088)
goal = (139.89746533839195,-142.01942663279107)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 219 of 399"
path = []
start = (139.89746533839195,-142.01942663279107)
goal = (148.81077851863188,-135.43446198659024)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 220 of 399"
path = []
start = (148.81077851863188,-135.43446198659024)
goal = (151.96539227146036,-119.06458483580131)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 221 of 399"
path = []
start = (151.96539227146036,-119.06458483580131)
goal = (158.97106135704695,-120.28584681544503)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 222 of 399"
path = []
start = (158.97106135704695,-120.28584681544503)
goal = (163.38762557243479,-113.13869454352314)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 223 of 399"
path = []
start = (163.38762557243479,-113.13869454352314)
goal = (184.59104408800744,-128.61291570619045)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 224 of 399"
path = []
start = (184.59104408800744,-128.61291570619045)
goal = (192.10439588811892,-139.2340830288017)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 225 of 399"
path = []
start = (192.10439588811892,-139.2340830288017)
goal = (193.52012980528303,-152.5636758076991)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 226 of 399"
path = []
start = (193.52012980528303,-152.5636758076991)
goal = (182.4599714901076,-157.47132897086965)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 227 of 399"
path = []
start = (182.4599714901076,-157.47132897086965)
goal = (172.81984149992599,-162.01813016411518)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 228 of 399"
path = []
start = (172.81984149992599,-162.01813016411518)
goal = (167.27485429136004,-161.1166635539668)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 229 of 399"
path = []
start = (167.27485429136004,-161.1166635539668)
goal = (150.74731960392523,-162.4767305653973)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 230 of 399"
path = []
start = (150.74731960392523,-162.4767305653973)
goal = (154.19715352046455,-176.9403545219042)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 231 of 399"
path = []
start = (154.19715352046455,-176.9403545219042)
goal = (133.1286460148283,-186.48363286611982)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 232 of 399"
path = []
start = (133.1286460148283,-186.48363286611982)
goal = (132.78246784642351,-190.26011729616962)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 233 of 399"
path = []
start = (132.78246784642351,-190.26011729616962)
goal = (128.76469965385752,-186.947096286443)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 234 of 399"
path = []
start = (128.76469965385752,-186.947096286443)
goal = (126.5607074762433,-162.02412864509836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 235 of 399"
path = []
start = (126.5607074762433,-162.02412864509836)
goal = (95.86975345622244,-117.01588469123449)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 236 of 399"
path = []
start = (95.86975345622244,-117.01588469123449)
goal = (86.12216983667491,-114.39306160845119)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 237 of 399"
path = []
start = (86.12216983667491,-114.39306160845119)
goal = (78.85976666363007,-84.86635313263022)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 238 of 399"
path = []
start = (78.85976666363007,-84.86635313263022)
goal = (79.08801174480777,-77.25876932003813)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 239 of 399"
path = []
start = (79.08801174480777,-77.25876932003813)
goal = (71.80983583461204,-72.88452937690116)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 240 of 399"
path = []
start = (71.80983583461204,-72.88452937690116)
goal = (83.84419744275414,-65.66862170629389)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 241 of 399"
path = []
start = (83.84419744275414,-65.66862170629389)
goal = (97.7146026766772,-79.84319728545763)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 242 of 399"
path = []
start = (97.7146026766772,-79.84319728545763)
goal = (134.80237210012916,-106.08510030980023)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 243 of 399"
path = []
start = (134.80237210012916,-106.08510030980023)
goal = (149.68553013133732,-78.07827812740054)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 244 of 399"
path = []
start = (149.68553013133732,-78.07827812740054)
goal = (146.14847452129254,-63.85436629165707)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 245 of 399"
path = []
start = (146.14847452129254,-63.85436629165707)
goal = (172.11362108883696,-59.60149207112107)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 246 of 399"
path = []
start = (172.11362108883696,-59.60149207112107)
goal = (180.59402127417303,-61.53660963620317)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 247 of 399"
path = []
start = (180.59402127417303,-61.53660963620317)
goal = (171.48911738261148,-47.463486909498386)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 248 of 399"
path = []
start = (171.48911738261148,-47.463486909498386)
goal = (158.20850356757916,-39.82821227854393)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 249 of 399"
path = []
start = (158.20850356757916,-39.82821227854393)
goal = (152.5621843304532,-36.64349787702656)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 250 of 399"
path = []
start = (152.5621843304532,-36.64349787702656)
goal = (164.02269286137982,-28.54253604443801)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 251 of 399"
path = []
start = (164.02269286137982,-28.54253604443801)
goal = (177.47780486395533,-23.145837614162815)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 252 of 399"
path = []
start = (177.47780486395533,-23.145837614162815)
goal = (175.58795663067974,4.001260290121081)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 253 of 399"
path = []
start = (175.58795663067974,4.001260290121081)
goal = (174.02652284306924,11.924530734953777)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 254 of 399"
path = []
start = (174.02652284306924,11.924530734953777)
goal = (164.6593191781347,11.393902501447371)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 255 of 399"
path = []
start = (164.6593191781347,11.393902501447371)
goal = (153.01242502912487,9.506392291031943)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 256 of 399"
path = []
start = (153.01242502912487,9.506392291031943)
goal = (142.44458201298357,-4.14763999408018)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 257 of 399"
path = []
start = (142.44458201298357,-4.14763999408018)
goal = (139.80568515764747,-6.632263328310643)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 258 of 399"
path = []
start = (139.80568515764747,-6.632263328310643)
goal = (164.33557941493524,30.86523338130104)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 259 of 399"
path = []
start = (164.33557941493524,30.86523338130104)
goal = (184.67505894052914,25.695140743987736)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 260 of 399"
path = []
start = (184.67505894052914,25.695140743987736)
goal = (192.88168301708924,27.034844304625466)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 261 of 399"
path = []
start = (192.88168301708924,27.034844304625466)
goal = (192.1277835671999,12.759467776821651)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 262 of 399"
path = []
start = (192.1277835671999,12.759467776821651)
goal = (198.67108359390153,61.09409787730203)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 263 of 399"
path = []
start = (198.67108359390153,61.09409787730203)
goal = (186.10321844801695,89.6872577731923)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 264 of 399"
path = []
start = (186.10321844801695,89.6872577731923)
goal = (198.65961098170976,126.22406491184432)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 265 of 399"
path = []
start = (198.65961098170976,126.22406491184432)
goal = (195.6513083621403,132.81218054254964)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 266 of 399"
path = []
start = (195.6513083621403,132.81218054254964)
goal = (197.78160460743652,139.86934847811955)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 267 of 399"
path = []
start = (197.78160460743652,139.86934847811955)
goal = (184.68589104049585,153.8619570383894)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 268 of 399"
path = []
start = (184.68589104049585,153.8619570383894)
goal = (188.45373872224667,163.52966189043576)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 269 of 399"
path = []
start = (188.45373872224667,163.52966189043576)
goal = (192.109302272849,188.84194032774786)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 270 of 399"
path = []
start = (192.109302272849,188.84194032774786)
goal = (173.46595531966364,196.6641441712851)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 271 of 399"
path = []
start = (173.46595531966364,196.6641441712851)
goal = (142.9893672554408,119.32896934070754)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 272 of 399"
path = []
start = (142.9893672554408,119.32896934070754)
goal = (127.58128004148085,125.23005452461035)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 273 of 399"
path = []
start = (127.58128004148085,125.23005452461035)
goal = (127.44738396671187,110.80825047277523)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 274 of 399"
path = []
start = (127.44738396671187,110.80825047277523)
goal = (103.17309326966415,85.88358051605599)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 275 of 399"
path = []
start = (103.17309326966415,85.88358051605599)
goal = (102.94247242289913,85.85292458519189)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 276 of 399"
path = []
start = (102.94247242289913,85.85292458519189)
goal = (94.39074539217421,85.63955592733146)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 277 of 399"
path = []
start = (94.39074539217421,85.63955592733146)
goal = (87.99337112710003,67.29121454735753)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 278 of 399"
path = []
start = (87.99337112710003,67.29121454735753)
goal = (98.53963265617517,62.12037410768846)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 279 of 399"
path = []
start = (98.53963265617517,62.12037410768846)
goal = (115.07654015754082,64.64217589749666)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 280 of 399"
path = []
start = (115.07654015754082,64.64217589749666)
goal = (105.90158683749235,46.30821693373056)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 281 of 399"
path = []
start = (105.90158683749235,46.30821693373056)
goal = (89.87060690444008,40.058320157918956)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 282 of 399"
path = []
start = (89.87060690444008,40.058320157918956)
goal = (83.98636811611789,53.50947974861566)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 283 of 399"
path = []
start = (83.98636811611789,53.50947974861566)
goal = (74.12034147272112,34.268274987811765)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 284 of 399"
path = []
start = (74.12034147272112,34.268274987811765)
goal = (87.39023492759077,13.912222994884047)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 285 of 399"
path = []
start = (87.39023492759077,13.912222994884047)
goal = (93.28080982344181,14.905186471348657)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 286 of 399"
path = []
start = (93.28080982344181,14.905186471348657)
goal = (43.08054642125012,30.69934637844193)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 287 of 399"
path = []
start = (43.08054642125012,30.69934637844193)
goal = (46.78560842215575,46.48447771188614)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 288 of 399"
path = []
start = (46.78560842215575,46.48447771188614)
goal = (39.83950623854673,59.52266213226147)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 289 of 399"
path = []
start = (39.83950623854673,59.52266213226147)
goal = (29.925407471064375,53.93465139977772)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 290 of 399"
path = []
start = (29.925407471064375,53.93465139977772)
goal = (30.76099523838164,42.79965210343158)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 291 of 399"
path = []
start = (30.76099523838164,42.79965210343158)
goal = (26.20436197282237,41.4183924950832)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 292 of 399"
path = []
start = (26.20436197282237,41.4183924950832)
goal = (25.757640061629246,27.285260598419967)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 293 of 399"
path = []
start = (25.757640061629246,27.285260598419967)
goal = (25.481180168804315,21.79981489694603)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 294 of 399"
path = []
start = (25.481180168804315,21.79981489694603)
goal = (11.884787905342677,8.218438764755888)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 295 of 399"
path = []
start = (11.884787905342677,8.218438764755888)
goal = (5.67085728440756,12.277800487954892)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 296 of 399"
path = []
start = (5.67085728440756,12.277800487954892)
goal = (-5.805030536565255,12.943825303314327)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 297 of 399"
path = []
start = (-5.805030536565255,12.943825303314327)
goal = (4.594085268623388,-1.1714267444950224)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 298 of 399"
path = []
start = (4.594085268623388,-1.1714267444950224)
goal = (7.390065227527913,-6.027142315073519)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 299 of 399"
path = []
start = (7.390065227527913,-6.027142315073519)
goal = (16.523718996680714,75.8957198849368)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 300 of 399"
path = []
start = (16.523718996680714,75.8957198849368)
goal = (50.19164961630972,99.71815301396907)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 301 of 399"
path = []
start = (50.19164961630972,99.71815301396907)
goal = (53.24849993556006,112.31316426076143)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 302 of 399"
path = []
start = (53.24849993556006,112.31316426076143)
goal = (62.128724249655875,112.86601396035775)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 303 of 399"
path = []
start = (62.128724249655875,112.86601396035775)
goal = (56.640099695384606,127.81353496612809)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 304 of 399"
path = []
start = (56.640099695384606,127.81353496612809)
goal = (76.64267415037818,133.8085309104207)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 305 of 399"
path = []
start = (76.64267415037818,133.8085309104207)
goal = (69.32868120178318,145.24981339496543)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 306 of 399"
path = []
start = (69.32868120178318,145.24981339496543)
goal = (64.48871850083424,161.78999757858998)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 307 of 399"
path = []
start = (64.48871850083424,161.78999757858998)
goal = (54.44834813632332,159.680400479684)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 308 of 399"
path = []
start = (54.44834813632332,159.680400479684)
goal = (44.361648726125566,165.13788269937106)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 309 of 399"
path = []
start = (44.361648726125566,165.13788269937106)
goal = (54.30109780363321,174.63520783070948)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 310 of 399"
path = []
start = (54.30109780363321,174.63520783070948)
goal = (86.21324192418479,195.03029144190197)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 311 of 399"
path = []
start = (86.21324192418479,195.03029144190197)
goal = (91.29807298045716,134.45572714213807)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 312 of 399"
path = []
start = (91.29807298045716,134.45572714213807)
goal = (80.78149657650181,109.70166316119577)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 313 of 399"
path = []
start = (80.78149657650181,109.70166316119577)
goal = (-14.68752023891443,120.9973417006072)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 314 of 399"
path = []
start = (-14.68752023891443,120.9973417006072)
goal = (-53.529580993264545,135.82667397667024)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 315 of 399"
path = []
start = (-53.529580993264545,135.82667397667024)
goal = (-55.47538011631258,140.87620431731074)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 316 of 399"
path = []
start = (-55.47538011631258,140.87620431731074)
goal = (-69.86708187200043,137.925768455911)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 317 of 399"
path = []
start = (-69.86708187200043,137.925768455911)
goal = (-71.5162954002048,142.26976342594958)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 318 of 399"
path = []
start = (-71.5162954002048,142.26976342594958)
goal = (-89.66460785948067,148.8780246827788)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 319 of 399"
path = []
start = (-89.66460785948067,148.8780246827788)
goal = (-100.41973874871894,145.03896761517996)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 320 of 399"
path = []
start = (-100.41973874871894,145.03896761517996)
goal = (-113.71732699589975,138.90629424191798)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 321 of 399"
path = []
start = (-113.71732699589975,138.90629424191798)
goal = (-127.59324395388533,131.96277292571529)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 322 of 399"
path = []
start = (-127.59324395388533,131.96277292571529)
goal = (-116.1439922321895,123.52177630021305)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 323 of 399"
path = []
start = (-116.1439922321895,123.52177630021305)
goal = (-125.9945310197479,103.54158340123575)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 324 of 399"
path = []
start = (-125.9945310197479,103.54158340123575)
goal = (-156.55932702724465,113.59881055314628)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 325 of 399"
path = []
start = (-156.55932702724465,113.59881055314628)
goal = (-155.29994445832438,123.15210374555068)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 326 of 399"
path = []
start = (-155.29994445832438,123.15210374555068)
goal = (-170.75145262702299,136.99953970837737)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 327 of 399"
path = []
start = (-170.75145262702299,136.99953970837737)
goal = (-177.4056251797532,150.7444429581023)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 328 of 399"
path = []
start = (-177.4056251797532,150.7444429581023)
goal = (-182.664305515401,161.21727969463302)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 329 of 399"
path = []
start = (-182.664305515401,161.21727969463302)
goal = (-199.33947353393452,158.05417075667123)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 330 of 399"
path = []
start = (-199.33947353393452,158.05417075667123)
goal = (-187.75302465016614,137.3288708318491)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 331 of 399"
path = []
start = (-187.75302465016614,137.3288708318491)
goal = (-194.62854223867566,105.0161966815615)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 332 of 399"
path = []
start = (-194.62854223867566,105.0161966815615)
goal = (-194.84711827146714,98.48525158488872)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 333 of 399"
path = []
start = (-194.84711827146714,98.48525158488872)
goal = (-175.08876468546907,80.65304160753249)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 334 of 399"
path = []
start = (-175.08876468546907,80.65304160753249)
goal = (-157.83754005304718,84.05017278096057)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 335 of 399"
path = []
start = (-157.83754005304718,84.05017278096057)
goal = (-147.4909646127095,58.08394664738108)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 336 of 399"
path = []
start = (-147.4909646127095,58.08394664738108)
goal = (-140.70077198954056,59.74393276967095)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 337 of 399"
path = []
start = (-140.70077198954056,59.74393276967095)
goal = (-131.57793372964974,68.22170529784569)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 338 of 399"
path = []
start = (-131.57793372964974,68.22170529784569)
goal = (-115.54330670608302,50.27555127124592)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 339 of 399"
path = []
start = (-115.54330670608302,50.27555127124592)
goal = (-122.01737543429974,40.22749312800423)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 340 of 399"
path = []
start = (-122.01737543429974,40.22749312800423)
goal = (-117.21570184388956,32.30813933941019)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 341 of 399"
path = []
start = (-117.21570184388956,32.30813933941019)
goal = (-124.5040049301779,29.706234789870052)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 342 of 399"
path = []
start = (-124.5040049301779,29.706234789870052)
goal = (-127.15557719475048,24.268526831164735)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 343 of 399"
path = []
start = (-127.15557719475048,24.268526831164735)
goal = (-126.01937942333822,1.8202750229209528)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 344 of 399"
path = []
start = (-126.01937942333822,1.8202750229209528)
goal = (-121.68638601255464,-6.806480698559568)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 345 of 399"
path = []
start = (-121.68638601255464,-6.806480698559568)
goal = (-126.45445117893162,-8.803955973549535)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 346 of 399"
path = []
start = (-126.45445117893162,-8.803955973549535)
goal = (-128.3052679925711,-22.484769735577544)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 347 of 399"
path = []
start = (-128.3052679925711,-22.484769735577544)
goal = (-135.372595756506,-33.348425835076256)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 348 of 399"
path = []
start = (-135.372595756506,-33.348425835076256)
goal = (-142.44615711735645,-30.294469720159924)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 349 of 399"
path = []
start = (-142.44615711735645,-30.294469720159924)
goal = (-146.84448457021006,-32.124993663863535)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 350 of 399"
path = []
start = (-146.84448457021006,-32.124993663863535)
goal = (-146.59049204534364,-25.3810863909857)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 351 of 399"
path = []
start = (-146.59049204534364,-25.3810863909857)
goal = (-152.71727658961186,-26.03937697894554)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 352 of 399"
path = []
start = (-152.71727658961186,-26.03937697894554)
goal = (-153.49844726455007,-27.65552023576467)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 353 of 399"
path = []
start = (-153.49844726455007,-27.65552023576467)
goal = (-158.58760464431563,-27.431037705673134)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 354 of 399"
path = []
start = (-158.58760464431563,-27.431037705673134)
goal = (-158.60408809578095,-15.713853965535577)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 355 of 399"
path = []
start = (-158.60408809578095,-15.713853965535577)
goal = (-158.28384943439224,-5.613030082274264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 356 of 399"
path = []
start = (-158.28384943439224,-5.613030082274264)
goal = (-164.13728028842092,1.498163428264263)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 357 of 399"
path = []
start = (-164.13728028842092,1.498163428264263)
goal = (-164.47223075559054,5.708369465196824)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 358 of 399"
path = []
start = (-164.47223075559054,5.708369465196824)
goal = (-159.14916108304124,10.804622461629265)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 359 of 399"
path = []
start = (-159.14916108304124,10.804622461629265)
goal = (-164.22726384715406,32.74431625964479)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 360 of 399"
path = []
start = (-164.22726384715406,32.74431625964479)
goal = (-154.11365888182505,35.63470140603789)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 361 of 399"
path = []
start = (-154.11365888182505,35.63470140603789)
goal = (-176.52556602462505,33.746826547941026)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 362 of 399"
path = []
start = (-176.52556602462505,33.746826547941026)
goal = (-188.43648619129362,31.605851175719295)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 363 of 399"
path = []
start = (-188.43648619129362,31.605851175719295)
goal = (-190.56358360315494,53.810115555728714)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 364 of 399"
path = []
start = (-190.56358360315494,53.810115555728714)
goal = (-188.7211106321568,55.53868510498984)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 365 of 399"
path = []
start = (-188.7211106321568,55.53868510498984)
goal = (-193.3993583441474,63.19999031868247)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 366 of 399"
path = []
start = (-193.3993583441474,63.19999031868247)
goal = (-188.88143748229166,-3.6266901174916484)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 367 of 399"
path = []
start = (-188.88143748229166,-3.6266901174916484)
goal = (-189.10328315068998,-8.826743951886357)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 368 of 399"
path = []
start = (-189.10328315068998,-8.826743951886357)
goal = (-176.66907565068036,-14.88758912778323)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 369 of 399"
path = []
start = (-176.66907565068036,-14.88758912778323)
goal = (-166.26927135926798,-37.83441643441509)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 370 of 399"
path = []
start = (-166.26927135926798,-37.83441643441509)
goal = (-193.0917168563525,-39.53461105518534)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 371 of 399"
path = []
start = (-193.0917168563525,-39.53461105518534)
goal = (-197.67123181952093,-30.014097416768863)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 372 of 399"
path = []
start = (-197.67123181952093,-30.014097416768863)
goal = (-110.95668477287708,5.797206468695435)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 373 of 399"
path = []
start = (-110.95668477287708,5.797206468695435)
goal = (-97.44870929358753,-23.96440208871701)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 374 of 399"
path = []
start = (-97.44870929358753,-23.96440208871701)
goal = (-102.86046887735347,-42.90200297930576)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 375 of 399"
path = []
start = (-102.86046887735347,-42.90200297930576)
goal = (-79.94004285135885,95.20881460235529)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 376 of 399"
path = []
start = (-79.94004285135885,95.20881460235529)
goal = (-76.7318937285547,97.62015832139491)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 377 of 399"
path = []
start = (-76.7318937285547,97.62015832139491)
goal = (-76.00289479117887,116.22548168396071)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 378 of 399"
path = []
start = (-76.00289479117887,116.22548168396071)
goal = (-90.80436360547365,131.7563047327916)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 379 of 399"
path = []
start = (-90.80436360547365,131.7563047327916)
goal = (-98.58826537587366,162.05050186411978)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 380 of 399"
path = []
start = (-98.58826537587366,162.05050186411978)
goal = (-110.75778821787176,156.20835497311725)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 381 of 399"
path = []
start = (-110.75778821787176,156.20835497311725)
goal = (-112.54963255824939,163.46506253504577)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 382 of 399"
path = []
start = (-112.54963255824939,163.46506253504577)
goal = (-112.08491528931197,167.60771095543754)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 383 of 399"
path = []
start = (-112.08491528931197,167.60771095543754)
goal = (-105.95851611548666,178.19031861780917)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 384 of 399"
path = []
start = (-105.95851611548666,178.19031861780917)
goal = (-125.1736214955196,171.8170558535232)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 385 of 399"
path = []
start = (-125.1736214955196,171.8170558535232)
goal = (-149.05371997023647,160.3487374224369)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 386 of 399"
path = []
start = (-149.05371997023647,160.3487374224369)
goal = (-151.5970259302144,174.53273674379972)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 387 of 399"
path = []
start = (-151.5970259302144,174.53273674379972)
goal = (-154.00294553436598,179.14610916165572)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 388 of 399"
path = []
start = (-154.00294553436598,179.14610916165572)
goal = (-150.7881251760274,182.7376486857611)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 389 of 399"
path = []
start = (-150.7881251760274,182.7376486857611)
goal = (-170.8907063783985,199.27964743256211)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 390 of 399"
path = []
start = (-170.8907063783985,199.27964743256211)
goal = (-175.90621053708614,196.56874692878228)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 391 of 399"
path = []
start = (-175.90621053708614,196.56874692878228)
goal = (-193.44665371145382,182.25282271581278)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 392 of 399"
path = []
start = (-193.44665371145382,182.25282271581278)
goal = (-76.60108135587267,165.6903229745302)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 393 of 399"
path = []
start = (-76.60108135587267,165.6903229745302)
goal = (-69.2740541752307,166.6264578303331)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 394 of 399"
path = []
start = (-69.2740541752307,166.6264578303331)
goal = (-58.457065626505425,170.46722679338444)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 395 of 399"
path = []
start = (-58.457065626505425,170.46722679338444)
goal = (-41.420333081517924,182.49584952762257)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 396 of 399"
path = []
start = (-41.420333081517924,182.49584952762257)
goal = (-29.594991615555955,183.37719178119988)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 397 of 399"
path = []
start = (-29.594991615555955,183.37719178119988)
goal = (-71.88518123295214,196.99352098644988)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 398 of 399"
path = []
start = (-71.88518123295214,196.99352098644988)
goal = (192.509207219271,-110.37789085609128)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 399 of 399"
path = []
start = (192.509207219271,-110.37789085609128)
goal = (187.19727177363075,-179.11740693135854)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
endtime = datetime.datetime.now()
timeTaken = endtime - starttime
tts = str(timeTaken)
content = "Time Taken: " + tts + "\n" + content
content = content[:-1]
f = open('smo2sol-9.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
