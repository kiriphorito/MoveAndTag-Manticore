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
rand = (-210,210)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 145"
path = []
start = (-167.5384626894451,177.08127484328156)
goal = (-160.14363897371507,190.47384648668032)
print "     Node 1 and 2 of 9"
path += rrtpath(obstacleList,start,goal,rand)
start = (-160.14363897371507,190.47384648668032)
goal = (-182.88104144918438,186.6384384813557)
print "     Node 2 and 3 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-182.88104144918438,186.6384384813557)
goal = (-159.01223917250468,147.62976948844948)
print "     Node 3 and 4 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-159.01223917250468,147.62976948844948)
goal = (-116.9803902725464,148.39040461366938)
print "     Node 4 and 5 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-116.9803902725464,148.39040461366938)
goal = (-122.61033043101807,112.36418709364398)
print "     Node 5 and 6 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-122.61033043101807,112.36418709364398)
goal = (-189.98081085180058,58.80695996587633)
print "     Node 6 and 7 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-189.98081085180058,58.80695996587633)
goal = (-192.17436799398718,-26.44820478610069)
print "     Node 7 and 8 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-192.17436799398718,-26.44820478610069)
goal = (104.60595709380573,30.272731088438263)
print "     Node 8 and 9 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 145"
path = []
start = (-160.14363897371507,190.47384648668032)
goal = (-161.8661603672971,193.25686093664342)
print "     Node 1 and 2 of 9"
path += rrtpath(obstacleList,start,goal,rand)
start = (-161.8661603672971,193.25686093664342)
goal = (-150.4509718764017,189.5578533235651)
print "     Node 2 and 3 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-150.4509718764017,189.5578533235651)
goal = (-136.34839529947263,183.89378477189575)
print "     Node 3 and 4 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-136.34839529947263,183.89378477189575)
goal = (-113.79950329229361,144.86881491039003)
print "     Node 4 and 5 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-113.79950329229361,144.86881491039003)
goal = (-139.82229709670202,102.51233399783723)
print "     Node 5 and 6 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-139.82229709670202,102.51233399783723)
goal = (-19.37978659582876,181.82037766620573)
print "     Node 6 and 7 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-19.37978659582876,181.82037766620573)
goal = (56.60860421323446,174.24674641926407)
print "     Node 7 and 8 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (56.60860421323446,174.24674641926407)
goal = (150.38186840963908,174.80024465138)
print "     Node 8 and 9 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 145"
path = []
start = (-161.8661603672971,193.25686093664342)
goal = (-140.69557426581997,192.9210777840674)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-140.69557426581997,192.9210777840674)
goal = (-129.4042831626197,196.799680650565)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-129.4042831626197,196.799680650565)
goal = (-90.9309497252289,177.41952936665973)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-90.9309497252289,177.41952936665973)
goal = (-130.7069099347018,105.05736175095461)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-130.7069099347018,105.05736175095461)
goal = (-156.89361799406072,49.25463078517646)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-156.89361799406072,49.25463078517646)
goal = (61.47792256699114,151.54156682694514)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (61.47792256699114,151.54156682694514)
goal = (151.69651618810423,178.00386953777388)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 145"
path = []
start = (-182.88104144918438,186.6384384813557)
goal = (-192.80817132408296,187.85284624989032)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-192.80817132408296,187.85284624989032)
goal = (-133.18362811297885,174.9244277562865)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-133.18362811297885,174.9244277562865)
goal = (-168.06069424506643,104.33211889511409)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-168.06069424506643,104.33211889511409)
goal = (-160.55047938474002,89.76852565300373)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-160.55047938474002,89.76852565300373)
goal = (-193.91073073239977,51.41431805666113)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-193.91073073239977,51.41431805666113)
goal = (-196.0750271576928,-35.85746699522505)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-196.0750271576928,-35.85746699522505)
goal = (97.17348896356816,5.058562803444374)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 145"
path = []
start = (-150.4509718764017,189.5578533235651)
goal = (-103.82610853351646,169.14642054615996)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-103.82610853351646,169.14642054615996)
goal = (-86.90958862400873,176.00011164101255)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-86.90958862400873,176.00011164101255)
goal = (-60.35817895511278,173.65001968889675)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-60.35817895511278,173.65001968889675)
goal = (-185.29278338598613,50.29298650200903)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-185.29278338598613,50.29298650200903)
goal = (56.520151546520935,124.03815259456445)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (56.520151546520935,124.03815259456445)
goal = (144.42868118315602,113.87445638580266)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 145"
path = []
start = (-140.69557426581997,192.9210777840674)
goal = (-103.52160736785252,162.10253240987515)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-103.52160736785252,162.10253240987515)
goal = (-111.49985272128981,135.81916451994277)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-111.49985272128981,135.81916451994277)
goal = (-86.106658365251,117.21666727310844)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-86.106658365251,117.21666727310844)
goal = (-57.655872428633785,78.53061548632945)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-57.655872428633785,78.53061548632945)
goal = (70.96916935731133,181.02003919203037)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (70.96916935731133,181.02003919203037)
goal = (154.0774977234011,175.34824547588516)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 145"
path = []
start = (-192.80817132408296,187.85284624989032)
goal = (-164.9410165864133,138.15535752831545)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-164.9410165864133,138.15535752831545)
goal = (-140.68362783159387,111.81224494943746)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-140.68362783159387,111.81224494943746)
goal = (-170.98851015218804,84.30739778676559)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-170.98851015218804,84.30739778676559)
goal = (-199.77284762354262,49.227375057209116)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-199.77284762354262,49.227375057209116)
goal = (-188.11449970152253,-42.140771328467025)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-188.11449970152253,-42.140771328467025)
goal = (-75.35847394388301,-136.91927425869574)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 145"
path = []
start = (-159.01223917250468,147.62976948844948)
goal = (-140.51127569872088,128.1689596447979)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-140.51127569872088,128.1689596447979)
goal = (-120.49108770057325,125.73515532641835)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-120.49108770057325,125.73515532641835)
goal = (-125.41364367297389,87.7632944796278)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-125.41364367297389,87.7632944796278)
goal = (-163.92619212210727,38.4460528098293)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-163.92619212210727,38.4460528098293)
goal = (-141.44723226957524,-37.40450470243667)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-141.44723226957524,-37.40450470243667)
goal = (79.21369774163514,-25.751922902262862)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 145"
path = []
start = (-136.34839529947263,183.89378477189575)
goal = (-76.39742015603157,184.8048233649862)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-76.39742015603157,184.8048233649862)
goal = (-55.39355255632813,154.512549595397)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-55.39355255632813,154.512549595397)
goal = (-70.78063612284006,64.85357814821265)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-70.78063612284006,64.85357814821265)
goal = (72.71622160143966,187.25524922282148)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (72.71622160143966,187.25524922282148)
goal = (139.48517642283218,92.00985369238697)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 145"
path = []
start = (-129.4042831626197,196.799680650565)
goal = (-62.85520190124109,186.26548135882)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-62.85520190124109,186.26548135882)
goal = (-81.30762351289604,110.47529783450017)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-81.30762351289604,110.47529783450017)
goal = (6.652766273510736,191.32894466411045)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.652766273510736,191.32894466411045)
goal = (32.903268734998164,60.47844597862661)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (32.903268734998164,60.47844597862661)
goal = (162.1380404243028,172.08587580755045)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 145"
path = []
start = (-133.18362811297885,174.9244277562865)
goal = (-84.54591740698535,142.9829993729643)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-84.54591740698535,142.9829993729643)
goal = (-68.18685167869836,114.00875398209234)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-68.18685167869836,114.00875398209234)
goal = (-74.45044962079423,58.70164830836387)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-74.45044962079423,58.70164830836387)
goal = (48.5517139447976,75.43656268616019)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (48.5517139447976,75.43656268616019)
goal = (132.49850857458927,57.437546168374524)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 145"
path = []
start = (-103.82610853351646,169.14642054615996)
goal = (-87.54472902315102,137.97604292902525)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-87.54472902315102,137.97604292902525)
goal = (-44.38445615804105,140.75982872286232)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-44.38445615804105,140.75982872286232)
goal = (-2.697011300648228,143.9363868018945)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.697011300648228,143.9363868018945)
goal = (69.08183864042485,115.87521707345041)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (69.08183864042485,115.87521707345041)
goal = (157.1162093359028,134.1905964196951)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 145"
path = []
start = (-103.52160736785252,162.10253240987515)
goal = (-74.69986553266664,143.5625874077504)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-74.69986553266664,143.5625874077504)
goal = (-84.94644383140314,96.5047712876464)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-84.94644383140314,96.5047712876464)
goal = (3.6518025864384924,145.14340172081654)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.6518025864384924,145.14340172081654)
goal = (50.29802907402171,69.85838578352286)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (50.29802907402171,69.85838578352286)
goal = (101.69014017973456,3.7218008401690668)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 145"
path = []
start = (-164.9410165864133,138.15535752831545)
goal = (-165.30406044773449,96.81176673810705)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-165.30406044773449,96.81176673810705)
goal = (-177.57592007295372,71.31254151109596)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-177.57592007295372,71.31254151109596)
goal = (-196.69855439285806,27.834434320465874)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-196.69855439285806,27.834434320465874)
goal = (-112.32458553403157,-33.51142488650146)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-112.32458553403157,-33.51142488650146)
goal = (46.73777511449396,-68.52588770376283)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 145"
path = []
start = (-140.51127569872088,128.1689596447979)
goal = (-127.51423790185589,109.21303293027427)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-127.51423790185589,109.21303293027427)
goal = (-100.56453051580209,90.38760116789007)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-100.56453051580209,90.38760116789007)
goal = (-90.77504013498654,50.118210844880196)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-90.77504013498654,50.118210844880196)
goal = (-79.56868068181504,-32.29116084925337)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-79.56868068181504,-32.29116084925337)
goal = (90.81477949750308,-11.307607901068195)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 145"
path = []
start = (-116.9803902725464,148.39040461366938)
goal = (-115.85067430030645,116.21117611881033)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-115.85067430030645,116.21117611881033)
goal = (-93.46010256798074,93.64977584785044)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-93.46010256798074,93.64977584785044)
goal = (-66.3663668140249,52.79695303890412)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-66.3663668140249,52.79695303890412)
goal = (25.428748226877303,37.188938237140235)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (25.428748226877303,37.188938237140235)
goal = (116.02752934754909,20.978674052120937)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 145"
path = []
start = (-113.79950329229361,144.86881491039003)
goal = (-88.5637634439799,91.42884230939023)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-88.5637634439799,91.42884230939023)
goal = (-78.88271167082812,46.39849311799463)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-78.88271167082812,46.39849311799463)
goal = (-11.26754925809621,-0.92829795549045)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-11.26754925809621,-0.92829795549045)
goal = (135.43881224772525,54.31371766853249)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 145"
path = []
start = (-90.9309497252289,177.41952936665973)
goal = (-34.508674614966395,164.8932150955444)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-34.508674614966395,164.8932150955444)
goal = (16.99561320722657,164.5436919945053)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (16.99561320722657,164.5436919945053)
goal = (89.50353370971777,155.02679281841478)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (89.50353370971777,155.02679281841478)
goal = (161.57158392174603,155.41629449076277)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 145"
path = []
start = (-168.06069424506643,104.33211889511409)
goal = (-167.420355244376,65.84567786733004)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-167.420355244376,65.84567786733004)
goal = (-178.07632880328916,20.692979382051135)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-178.07632880328916,20.692979382051135)
goal = (-149.05715932503827,-50.565819556400754)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-149.05715932503827,-50.565819556400754)
goal = (-63.421763131399814,-142.95125900050527)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 145"
path = []
start = (-86.90958862400873,176.00011164101255)
goal = (-30.059263565081153,175.98802720081108)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-30.059263565081153,175.98802720081108)
goal = (16.553181353247737,160.42913458619068)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (16.553181353247737,160.42913458619068)
goal = (35.3114899809849,44.90291439899519)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (35.3114899809849,44.90291439899519)
goal = (155.81481807382767,106.57155500552682)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 145"
path = []
start = (-111.49985272128981,135.81916451994277)
goal = (-124.54900776877986,68.57819097581671)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-124.54900776877986,68.57819097581671)
goal = (-101.52181123721053,34.287608930758125)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-101.52181123721053,34.287608930758125)
goal = (15.566988520087364,20.995063998322394)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (15.566988520087364,20.995063998322394)
goal = (60.4133053795868,-55.791345424862186)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 145"
path = []
start = (-140.68362783159387,111.81224494943746)
goal = (-158.3187138062852,60.75965295401289)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-158.3187138062852,60.75965295401289)
goal = (-124.65010938588978,26.618094878315333)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-124.65010938588978,26.618094878315333)
goal = (-130.27297421878882,-47.994534588517865)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-130.27297421878882,-47.994534588517865)
goal = (78.6871095272557,-44.345960278967624)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 145"
path = []
start = (-120.49108770057325,125.73515532641835)
goal = (-105.5654897473842,61.27350493488592)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-105.5654897473842,61.27350493488592)
goal = (-101.82406544853832,20.447301490103285)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-101.82406544853832,20.447301490103285)
goal = (-54.54237754241703,-29.061833115256036)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-54.54237754241703,-29.061833115256036)
goal = (129.61889248523028,34.663239617011754)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 145"
path = []
start = (-76.39742015603157,184.8048233649862)
goal = (-27.8233311929597,173.3710144854063)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-27.8233311929597,173.3710144854063)
goal = (19.96132159479052,173.55144716376935)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (19.96132159479052,173.55144716376935)
goal = (90.82633112018863,150.96158555347563)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (90.82633112018863,150.96158555347563)
goal = (165.26026139301797,132.36461885060794)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 145"
path = []
start = (-62.85520190124109,186.26548135882)
goal = (-22.50041052453028,181.9017005941913)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.50041052453028,181.9017005941913)
goal = (20.519498492283105,195.0972996215038)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (20.519498492283105,195.0972996215038)
goal = (96.17457451846275,171.81530066784296)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (96.17457451846275,171.81530066784296)
goal = (170.3109980281771,159.15890363920988)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 145"
path = []
start = (-84.54591740698535,142.9829993729643)
goal = (-37.055829507827724,103.21995180513335)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-37.055829507827724,103.21995180513335)
goal = (-9.196393801384858,105.43031523592111)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-9.196393801384858,105.43031523592111)
goal = (76.28160569705153,97.28177640999536)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (76.28160569705153,97.28177640999536)
goal = (131.28391666426745,35.89358997786172)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 145"
path = []
start = (-87.54472902315102,137.97604292902525)
goal = (-50.48229100969945,84.35768238404597)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-50.48229100969945,84.35768238404597)
goal = (3.0716167593715227,119.67241693569804)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.0716167593715227,119.67241693569804)
goal = (-24.831571846743344,-14.10313123374408)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-24.831571846743344,-14.10313123374408)
goal = (111.90901251844383,-11.164634786719887)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 145"
path = []
start = (-74.69986553266664,143.5625874077504)
goal = (-22.6381712851651,102.08976543698549)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.6381712851651,102.08976543698549)
goal = (-3.6811319555396835,97.61513486548392)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.6811319555396835,97.61513486548392)
goal = (61.032982760536186,60.7029932995481)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (61.032982760536186,60.7029932995481)
goal = (119.7387817420352,-6.215854313702266)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 145"
path = []
start = (-165.30406044773449,96.81176673810705)
goal = (-169.89476162363206,56.330524352502096)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-169.89476162363206,56.330524352502096)
goal = (-199.31933022925438,25.29340184766022)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-199.31933022925438,25.29340184766022)
goal = (-176.20780161724633,-57.721307337768025)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-176.20780161724633,-57.721307337768025)
goal = (-106.2439819856718,-167.9149256122417)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 145"
path = []
start = (-127.51423790185589,109.21303293027427)
goal = (-148.49390536432958,51.91288365933991)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-148.49390536432958,51.91288365933991)
goal = (-127.32966016669769,16.943077501925018)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-127.32966016669769,16.943077501925018)
goal = (-29.797286866819434,-19.385321715673996)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-29.797286866819434,-19.385321715673996)
goal = (21.757893471877054,-116.13864838260932)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 145"
path = []
start = (-115.85067430030645,116.21117611881033)
goal = (-65.32275432040362,75.45313689512977)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-65.32275432040362,75.45313689512977)
goal = (-76.31637071191118,23.91864286330997)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-76.31637071191118,23.91864286330997)
goal = (-39.58382909577489,-26.439199610794674)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-39.58382909577489,-26.439199610794674)
goal = (79.80138950290711,-64.85144534038506)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 145"
path = []
start = (-122.61033043101807,112.36418709364398)
goal = (-68.70147625902052,69.43442061359889)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-68.70147625902052,69.43442061359889)
goal = (-138.58346171509345,16.788027974171882)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-138.58346171509345,16.788027974171882)
goal = (-80.25134496528369,-47.25070841898017)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-80.25134496528369,-47.25070841898017)
goal = (-34.89247201983457,-146.19601283834487)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 145"
path = []
start = (-139.82229709670202,102.51233399783723)
goal = (-112.33999799348133,14.529119313970682)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-112.33999799348133,14.529119313970682)
goal = (-114.53420614878836,-52.581047312894555)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-114.53420614878836,-52.581047312894555)
goal = (-102.65366259069539,-167.68068937870729)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 145"
path = []
start = (-130.7069099347018,105.05736175095461)
goal = (-115.36949199338511,10.549611070311727)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-115.36949199338511,10.549611070311727)
goal = (-113.90393168761612,-62.679913720979386)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-113.90393168761612,-62.679913720979386)
goal = (86.27888036793797,-66.12092264244839)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 145"
path = []
start = (-160.55047938474002,89.76852565300373)
goal = (-197.3422928566493,13.312195781567624)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-197.3422928566493,13.312195781567624)
goal = (-185.793736323553,-63.66987763264643)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-185.793736323553,-63.66987763264643)
goal = (-152.03403979601916,-174.79797625973768)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 145"
path = []
start = (-60.35817895511278,173.65001968889675)
goal = (27.31195197429531,138.41987626764)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (27.31195197429531,138.41987626764)
goal = (81.8405268291591,110.91695205603008)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (81.8405268291591,110.91695205603008)
goal = (172.10569307139434,129.22167126466553)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 145"
path = []
start = (-86.106658365251,117.21666727310844)
goal = (-3.2308517552532976,96.46719797436572)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.2308517552532976,96.46719797436572)
goal = (-0.9467459936551847,-12.672969808223485)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.9467459936551847,-12.672969808223485)
goal = (145.44471989735553,31.88217268870912)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 145"
path = []
start = (-170.98851015218804,84.30739778676559)
goal = (-135.91605111603573,2.1896084594929164)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-135.91605111603573,2.1896084594929164)
goal = (-165.7663524603215,-76.13647505495744)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-165.7663524603215,-76.13647505495744)
goal = (-163.86452005557567,-177.71298764882152)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 145"
path = []
start = (-125.41364367297389,87.7632944796278)
goal = (-103.00358829720655,6.3361757278075)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-103.00358829720655,6.3361757278075)
goal = (-44.1253047758473,-44.50278830485087)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-44.1253047758473,-44.50278830485087)
goal = (-55.97145524478836,-164.72325215443885)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 145"
path = []
start = (-55.39355255632813,154.512549595397)
goal = (11.27637966064944,108.89304467779459)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.27637966064944,108.89304467779459)
goal = (82.09432054093566,104.96333121779367)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (82.09432054093566,104.96333121779367)
goal = (164.62134626410017,84.70781251747479)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 145"
path = []
start = (-81.30762351289604,110.47529783450017)
goal = (-50.41673115143843,30.050617756157465)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-50.41673115143843,30.050617756157465)
goal = (12.253766634024515,-4.459806682351001)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (12.253766634024515,-4.459806682351001)
goal = (125.36305996101441,-10.895293058990688)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 145"
path = []
start = (-68.18685167869836,114.00875398209234)
goal = (-2.4050887725639427,73.3876798313322)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.4050887725639427,73.3876798313322)
goal = (36.0978587246604,18.344147590077455)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (36.0978587246604,18.344147590077455)
goal = (153.04123898616757,50.51598274366799)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 145"
path = []
start = (-44.38445615804105,140.75982872286232)
goal = (6.800134992488097,77.06635423538211)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.800134992488097,77.06635423538211)
goal = (88.13662832687254,126.42578875356259)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (88.13662832687254,126.42578875356259)
goal = (162.1248711023636,77.0710145377729)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 145"
path = []
start = (-84.94644383140314,96.5047712876464)
goal = (-60.24692201568723,18.99654753664049)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-60.24692201568723,18.99654753664049)
goal = (-11.923303705533442,-23.28563910703815)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-11.923303705533442,-23.28563910703815)
goal = (140.79354010556273,3.404276774215788)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 145"
path = []
start = (-177.57592007295372,71.31254151109596)
goal = (-190.26144526897957,-9.481914356020383)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-190.26144526897957,-9.481914356020383)
goal = (-188.03343586641824,-85.13200074187858)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-188.03343586641824,-85.13200074187858)
goal = (-192.24985675495887,-177.0418104548843)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 145"
path = []
start = (-100.56453051580209,90.38760116789007)
goal = (-89.94472274049983,11.023495976199143)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-89.94472274049983,11.023495976199143)
goal = (-36.55951541437267,-44.04663685378313)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-36.55951541437267,-44.04663685378313)
goal = (-32.60705723740159,-158.61104702178125)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 145"
path = []
start = (-93.46010256798074,93.64977584785044)
goal = (-38.91059616922158,23.296669879335752)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-38.91059616922158,23.296669879335752)
goal = (-8.433951238458548,-28.995438607130097)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-8.433951238458548,-28.995438607130097)
goal = (132.3395261394918,-23.606277395089677)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 145"
path = []
start = (-88.5637634439799,91.42884230939023)
goal = (-22.283802752095653,39.1986404811457)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.283802752095653,39.1986404811457)
goal = (-31.80556332317508,-42.82874671176518)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-31.80556332317508,-42.82874671176518)
goal = (157.87063267032465,50.55215633508698)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 145"
path = []
start = (-34.508674614966395,164.8932150955444)
goal = (36.04892015989432,173.41611963701354)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (36.04892015989432,173.41611963701354)
goal = (96.81354655447103,174.0232842210542)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (96.81354655447103,174.0232842210542)
goal = (177.09467436222883,186.9689090500557)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 145"
path = []
start = (-167.420355244376,65.84567786733004)
goal = (-156.23088109714357,-10.88300380764656)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-156.23088109714357,-10.88300380764656)
goal = (-95.64179096249403,-68.20019142784685)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-95.64179096249403,-68.20019142784685)
goal = (-74.62281020616506,-170.81212294980176)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 145"
path = []
start = (-30.059263565081153,175.98802720081108)
goal = (39.51273267835131,180.58645540476437)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (39.51273267835131,180.58645540476437)
goal = (98.09190199323342,176.0329786599092)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (98.09190199323342,176.0329786599092)
goal = (171.8471102252659,100.83161601973512)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 145"
path = []
start = (-124.54900776877986,68.57819097581671)
goal = (-94.29379003543774,-3.1001053172707884)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-94.29379003543774,-3.1001053172707884)
goal = (-93.46280715484299,-71.11614075897967)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-93.46280715484299,-71.11614075897967)
goal = (-22.134479065077528,-159.30262505612362)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 145"
path = []
start = (-158.3187138062852,60.75965295401289)
goal = (-190.0337872213163,-10.769949438108029)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-190.0337872213163,-10.769949438108029)
goal = (-169.4329898317196,-89.55101520804578)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-169.4329898317196,-89.55101520804578)
goal = (-152.01427942699485,-188.589363345043)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 145"
path = []
start = (-105.5654897473842,61.27350493488592)
goal = (-46.50693651575358,16.625286983042514)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-46.50693651575358,16.625286983042514)
goal = (-15.779262363123877,-41.830746361842216)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.779262363123877,-41.830746361842216)
goal = (-9.757071866475997,-157.13711246243207)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 145"
path = []
start = (-27.8233311929597,173.3710144854063)
goal = (38.11259026882428,153.57883920622345)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (38.11259026882428,153.57883920622345)
goal = (99.22959206833713,184.11618515217287)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (99.22959206833713,184.11618515217287)
goal = (169.09133536098847,81.5996330809648)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 145"
path = []
start = (-22.50041052453028,181.9017005941913)
goal = (40.304603066962216,142.6725360321803)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (40.304603066962216,142.6725360321803)
goal = (101.55056892490506,163.80848399874606)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (101.55056892490506,163.80848399874606)
goal = (188.41093303721595,146.9958996813932)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 145"
path = []
start = (-37.055829507827724,103.21995180513335)
goal = (-11.776504750619893,38.550749467985355)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.776504750619893,38.550749467985355)
goal = (69.74390898355767,61.317817743935905)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (69.74390898355767,61.317817743935905)
goal = (161.83796835355497,14.938021853686138)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 145"
path = []
start = (-50.48229100969945,84.35768238404597)
goal = (9.823283477992732,35.431512347262384)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.823283477992732,35.431512347262384)
goal = (43.86180808464087,12.941753693875938)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (43.86180808464087,12.941753693875938)
goal = (157.0079624400816,3.8264299096435934)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 145"
path = []
start = (-22.6381712851651,102.08976543698549)
goal = (40.48389939088915,129.3151538141633)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (40.48389939088915,129.3151538141633)
goal = (76.00852315164042,71.92921011417559)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (76.00852315164042,71.92921011417559)
goal = (174.3750527160817,49.80801242000203)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 145"
path = []
start = (-169.89476162363206,56.330524352502096)
goal = (-183.87496529824463,-23.73647994983301)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-183.87496529824463,-23.73647994983301)
goal = (-104.73721993425569,-82.1087459117523)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-104.73721993425569,-82.1087459117523)
goal = (-161.70321134807946,-196.34235999627444)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 145"
path = []
start = (-148.49390536432958,51.91288365933991)
goal = (-158.53462656809242,-36.12118640518807)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-158.53462656809242,-36.12118640518807)
goal = (-188.8822164995839,-95.96611989729267)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-188.8822164995839,-95.96611989729267)
goal = (-82.20511946911682,-184.43860418452175)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 145"
path = []
start = (-65.32275432040362,75.45313689512977)
goal = (-8.974233080475614,4.8749259223372405)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.974233080475614,4.8749259223372405)
goal = (31.981673347638008,-6.826607130094118)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (31.981673347638008,-6.826607130094118)
goal = (112.65256191471963,-72.48206292981698)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 145"
path = []
start = (-68.70147625902052,69.43442061359889)
goal = (-40.57277826430621,-14.104171181584007)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-40.57277826430621,-14.104171181584007)
goal = (-10.089538912646162,-49.26267208395902)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-10.089538912646162,-49.26267208395902)
goal = (90.60841613792923,-98.1102665513852)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 145"
path = []
start = (-189.98081085180058,58.80695996587633)
goal = (-196.0724456882388,-25.721729476945796)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-196.0724456882388,-25.721729476945796)
goal = (-185.9447556325072,-98.50937841991927)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-185.9447556325072,-98.50937841991927)
goal = (-91.9206630998946,-192.16514269716058)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 145"
path = []
start = (-19.37978659582876,181.82037766620573)
goal = (92.35710457945908,116.0026374247629)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (92.35710457945908,116.0026374247629)
goal = (181.24697727358352,94.01285296890637)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 145"
path = []
start = (-156.89361799406072,49.25463078517646)
goal = (-95.85941291231599,-94.36033792754377)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-95.85941291231599,-94.36033792754377)
goal = (-32.93733958500593,-171.93229552731722)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 145"
path = []
start = (-193.91073073239977,51.41431805666113)
goal = (-119.57075189913043,-110.33968425835381)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-119.57075189913043,-110.33968425835381)
goal = (-54.676784941875184,-184.2046474310436)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 145"
path = []
start = (-185.29278338598613,50.29298650200903)
goal = (-132.06458502398425,-116.88973825838774)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-132.06458502398425,-116.88973825838774)
goal = (-52.63167038062102,-191.50799792501903)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 145"
path = []
start = (-57.655872428633785,78.53061548632945)
goal = (32.55663275085382,-15.949762331189504)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.55663275085382,-15.949762331189504)
goal = (127.61965057972435,-53.64643755186319)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 145"
path = []
start = (-199.77284762354262,49.227375057209116)
goal = (-67.91750395775313,-80.51632120381566)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-67.91750395775313,-80.51632120381566)
goal = (-44.58044181525304,-195.3916039823016)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 145"
path = []
start = (-163.92619212210727,38.4460528098293)
goal = (-54.526015842388404,-69.46049884028835)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-54.526015842388404,-69.46049884028835)
goal = (20.900170199577246,-145.9400560252063)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 145"
path = []
start = (-70.78063612284006,64.85357814821265)
goal = (13.4972447519024,-39.76067852944726)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (13.4972447519024,-39.76067852944726)
goal = (120.50700088687324,-63.45422011831624)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 145"
path = []
start = (6.652766273510736,191.32894466411045)
goal = (114.36671250461205,172.90876179817485)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (114.36671250461205,172.90876179817485)
goal = (195.30181254739716,137.99807837642078)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 145"
path = []
start = (-74.45044962079423,58.70164830836387)
goal = (33.05966865479411,-24.479661972434428)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (33.05966865479411,-24.479661972434428)
goal = (52.36322416668307,-132.8604789894659)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 145"
path = []
start = (-2.697011300648228,143.9363868018945)
goal = (95.2570903014194,121.1658433511376)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (95.2570903014194,121.1658433511376)
goal = (182.21837218065133,92.563828728636)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 145"
path = []
start = (3.6518025864384924,145.14340172081654)
goal = (86.53240245023062,93.33077400890738)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (86.53240245023062,93.33077400890738)
goal = (199.1005091149297,105.70539276633428)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 145"
path = []
start = (-196.69855439285806,27.834434320465874)
goal = (-98.87057382300965,-113.24560665802639)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-98.87057382300965,-113.24560665802639)
goal = (-17.650618744268513,-195.0794411873892)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 145"
path = []
start = (-90.77504013498654,50.118210844880196)
goal = (-17.65366381359152,-71.50101143252323)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-17.65366381359152,-71.50101143252323)
goal = (68.88519330261101,-123.91434478092837)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 145"
path = []
start = (-66.3663668140249,52.79695303890412)
goal = (63.07897645678071,27.53723084968479)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (63.07897645678071,27.53723084968479)
goal = (98.88942691887064,-95.94219358089684)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 145"
path = []
start = (-78.88271167082812,46.39849311799463)
goal = (-3.6438698130710065,-65.61279939819391)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.6438698130710065,-65.61279939819391)
goal = (72.80032570921031,-122.02936997373683)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 145"
path = []
start = (16.99561320722657,164.5436919945053)
goal = (106.20638358607863,134.72130009495652)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (106.20638358607863,134.72130009495652)
goal = (192.4857358120006,42.01621813213171)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 145"
path = []
start = (-178.07632880328916,20.692979382051135)
goal = (-72.1221107346671,-98.31226655460563)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-72.1221107346671,-98.31226655460563)
goal = (7.601120137847516,-177.39767744374117)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 145"
path = []
start = (16.553181353247737,160.42913458619068)
goal = (106.52688811164143,117.9794500415399)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (106.52688811164143,117.9794500415399)
goal = (173.70328107888304,18.74938316442831)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 145"
path = []
start = (-101.52181123721053,34.287608930758125)
goal = (-34.30642084540966,-90.07633844127297)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-34.30642084540966,-90.07633844127297)
goal = (22.18170821024512,-163.26844847789116)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 145"
path = []
start = (-124.65010938588978,26.618094878315333)
goal = (-44.49990224821488,-104.33515680675241)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-44.49990224821488,-104.33515680675241)
goal = (44.64813836641585,-148.8826194412232)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 145"
path = []
start = (-101.82406544853832,20.447301490103285)
goal = (-0.47716379958154675,-69.15226172876555)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.47716379958154675,-69.15226172876555)
goal = (70.93083912423708,-126.31589955903264)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 145"
path = []
start = (19.96132159479052,173.55144716376935)
goal = (120.51304264521553,158.81366240782341)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (120.51304264521553,158.81366240782341)
goal = (195.39854311039016,38.95655451548956)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 145"
path = []
start = (20.519498492283105,195.0972996215038)
goal = (126.85847029568566,192.8632772217319)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (126.85847029568566,192.8632772217319)
goal = (193.53012079682418,29.154053946572077)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 145"
path = []
start = (-9.196393801384858,105.43031523592111)
goal = (72.41857094951587,58.4908745086945)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (72.41857094951587,58.4908745086945)
goal = (185.70857213751583,18.923735818136976)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 145"
path = []
start = (3.0716167593715227,119.67241693569804)
goal = (104.55344130006665,82.19139399194233)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (104.55344130006665,82.19139399194233)
goal = (194.2983076387988,11.266963412462644)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 145"
path = []
start = (-3.6811319555396835,97.61513486548392)
goal = (66.80786030078076,18.190500539571843)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (66.80786030078076,18.190500539571843)
goal = (172.2485526020638,-31.914296429547846)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 145"
path = []
start = (-199.31933022925438,25.29340184766022)
goal = (-123.82736337806963,-129.42315227404316)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-123.82736337806963,-129.42315227404316)
goal = (44.77380797298861,-153.3443458598519)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 145"
path = []
start = (-127.32966016669769,16.943077501925018)
goal = (-69.88657380752966,-118.00328624108802)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-69.88657380752966,-118.00328624108802)
goal = (43.37946145349355,-156.16998206845918)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 145"
path = []
start = (-76.31637071191118,23.91864286330997)
goal = (20.957305340859676,-57.96670590087044)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (20.957305340859676,-57.96670590087044)
goal = (69.19741019452488,-132.94320416353975)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 145"
path = []
start = (-138.58346171509345,16.788027974171882)
goal = (-128.8835067832824,-137.002266173935)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-128.8835067832824,-137.002266173935)
goal = (42.27920095643617,-158.33102630397548)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 145"
path = []
start = (-112.33999799348133,14.529119313970682)
goal = (-39.70397242208472,-103.93562334033342)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-39.70397242208472,-103.93562334033342)
goal = (92.51233263423069,-107.73767744209385)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 145"
path = []
start = (-115.36949199338511,10.549611070311727)
goal = (-30.97269355251865,-103.80572055328084)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-30.97269355251865,-103.80572055328084)
goal = (97.49504190820375,-104.16738211643172)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 145"
path = []
start = (-197.3422928566493,13.312195781567624)
goal = (-151.03103879376215,-142.12129053285997)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-151.03103879376215,-142.12129053285997)
goal = (80.21508235715078,-135.73944288806564)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 145"
path = []
start = (27.31195197429531,138.41987626764)
goal = (109.0704351800963,77.1610826056812)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (109.0704351800963,77.1610826056812)
goal = (197.24776058842576,-8.77451163255759)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 145"
path = []
start = (-3.2308517552532976,96.46719797436572)
goal = (64.50844183720636,13.846752965631254)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (64.50844183720636,13.846752965631254)
goal = (134.10742349710847,-74.7753193077446)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 145"
path = []
start = (-135.91605111603573,2.1896084594929164)
goal = (-102.97094208368458,-133.42408335675157)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-102.97094208368458,-133.42408335675157)
goal = (79.63967087832009,-140.93383086372802)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 145"
path = []
start = (-103.00358829720655,6.3361757278075)
goal = (-9.222617876603778,-96.82702696262027)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.222617876603778,-96.82702696262027)
goal = (93.10525604159062,-130.0345338991931)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 145"
path = []
start = (11.27637966064944,108.89304467779459)
goal = (95.23082361425992,48.729359882277066)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (95.23082361425992,48.729359882277066)
goal = (190.04057911129723,-17.370719966804018)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 145"
path = []
start = (-50.41673115143843,30.050617756157465)
goal = (47.00791179439045,-22.305796934530974)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (47.00791179439045,-22.305796934530974)
goal = (145.3175269729868,-73.56500513850794)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 145"
path = []
start = (-2.4050887725639427,73.3876798313322)
goal = (68.78323841663246,8.061930837978707)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (68.78323841663246,8.061930837978707)
goal = (149.23716351867682,-69.32694653032837)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 145"
path = []
start = (6.800134992488097,77.06635423538211)
goal = (96.77405349629458,42.364346960780836)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (96.77405349629458,42.364346960780836)
goal = (178.61003845015603,-43.1899141317991)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 145"
path = []
start = (-60.24692201568723,18.99654753664049)
goal = (25.003091549870675,-58.12112412914857)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (25.003091549870675,-58.12112412914857)
goal = (103.75482610298087,-143.72778982368686)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 145"
path = []
start = (-190.26144526897957,-9.481914356020383)
goal = (-182.98191505103208,-149.49062549518098)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-182.98191505103208,-149.49062549518098)
goal = (51.303436210931295,-192.4807188246652)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 145"
path = []
start = (-89.94472274049983,11.023495976199143)
goal = (17.75155954490444,-65.63522845086686)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (17.75155954490444,-65.63522845086686)
goal = (86.5813582925137,-164.00315884418433)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 145"
path = []
start = (-38.91059616922158,23.296669879335752)
goal = (43.17490351123075,-45.23996262217733)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (43.17490351123075,-45.23996262217733)
goal = (108.03967539588365,-140.2413858513613)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 145"
path = []
start = (-22.283802752095653,39.1986404811457)
goal = (78.5906579856537,6.859985959173713)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (78.5906579856537,6.859985959173713)
goal = (167.3405346676954,-62.73748873879791)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 145"
path = []
start = (36.04892015989432,173.41611963701354)
goal = (137.72964241977024,177.45677637308887)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (137.72964241977024,177.45677637308887)
goal = (184.3389050869725,-59.778430494427994)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 145"
path = []
start = (-156.23088109714357,-10.88300380764656)
goal = (-117.09642925644049,-137.6748550444163)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-117.09642925644049,-137.6748550444163)
goal = (55.690973159086184,-194.70990631779367)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 145"
path = []
start = (39.51273267835131,180.58645540476437)
goal = (137.62964472918713,145.897416918508)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (137.62964472918713,145.897416918508)
goal = (196.33182886318292,-54.25075654202857)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 145"
path = []
start = (-94.29379003543774,-3.1001053172707884)
goal = (-13.656171965935073,-105.35881757599692)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-13.656171965935073,-105.35881757599692)
goal = (68.49566084273795,-195.2262321681768)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 145"
path = []
start = (-190.0337872213163,-10.769949438108029)
goal = (-141.98853446903658,-145.9153777790457)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-141.98853446903658,-145.9153777790457)
goal = (78.45959876577064,-191.01249602930338)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 145"
path = []
start = (-46.50693651575358,16.625286983042514)
goal = (39.55190435399527,-66.19068493242631)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (39.55190435399527,-66.19068493242631)
goal = (126.85601341842386,-131.65420701229607)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 145"
path = []
start = (38.11259026882428,153.57883920622345)
goal = (120.62300072328196,88.23406648307264)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (120.62300072328196,88.23406648307264)
goal = (176.15384846239834,-99.10324699456817)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 145"
path = []
start = (40.304603066962216,142.6725360321803)
goal = (147.24091406198932,145.1327238448202)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (147.24091406198932,145.1327238448202)
goal = (151.6029395268851,-111.3265965067884)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 145"
path = []
start = (-11.776504750619893,38.550749467985355)
goal = (88.71118553045659,21.02106990017134)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (88.71118553045659,21.02106990017134)
goal = (115.1212750325356,-149.83469336627974)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 145"
path = []
start = (9.823283477992732,35.431512347262384)
goal = (96.54123695832959,34.3877684517536)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (96.54123695832959,34.3877684517536)
goal = (167.13454399024295,-112.49499721904219)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 145"
path = []
start = (40.48389939088915,129.3151538141633)
goal = (108.48182742679546,59.124695671754296)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (108.48182742679546,59.124695671754296)
goal = (179.30809731937643,-107.65975614949576)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 145"
path = []
start = (-183.87496529824463,-23.73647994983301)
goal = (-142.58687627523545,-147.7589927234261)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-142.58687627523545,-147.7589927234261)
goal = (85.16107137767227,-187.04813396520592)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 124 of 145"
path = []
start = (-158.53462656809242,-36.12118640518807)
goal = (-131.9038473552964,-158.8273288774687)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-131.9038473552964,-158.8273288774687)
goal = (105.80330401113912,-173.07709743749675)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 125 of 145"
path = []
start = (-8.974233080475614,4.8749259223372405)
goal = (38.821111708573625,-70.59371415662415)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (38.821111708573625,-70.59371415662415)
goal = (121.65684844791036,-149.61101564745104)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 126 of 145"
path = []
start = (-40.57277826430621,-14.104171181584007)
goal = (26.173128287623456,-82.11874626759842)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (26.173128287623456,-82.11874626759842)
goal = (127.03233173220235,-147.23629651434277)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 127 of 145"
path = []
start = (-196.0724456882388,-25.721729476945796)
goal = (-195.96017205255336,-156.61388502895178)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-195.96017205255336,-156.61388502895178)
goal = (102.53066254197057,-186.521602974507)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 128 of 145"
path = []
start = (-192.17436799398718,-26.44820478610069)
goal = (-187.2293612719438,-168.91577368669167)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-187.2293612719438,-168.91577368669167)
goal = (100.97077172816103,-199.3658775104434)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 129 of 145"
path = []
start = (56.60860421323446,174.24674641926407)
goal = (191.9993454649392,-113.04235770027647)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 130 of 145"
path = []
start = (61.47792256699114,151.54156682694514)
goal = (196.20268226954056,-116.94072081018723)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 131 of 145"
path = []
start = (-196.0750271576928,-35.85746699522505)
goal = (123.76773084554526,-154.46614989111572)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 132 of 145"
path = []
start = (56.520151546520935,124.03815259456445)
goal = (167.10948404282226,-141.2806679346426)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 133 of 145"
path = []
start = (70.96916935731133,181.02003919203037)
goal = (182.95033673842266,-137.6016398158725)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 134 of 145"
path = []
start = (-188.11449970152253,-42.140771328467025)
goal = (135.64997540210737,-158.9735612754123)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 135 of 145"
path = []
start = (-141.44723226957524,-37.40450470243667)
goal = (131.34985725625324,-172.25752483433138)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 136 of 145"
path = []
start = (72.71622160143966,187.25524922282148)
goal = (186.20238013293368,-140.12535308488987)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 137 of 145"
path = []
start = (32.903268734998164,60.47844597862661)
goal = (156.50806806134688,-149.95326816907607)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 138 of 145"
path = []
start = (48.5517139447976,75.43656268616019)
goal = (172.32823426880913,-149.0157339396462)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 139 of 145"
path = []
start = (69.08183864042485,115.87521707345041)
goal = (199.61113616660435,-137.11584679315064)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 140 of 145"
path = []
start = (50.29802907402171,69.85838578352286)
goal = (150.79667685371294,-184.35937998746334)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 141 of 145"
path = []
start = (-112.32458553403157,-33.51142488650146)
goal = (150.74757142886608,-193.04620170010125)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 142 of 145"
path = []
start = (-79.56868068181504,-32.29116084925337)
goal = (157.72164536973793,-187.59898464660725)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 143 of 145"
path = []
start = (25.428748226877303,37.188938237140235)
goal = (173.79252365861362,-176.4715304879786)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 144 of 145"
path = []
start = (-11.26754925809621,-0.92829795549045)
goal = (156.29669098468815,-197.5769488718103)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 145 of 145"
path = []
start = (89.50353370971777,155.02679281841478)
goal = (165.8841727710673,-199.2104428148204)
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
f = open('smo2sol-8.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
