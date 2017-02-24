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
rand = (-1,11)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 111"
path = []
start = (14.34315733696458,1.6912329390531937)
goal = (12.044799647618504,1.9620221379939622)
print "     Node 1 and 2 of 9"
path += rrtpath(obstacleList,start,goal,rand)
start = (12.044799647618504,1.9620221379939622)
goal = (10.270640716826243,2.2931653599403248)
print "     Node 2 and 3 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (10.270640716826243,2.2931653599403248)
goal = (3.85553009788822,3.0931619315649073)
print "     Node 3 and 4 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.85553009788822,3.0931619315649073)
goal = (1.860732918321773,0.5994308460443634)
print "     Node 4 and 5 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.860732918321773,0.5994308460443634)
goal = (-2.196755055443802,-4.640956390275605)
print "     Node 5 and 6 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.196755055443802,-4.640956390275605)
goal = (-3.2493702991817273,-19.351727665748307)
print "     Node 6 and 7 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.2493702991817273,-19.351727665748307)
goal = (-5.129636822329701,-35.089590128698596)
print "     Node 7 and 8 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.129636822329701,-35.089590128698596)
goal = (-5.726778654371742,-37.97241649855256)
print "     Node 8 and 9 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 2 of 111"
path = []
start = (12.044799647618504,1.9620221379939622)
goal = (11.47444787863762,3.745699953837054)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.47444787863762,3.745699953837054)
goal = (11.4107940028115,4.438396711824019)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (11.4107940028115,4.438396711824019)
goal = (11.329258159533858,7.607653421346832)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (11.329258159533858,7.607653421346832)
goal = (20.870571623042068,9.820493507900409)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (20.870571623042068,9.820493507900409)
goal = (21.56062974524562,12.197790863257957)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (21.56062974524562,12.197790863257957)
goal = (36.54781596013119,31.224118788057403)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (36.54781596013119,31.224118788057403)
goal = (42.12429186203424,42.31610500062258)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 3 of 111"
path = []
start = (10.270640716826243,2.2931653599403248)
goal = (12.984969136744041,-4.0054496206621835)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (12.984969136744041,-4.0054496206621835)
goal = (20.0177165186349,-3.645249909066621)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (20.0177165186349,-3.645249909066621)
goal = (15.404843877113436,-16.06746427850745)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (15.404843877113436,-16.06746427850745)
goal = (12.991536446370787,-19.952079815468977)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (12.991536446370787,-19.952079815468977)
goal = (7.63906995094186,-25.761474129116586)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.63906995094186,-25.761474129116586)
goal = (8.92332581032305,-36.176593652171455)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.92332581032305,-36.176593652171455)
goal = (7.213089920581865,-44.24104880893125)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 4 of 111"
path = []
start = (11.47444787863762,3.745699953837054)
goal = (8.786663200288224,6.4392860634852696)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.786663200288224,6.4392860634852696)
goal = (17.273441942326286,17.239408215477347)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (17.273441942326286,17.239408215477347)
goal = (24.860141928343396,22.14568168045688)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 5 of 111"
path = []
start = (11.4107940028115,4.438396711824019)
goal = (20.635165747583073,5.947327562614937)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (20.635165747583073,5.947327562614937)
goal = (24.27612393983445,11.54012567806128)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (24.27612393983445,11.54012567806128)
goal = (30.476370136087766,12.565278773359374)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (30.476370136087766,12.565278773359374)
goal = (33.369055825825825,13.329336493468212)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (33.369055825825825,13.329336493468212)
goal = (50.4229865733229,20.340954423273935)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (50.4229865733229,20.340954423273935)
goal = (54.83221071300322,22.74532950907598)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 6 of 111"
path = []
start = (8.786663200288224,6.4392860634852696)
goal = (13.533997748952288,19.477077221165665)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (13.533997748952288,19.477077221165665)
goal = (18.281168336297284,32.06060436132085)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 7 of 111"
path = []
start = (11.329258159533858,7.607653421346832)
goal = (6.046166408883316,16.949081085403485)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.046166408883316,16.949081085403485)
goal = (5.88083731078131,22.067297556327595)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.88083731078131,22.067297556327595)
goal = (7.489162736559486,26.953732420326396)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.489162736559486,26.953732420326396)
goal = (9.09072303832432,38.59580845796504)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 8 of 111"
path = []
start = (3.85553009788822,3.0931619315649073)
goal = (-1.5176888896037255,-0.22959553114546338)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.5176888896037255,-0.22959553114546338)
goal = (-10.716101819114712,-5.790846732300842)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-10.716101819114712,-5.790846732300842)
goal = (-13.675305129300952,-8.326621554295293)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-13.675305129300952,-8.326621554295293)
goal = (-22.584853834100464,-16.49129586787894)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-22.584853834100464,-16.49129586787894)
goal = (-22.928151277706753,-16.949757993103198)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-22.928151277706753,-16.949757993103198)
goal = (-22.08411667237579,-21.25659613330215)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 9 of 111"
path = []
start = (12.984969136744041,-4.0054496206621835)
goal = (20.447257859271744,-2.8555608066262366)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (20.447257859271744,-2.8555608066262366)
goal = (22.67282649853972,-2.0441316954870175)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (22.67282649853972,-2.0441316954870175)
goal = (36.56777809322233,1.3991839575175646)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (36.56777809322233,1.3991839575175646)
goal = (46.7635601464539,-2.9421863223344076)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 10 of 111"
path = []
start = (1.860732918321773,0.5994308460443634)
goal = (-5.386085248949975,1.7695784167601616)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.386085248949975,1.7695784167601616)
goal = (-13.627453910016015,6.901480793530325)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-13.627453910016015,6.901480793530325)
goal = (-23.55128969951076,12.864402059896854)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-23.55128969951076,12.864402059896854)
goal = (-29.179928460500502,16.75820286613773)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-29.179928460500502,16.75820286613773)
goal = (-42.358124936297024,28.318468944330412)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-42.358124936297024,28.318468944330412)
goal = (-46.8234227054565,33.553356930361794)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-46.8234227054565,33.553356930361794)
goal = (-48.23181543341552,38.35147044987472)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 11 of 111"
path = []
start = (20.635165747583073,5.947327562614937)
goal = (24.31918623261639,12.531483807384753)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (24.31918623261639,12.531483807384753)
goal = (34.29147795879126,23.755319399510476)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (34.29147795879126,23.755319399510476)
goal = (41.585514959242744,35.30061471525115)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (41.585514959242744,35.30061471525115)
goal = (44.55553143192867,42.57260975222433)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 12 of 111"
path = []
start = (-1.5176888896037255,-0.22959553114546338)
goal = (-15.740213982210612,0.2232831704341791)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-15.740213982210612,0.2232831704341791)
goal = (-34.66605700879898,-4.261010587990313)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-34.66605700879898,-4.261010587990313)
goal = (-36.25044472141615,-6.222590683081393)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 13 of 111"
path = []
start = (20.870571623042068,9.820493507900409)
goal = (22.390926613539733,18.146738957814982)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 14 of 111"
path = []
start = (20.0177165186349,-3.645249909066621)
goal = (21.707985775676846,-18.541255631180803)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (21.707985775676846,-18.541255631180803)
goal = (18.34494166691715,-22.777486965421275)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (18.34494166691715,-22.777486965421275)
goal = (17.377474198080307,-29.110750982456533)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (17.377474198080307,-29.110750982456533)
goal = (23.366157395889033,-30.266549944609096)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (23.366157395889033,-30.266549944609096)
goal = (24.054210311402613,-46.74783075948508)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 15 of 111"
path = []
start = (20.447257859271744,-2.8555608066262366)
goal = (28.019275294085787,0.21183527659022872)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (28.019275294085787,0.21183527659022872)
goal = (37.34060634394787,-5.305155969696763)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (37.34060634394787,-5.305155969696763)
goal = (46.525488556259816,-7.809623436330959)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (46.525488556259816,-7.809623436330959)
goal = (49.09069105478878,-7.009139503222244)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 16 of 111"
path = []
start = (6.046166408883316,16.949081085403485)
goal = (8.829319566529882,23.938194265732612)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.829319566529882,23.938194265732612)
goal = (15.292541799163715,37.18668876444472)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 17 of 111"
path = []
start = (21.56062974524562,12.197790863257957)
goal = (33.826365873646,33.158570248128164)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 18 of 111"
path = []
start = (-2.196755055443802,-4.640956390275605)
goal = (-7.10493014758525,-20.87339867754878)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.10493014758525,-20.87339867754878)
goal = (-10.446953907558516,-20.5676160328243)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-10.446953907558516,-20.5676160328243)
goal = (-14.116596602967846,-21.456821089498764)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 19 of 111"
path = []
start = (22.67282649853972,-2.0441316954870175)
goal = (32.522850454587825,-12.44317120417847)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.522850454587825,-12.44317120417847)
goal = (34.60261252438518,-10.711926975713183)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (34.60261252438518,-10.711926975713183)
goal = (39.00107927197817,-13.229585317094298)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (39.00107927197817,-13.229585317094298)
goal = (45.32154983886714,-18.012790991093027)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (45.32154983886714,-18.012790991093027)
goal = (55.67343627385185,-25.010610240063844)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 20 of 111"
path = []
start = (24.27612393983445,11.54012567806128)
goal = (29.806621834846197,4.141977859056595)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 21 of 111"
path = []
start = (-5.386085248949975,1.7695784167601616)
goal = (-14.926388000626247,6.121710968609243)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-14.926388000626247,6.121710968609243)
goal = (-27.98470509580953,13.722103367140797)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-27.98470509580953,13.722103367140797)
goal = (-35.68439523692163,12.711722979343982)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-35.68439523692163,12.711722979343982)
goal = (-44.448619836960965,16.36757844673182)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 22 of 111"
path = []
start = (17.273441942326286,17.239408215477347)
goal = (25.99164467118044,30.715894228823736)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (25.99164467118044,30.715894228823736)
goal = (28.744482992486525,32.84967246684081)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 23 of 111"
path = []
start = (24.31918623261639,12.531483807384753)
goal = (40.79085399512077,24.84634395657401)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (40.79085399512077,24.84634395657401)
goal = (54.197235702067346,27.45701288181582)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 24 of 111"
path = []
start = (13.533997748952288,19.477077221165665)
goal = (21.49945495932569,33.26333011251233)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 25 of 111"
path = []
start = (5.88083731078131,22.067297556327595)
goal = (1.4008105857118025,26.329957644970527)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.4008105857118025,26.329957644970527)
goal = (-1.8984337364465205,22.557631588097188)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.8984337364465205,22.557631588097188)
goal = (-24.206463137690214,32.0461666043606)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 26 of 111"
path = []
start = (8.829319566529882,23.938194265732612)
goal = (-9.596761478577541,40.610310729642855)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.596761478577541,40.610310729642855)
goal = (-22.482393448371223,42.2301254903448)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 27 of 111"
path = []
start = (28.019275294085787,0.21183527659022872)
goal = (42.19053828814866,-11.312560109886029)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (42.19053828814866,-11.312560109886029)
goal = (46.97689969590684,-12.072301928629813)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (46.97689969590684,-12.072301928629813)
goal = (52.02614439946937,-14.8917642695844)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 28 of 111"
path = []
start = (30.476370136087766,12.565278773359374)
goal = (34.161930211154456,12.117252645073734)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (34.161930211154456,12.117252645073734)
goal = (38.45315771202482,9.862333764334288)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (38.45315771202482,9.862333764334288)
goal = (49.317633791944814,6.921552770575701)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (49.317633791944814,6.921552770575701)
goal = (61.02842828761591,2.8548660178404077)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 29 of 111"
path = []
start = (-10.716101819114712,-5.790846732300842)
goal = (-22.433350765570996,-9.908148416332097)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.433350765570996,-9.908148416332097)
goal = (-31.78431793527689,-12.639152989521847)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-31.78431793527689,-12.639152989521847)
goal = (-41.94729209988368,-19.30507748265547)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-41.94729209988368,-19.30507748265547)
goal = (-43.41639879367876,-21.533633852981552)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 30 of 111"
path = []
start = (7.489162736559486,26.953732420326396)
goal = (7.466581872843108,40.85059678640729)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 31 of 111"
path = []
start = (1.4008105857118025,26.329957644970527)
goal = (-4.129342979820009,23.498458474405375)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.129342979820009,23.498458474405375)
goal = (-8.012353432510054,21.856991452379468)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 32 of 111"
path = []
start = (33.369055825825825,13.329336493468212)
goal = (52.170767228014284,19.800908995091497)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (52.170767228014284,19.800908995091497)
goal = (59.83908674295374,20.88604117416265)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 33 of 111"
path = []
start = (-13.627453910016015,6.901480793530325)
goal = (-14.524213263318956,20.788172312713264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 34 of 111"
path = []
start = (34.161930211154456,12.117252645073734)
goal = (41.16984284645895,12.926169172108963)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (41.16984284645895,12.926169172108963)
goal = (58.405948830222385,18.80740414922176)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 35 of 111"
path = []
start = (-15.740213982210612,0.2232831704341791)
goal = (-36.44397525528636,-1.9996741772807098)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-36.44397525528636,-1.9996741772807098)
goal = (-38.63583332375394,-0.3414456080323802)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-38.63583332375394,-0.3414456080323802)
goal = (-50.901001260133704,9.940178397731373)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 36 of 111"
path = []
start = (15.404843877113436,-16.06746427850745)
goal = (9.269737141184685,-18.342631722717748)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.269737141184685,-18.342631722717748)
goal = (8.367835925234324,-19.635022840052493)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 37 of 111"
path = []
start = (-13.675305129300952,-8.326621554295293)
goal = (-14.902390500584396,-20.639351255053377)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-14.902390500584396,-20.639351255053377)
goal = (-18.486146993135762,-23.163938123730137)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 38 of 111"
path = []
start = (-14.926388000626247,6.121710968609243)
goal = (-24.522938612970663,24.520135878557404)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.522938612970663,24.520135878557404)
goal = (-31.335425576519324,29.4157306300284)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-31.335425576519324,29.4157306300284)
goal = (-35.87118100271634,35.006230788882085)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-35.87118100271634,35.006230788882085)
goal = (-38.17082792135827,37.457568037915486)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 39 of 111"
path = []
start = (21.707985775676846,-18.541255631180803)
goal = (30.742356155641914,-24.46907640774498)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (30.742356155641914,-24.46907640774498)
goal = (32.16574740828787,-24.20077770143574)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (32.16574740828787,-24.20077770143574)
goal = (34.20893154196358,-32.17380608164268)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (34.20893154196358,-32.17380608164268)
goal = (37.102771194275874,-37.31609523032241)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 40 of 111"
path = []
start = (-1.8984337364465205,22.557631588097188)
goal = (-25.709927332467338,34.30009424311937)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 41 of 111"
path = []
start = (-3.2493702991817273,-19.351727665748307)
goal = (-5.9135023044485635,-35.09735478428955)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.9135023044485635,-35.09735478428955)
goal = (1.1975234447051335,-45.712421321162765)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1975234447051335,-45.712421321162765)
goal = (3.2079449572245977,-50.327656596222084)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 42 of 111"
path = []
start = (36.56777809322233,1.3991839575175646)
goal = (61.60065856654349,-8.349850191988942)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 43 of 111"
path = []
start = (32.522850454587825,-12.44317120417847)
goal = (33.61361465633864,-16.871618629279084)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (33.61361465633864,-16.871618629279084)
goal = (41.19571056407345,-22.662532452046623)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (41.19571056407345,-22.662532452046623)
goal = (47.004449679954206,-26.14338084979884)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (47.004449679954206,-26.14338084979884)
goal = (56.07964499866793,-36.093490124717405)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 44 of 111"
path = []
start = (38.45315771202482,9.862333764334288)
goal = (60.25915997586448,6.427488261988543)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 45 of 111"
path = []
start = (12.991536446370787,-19.952079815468977)
goal = (5.5028717365457,-27.734840469106548)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 46 of 111"
path = []
start = (-4.129342979820009,23.498458474405375)
goal = (-17.575945606565618,34.88611526634663)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-17.575945606565618,34.88611526634663)
goal = (-20.82690359591925,36.81282167559771)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-20.82690359591925,36.81282167559771)
goal = (-24.38701622665227,38.01899005436669)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-24.38701622665227,38.01899005436669)
goal = (-29.722399472750247,40.56213996757331)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 47 of 111"
path = []
start = (34.29147795879126,23.755319399510476)
goal = (48.66708809299861,34.87446892224583)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 48 of 111"
path = []
start = (-7.10493014758525,-20.87339867754878)
goal = (-11.321478560216093,-22.597023087854637)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.321478560216093,-22.597023087854637)
goal = (-13.09020047379586,-26.664066556826597)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-13.09020047379586,-26.664066556826597)
goal = (-16.1590856221271,-39.88024625824541)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 49 of 111"
path = []
start = (37.34060634394787,-5.305155969696763)
goal = (47.26158595190297,-8.65055631062684)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (47.26158595190297,-8.65055631062684)
goal = (50.08648500397739,-10.75639606401738)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 50 of 111"
path = []
start = (25.99164467118044,30.715894228823736)
goal = (35.37853106045091,40.995798677312514)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 51 of 111"
path = []
start = (34.60261252438518,-10.711926975713183)
goal = (40.539019422231036,-16.041575878644437)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 52 of 111"
path = []
start = (41.16984284645895,12.926169172108963)
goal = (60.208032749648396,15.669878057665223)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 53 of 111"
path = []
start = (18.34494166691715,-22.777486965421275)
goal = (15.788328363885185,-31.322465587073168)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (15.788328363885185,-31.322465587073168)
goal = (12.975139031058262,-36.902343424968116)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (12.975139031058262,-36.902343424968116)
goal = (10.617437043802404,-39.365085114591835)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 54 of 111"
path = []
start = (33.61361465633864,-16.871618629279084)
goal = (39.46971499580154,-25.000935532755665)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (39.46971499580154,-25.000935532755665)
goal = (40.430674120321214,-25.669872589805195)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (40.430674120321214,-25.669872589805195)
goal = (44.31149645005365,-33.11757042688752)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (44.31149645005365,-33.11757042688752)
goal = (47.903143168449446,-39.02542496330555)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 55 of 111"
path = []
start = (-22.433350765570996,-9.908148416332097)
goal = (-35.76671734142991,-21.59414697163153)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-35.76671734142991,-21.59414697163153)
goal = (-42.26253562086499,-28.725813396120774)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 56 of 111"
path = []
start = (-10.446953907558516,-20.5676160328243)
goal = (-28.22976054037914,-36.81316348731383)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 57 of 111"
path = []
start = (-11.321478560216093,-22.597023087854637)
goal = (-21.72556962964765,-36.20045458422262)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-21.72556962964765,-36.20045458422262)
goal = (-23.87626691837289,-41.794061210487364)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 58 of 111"
path = []
start = (-23.55128969951076,12.864402059896854)
goal = (-31.786587759621057,21.273908444966473)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-31.786587759621057,21.273908444966473)
goal = (-39.59764667217985,28.596940403417015)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-39.59764667217985,28.596940403417015)
goal = (-40.366684958825324,35.76090242499395)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 59 of 111"
path = []
start = (39.00107927197817,-13.229585317094298)
goal = (57.474941807695814,-20.907367414354287)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (57.474941807695814,-20.907367414354287)
goal = (61.39286142474029,-22.253178329740837)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 60 of 111"
path = []
start = (-22.584853834100464,-16.49129586787894)
goal = (-24.344020742031315,-15.671074905324602)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.344020742031315,-15.671074905324602)
goal = (-27.447689269526528,-19.98443574391247)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 61 of 111"
path = []
start = (7.63906995094186,-25.761474129116586)
goal = (9.299270191717682,-40.854772295438984)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 62 of 111"
path = []
start = (30.742356155641914,-24.46907640774498)
goal = (28.11954211333608,-28.90634833591951)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (28.11954211333608,-28.90634833591951)
goal = (28.29035802620531,-39.0120498086417)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 63 of 111"
path = []
start = (-14.902390500584396,-20.639351255053377)
goal = (-18.720019442512168,-23.835709631043233)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 64 of 111"
path = []
start = (-22.928151277706753,-16.949757993103198)
goal = (-25.90224517483013,-21.463575078543673)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-25.90224517483013,-21.463575078543673)
goal = (-30.2354359671145,-24.633841821876945)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-30.2354359671145,-24.633841821876945)
goal = (-38.80070694349371,-27.69710137563539)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 65 of 111"
path = []
start = (17.377474198080307,-29.110750982456533)
goal = (10.899275124460893,-37.03968935830579)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 66 of 111"
path = []
start = (42.19053828814866,-11.312560109886029)
goal = (48.236544128608145,-15.91452007930895)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 67 of 111"
path = []
start = (32.16574740828787,-24.20077770143574)
goal = (39.04464840315349,-30.48331859019516)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (39.04464840315349,-30.48331859019516)
goal = (39.84210228942791,-30.695192240850382)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 68 of 111"
path = []
start = (-24.344020742031315,-15.671074905324602)
goal = (-36.66788671118828,-20.811185974884022)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-36.66788671118828,-20.811185974884022)
goal = (-46.700213642517234,-27.280496284867954)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 69 of 111"
path = []
start = (-13.09020047379586,-26.664066556826597)
goal = (-17.443682421646336,-41.366327184802316)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-17.443682421646336,-41.366327184802316)
goal = (-22.26277478884602,-46.65404283019652)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 70 of 111"
path = []
start = (-27.98470509580953,13.722103367140797)
goal = (-40.03305831255773,7.459861723325929)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 71 of 111"
path = []
start = (46.525488556259816,-7.809623436330959)
goal = (57.90310410856338,-8.985837647443155)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 72 of 111"
path = []
start = (15.788328363885185,-31.322465587073168)
goal = (10.8547146292184,-35.64837126699209)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.8547146292184,-35.64837126699209)
goal = (19.24724666827376,-47.304562374395836)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 73 of 111"
path = []
start = (47.26158595190297,-8.65055631062684)
goal = (55.24717785942727,-15.569411663655067)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 74 of 111"
path = []
start = (28.11954211333608,-28.90634833591951)
goal = (32.08256661389636,-38.622351058400916)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.08256661389636,-38.622351058400916)
goal = (31.816359704214705,-40.52075938786891)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 75 of 111"
path = []
start = (-29.179928460500502,16.75820286613773)
goal = (-46.27656362637376,11.190113044787395)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 76 of 111"
path = []
start = (41.19571056407345,-22.662532452046623)
goal = (49.367276170702524,-24.79121702951345)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (49.367276170702524,-24.79121702951345)
goal = (57.802355489469235,-27.926652171281415)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 77 of 111"
path = []
start = (-25.90224517483013,-21.463575078543673)
goal = (-34.43447280212651,-29.67888642519932)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-34.43447280212651,-29.67888642519932)
goal = (-34.7006843392346,-29.813096639023744)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-34.7006843392346,-29.813096639023744)
goal = (-40.81027645171193,-33.33512309150633)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-40.81027645171193,-33.33512309150633)
goal = (-50.8211303986571,-39.4020906238195)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 78 of 111"
path = []
start = (39.46971499580154,-25.000935532755665)
goal = (41.65276753398078,-32.49928603510723)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (41.65276753398078,-32.49928603510723)
goal = (45.177772687271144,-39.43396871612059)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 79 of 111"
path = []
start = (-31.78431793527689,-12.639152989521847)
goal = (-42.447935249820056,-19.345541532310826)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 80 of 111"
path = []
start = (52.170767228014284,19.800908995091497)
goal = (60.13600966828635,23.209799152924226)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (60.13600966828635,23.209799152924226)
goal = (63.03042169127278,23.926354499390904)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 81 of 111"
path = []
start = (41.585514959242744,35.30061471525115)
goal = (50.54202624726513,40.646788536761)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 82 of 111"
path = []
start = (-34.66605700879898,-4.261010587990313)
goal = (-41.90478429021135,-8.182320288841261)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-41.90478429021135,-8.182320288841261)
goal = (-51.306060426052845,-10.362060279052137)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 83 of 111"
path = []
start = (40.430674120321214,-25.669872589805195)
goal = (48.12076696919037,-35.100945102138574)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (48.12076696919037,-35.100945102138574)
goal = (60.98237147114183,-50.466661765099154)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 84 of 111"
path = []
start = (-5.129636822329701,-35.089590128698596)
goal = (-4.284787706323819,-45.994309295961564)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 85 of 111"
path = []
start = (-5.9135023044485635,-35.09735478428955)
goal = (-9.795710784688517,-51.343361095149945)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 86 of 111"
path = []
start = (-36.44397525528636,-1.9996741772807098)
goal = (-50.17159563224247,-2.2199707445750647)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 87 of 111"
path = []
start = (-24.522938612970663,24.520135878557404)
goal = (-29.869567384200966,34.20210077407442)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 88 of 111"
path = []
start = (34.20893154196358,-32.17380608164268)
goal = (34.69754706112245,-39.71584587294984)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 89 of 111"
path = []
start = (12.975139031058262,-36.902343424968116)
goal = (11.999978920782844,-43.757249927687944)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.999978920782844,-43.757249927687944)
goal = (11.77561972653649,-45.763122333013634)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 90 of 111"
path = []
start = (-17.575945606565618,34.88611526634663)
goal = (-21.977937326204145,34.76253442216068)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 91 of 111"
path = []
start = (-31.786587759621057,21.273908444966473)
goal = (-44.79313224201687,26.211342819512446)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-44.79313224201687,26.211342819512446)
goal = (-47.841231635172484,26.254667078418343)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 92 of 111"
path = []
start = (-35.68439523692163,12.711722979343982)
goal = (-44.81965618978823,7.95634620064606)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 93 of 111"
path = []
start = (39.04464840315349,-30.48331859019516)
goal = (44.94139563642588,-42.243934449774066)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 94 of 111"
path = []
start = (-30.2354359671145,-24.633841821876945)
goal = (-32.27701686140454,-35.15854325253453)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-32.27701686140454,-35.15854325253453)
goal = (-41.768083660750435,-50.73442985948465)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 95 of 111"
path = []
start = (-20.82690359591925,36.81282167559771)
goal = (-25.240697877017233,39.906620187593354)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 96 of 111"
path = []
start = (-35.76671734142991,-21.59414697163153)
goal = (-46.73059851621215,-25.16356495455856)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 97 of 111"
path = []
start = (-21.72556962964765,-36.20045458422262)
goal = (-33.793166930061744,-43.53636881250331)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-33.793166930061744,-43.53636881250331)
goal = (-36.48449477976807,-46.63900210195892)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 98 of 111"
path = []
start = (32.08256661389636,-38.622351058400916)
goal = (32.614247984308875,-43.90510652539332)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 99 of 111"
path = []
start = (11.999978920782844,-43.757249927687944)
goal = (15.18227916232334,-49.7433478257636)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 100 of 111"
path = []
start = (-31.335425576519324,29.4157306300284)
goal = (-39.285867703687195,33.497029925785846)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 101 of 111"
path = []
start = (-34.43447280212651,-29.67888642519932)
goal = (-38.96919408182387,-32.63259477945171)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-38.96919408182387,-32.63259477945171)
goal = (-49.86090273262245,-48.92455977598853)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 102 of 111"
path = []
start = (-34.7006843392346,-29.813096639023744)
goal = (-40.022222457540515,-34.74177928730348)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-40.022222457540515,-34.74177928730348)
goal = (-42.342007966203994,-38.37112642287931)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-42.342007966203994,-38.37112642287931)
goal = (-49.62978410821379,-45.757913792387164)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 103 of 111"
path = []
start = (-41.94729209988368,-19.30507748265547)
goal = (-46.98547163308918,-23.515159089198463)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 104 of 111"
path = []
start = (-39.59764667217985,28.596940403417015)
goal = (-40.14642010962771,35.835948165979836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 105 of 111"
path = []
start = (-32.27701686140454,-35.15854325253453)
goal = (-43.90910559769502,-50.92843699093386)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 106 of 111"
path = []
start = (-42.358124936297024,28.318468944330412)
goal = (-45.8195286824369,36.650178314415875)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 107 of 111"
path = []
start = (-35.87118100271634,35.006230788882085)
goal = (-40.14779966163995,41.44473584183326)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 108 of 111"
path = []
start = (-44.79313224201687,26.211342819512446)
goal = (-48.09705090941673,27.376877214145942)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 109 of 111"
path = []
start = (-40.022222457540515,-34.74177928730348)
goal = (-47.74747986356131,-42.46469042658268)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 110 of 111"
path = []
start = (-33.793166930061744,-43.53636881250331)
goal = (-38.06416421274917,-50.120033941586286)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
print "Path 111 of 111"
path = []
start = (-42.342007966203994,-38.37112642287931)
goal = (-47.221875728916785,-48.54934718961852)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-47.221875728916785,-48.54934718961852)
goal = (-48.17668902069287,-50.77973865684312)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-1.txt', 'a+')
f.write(pathStr)
f.close
content += pathStr
#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
