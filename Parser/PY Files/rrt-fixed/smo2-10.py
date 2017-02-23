#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""
@brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai

@license: MIT

"""

import shapely
import datetime
from shapely.geometry import Polygon, LineString, Point, MultiPoint, GeometryCollection
import matplotlib.pyplot as plt
from ast import literal_eval

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


def rrtpath(obstacles,startcoord,goalcoord,randAreas):
    rrt = RRT(start=startcoord, goal=goalcoord,randArea = randAreas, obstacleList=obstacles)
    path= rrt.Planning(animation=False)
#    rrt.DrawGaph()
#    plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')
#    print path
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-y')
    smoothiePath.reverse()
    #print smoothiePath
    return smoothiePath

obstacleList = []
rand = (-8, 10)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 145"
path = []
start = (-118.0799988517641,-43.20309583428502)
goal = (-116.34243600064363,-30.18985832976719)
print "     Node 1 and 2 of 9"
path += rrtpath(obstacleList,start,goal,rand)
start = (-116.34243600064363,-30.18985832976719)
goal = (-133.13191248019285,-35.478540374291356)
print "     Node 2 and 3 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-133.13191248019285,-35.478540374291356)
goal = (-132.32133213777803,-57.19856474388595)
print "     Node 3 and 4 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-132.32133213777803,-57.19856474388595)
goal = (-116.01848327593993,-66.79532206705164)
print "     Node 4 and 5 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-116.01848327593993,-66.79532206705164)
goal = (-84.67559502464593,-63.21295478031806)
print "     Node 5 and 6 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-84.67559502464593,-63.21295478031806)
goal = (-87.61625992719723,-97.46242390178166)
print "     Node 6 and 7 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-87.61625992719723,-97.46242390178166)
goal = (1.6436153777304412,-32.894169177396066)
print "     Node 7 and 8 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.6436153777304412,-32.894169177396066)
goal = (78.93431998432317,-29.822046166942016)
print "     Node 8 and 9 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 145"
path = []
start = (-116.34243600064363,-30.18985832976719)
goal = (-118.61817617015578,-22.523814639762463)
print "     Node 1 and 2 of 9"
path += rrtpath(obstacleList,start,goal,rand)
start = (-118.61817617015578,-22.523814639762463)
goal = (-127.93788792304773,-17.652902135589926)
print "     Node 2 and 3 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-127.93788792304773,-17.652902135589926)
goal = (-99.64007667721116,-25.92952042560114)
print "     Node 3 and 4 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-99.64007667721116,-25.92952042560114)
goal = (-97.41897205591177,-28.154854035613482)
print "     Node 4 and 5 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-97.41897205591177,-28.154854035613482)
goal = (-72.0095535125012,-20.952951513758848)
print "     Node 5 and 6 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-72.0095535125012,-20.952951513758848)
goal = (-56.848398074656814,-55.365034084899634)
print "     Node 6 and 7 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-56.848398074656814,-55.365034084899634)
goal = (6.364021825461464,-56.65615477510485)
print "     Node 7 and 8 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.364021825461464,-56.65615477510485)
goal = (77.72234839068517,-95.07512476145297)
print "     Node 8 and 9 of 9"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 145"
path = []
start = (-118.61817617015578,-22.523814639762463)
goal = (-124.65468468154927,-12.646857229187788)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-124.65468468154927,-12.646857229187788)
goal = (-128.5163854305825,-4.822539824919772)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-128.5163854305825,-4.822539824919772)
goal = (-92.75407859792928,2.5986849187442544)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-92.75407859792928,2.5986849187442544)
goal = (-75.7875083595418,-5.394723995112514)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-75.7875083595418,-5.394723995112514)
goal = (-52.013901426541224,-0.9734825385189367)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-52.013901426541224,-0.9734825385189367)
goal = (-67.99142521044618,90.2703358461676)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-67.99142521044618,90.2703358461676)
goal = (39.457578354014544,107.38386334340515)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 145"
path = []
start = (-133.13191248019285,-35.478540374291356)
goal = (-135.0282026160119,-33.79151675444518)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-135.0282026160119,-33.79151675444518)
goal = (-153.901937808911,-18.78453553048348)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-153.901937808911,-18.78453553048348)
goal = (-146.9602768786348,-66.1103515180927)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-146.9602768786348,-66.1103515180927)
goal = (-179.29697412121462,-20.03924172546968)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-179.29697412121462,-20.03924172546968)
goal = (-48.23759872195566,-45.51574704536793)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-48.23759872195566,-45.51574704536793)
goal = (-11.969162831903418,-105.66526765735125)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-11.969162831903418,-105.66526765735125)
goal = (79.78598452529604,29.96388892085028)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 145"
path = []
start = (-127.93788792304773,-17.652902135589926)
goal = (-136.27547559358737,-7.398200247473994)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-136.27547559358737,-7.398200247473994)
goal = (-125.63444790535318,13.250737422560547)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-125.63444790535318,13.250737422560547)
goal = (-146.6597935673014,28.273665857061133)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-146.6597935673014,28.273665857061133)
goal = (-73.47565953863273,37.20115886331578)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-73.47565953863273,37.20115886331578)
goal = (-45.239306861897774,78.99855043945257)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-45.239306861897774,78.99855043945257)
goal = (59.425409728759405,87.57860496066615)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 145"
path = []
start = (-124.65468468154927,-12.646857229187788)
goal = (-128.5078343130727,4.770752083774056)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-128.5078343130727,4.770752083774056)
goal = (-149.91706535184846,7.808679792392979)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-149.91706535184846,7.808679792392979)
goal = (-96.75450853717318,24.14242749576107)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-96.75450853717318,24.14242749576107)
goal = (-54.82378192484157,22.454533364161335)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-54.82378192484157,22.454533364161335)
goal = (-103.07044262286959,118.5956943271961)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-103.07044262286959,118.5956943271961)
goal = (86.86904235680936,7.118189361793014)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 145"
path = []
start = (-135.0282026160119,-33.79151675444518)
goal = (-166.67054195832628,-27.945667369679626)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-166.67054195832628,-27.945667369679626)
goal = (-167.86526913790505,-36.18903289314218)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-167.86526913790505,-36.18903289314218)
goal = (-178.06743462430003,-8.97155284834406)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-178.06743462430003,-8.97155284834406)
goal = (-108.00166831863467,-117.82100017533148)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-108.00166831863467,-117.82100017533148)
goal = (-144.0858295564318,-176.0498360149448)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-144.0858295564318,-176.0498360149448)
goal = (89.08835447582015,-79.12044915351899)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 145"
path = []
start = (-132.32133213777803,-57.19856474388595)
goal = (-130.9044693818177,-71.72393073114853)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-130.9044693818177,-71.72393073114853)
goal = (-162.455275152176,-54.56445436793888)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-162.455275152176,-54.56445436793888)
goal = (-99.97671587965522,-83.52952763224923)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-99.97671587965522,-83.52952763224923)
goal = (-113.85230835372985,-121.64651522128392)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-113.85230835372985,-121.64651522128392)
goal = (-150.9050696109323,-180.2313067797798)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-150.9050696109323,-180.2313067797798)
goal = (84.63134661653027,-99.23721896344505)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 145"
path = []
start = (-99.64007667721116,-25.92952042560114)
goal = (-80.0417130500942,-20.754955329639188)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-80.0417130500942,-20.754955329639188)
goal = (-80.33522998112738,7.439113370401429)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-80.33522998112738,7.439113370401429)
goal = (-40.63167937802211,-9.00901698451645)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-40.63167937802211,-9.00901698451645)
goal = (9.59244893939416,-44.622263414625706)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.59244893939416,-44.622263414625706)
goal = (94.2940438077469,-16.964864622478075)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 145"
path = []
start = (-128.5163854305825,-4.822539824919772)
goal = (-118.64916379776531,26.781840590752722)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-118.64916379776531,26.781840590752722)
goal = (-136.64513717213146,39.46994364527944)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-136.64513717213146,39.46994364527944)
goal = (-104.42173368847398,71.62528375605814)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-104.42173368847398,71.62528375605814)
goal = (-59.945285853797515,110.54669396368843)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-59.945285853797515,110.54669396368843)
goal = (14.108154234424433,155.67480838723043)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 145"
path = []
start = (-153.901937808911,-18.78453553048348)
goal = (-165.9020814628882,-21.51173657300464)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-165.9020814628882,-21.51173657300464)
goal = (-188.07652400224225,-33.302339802747696)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-188.07652400224225,-33.302339802747696)
goal = (-150.14596847726017,74.79411136583838)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-150.14596847726017,74.79411136583838)
goal = (-111.43938107550983,135.91830445139846)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-111.43938107550983,135.91830445139846)
goal = (35.02468512622491,134.9968944533308)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 145"
path = []
start = (-136.27547559358737,-7.398200247473994)
goal = (-166.26410056121244,2.9803780497311436)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-166.26410056121244,2.9803780497311436)
goal = (-170.70826644132765,31.919169993195595)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-170.70826644132765,31.919169993195595)
goal = (-72.6632640437582,54.57123268473049)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-72.6632640437582,54.57123268473049)
goal = (-55.40749719781624,108.11226349024219)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-55.40749719781624,108.11226349024219)
goal = (53.64213050910786,110.73033226946552)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 145"
path = []
start = (-128.5078343130727,4.770752083774056)
goal = (-108.42017734455078,22.999321696240315)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-108.42017734455078,22.999321696240315)
goal = (-170.79593086920138,32.39387738990084)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-170.79593086920138,32.39387738990084)
goal = (-57.631959124498735,37.059842040150016)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-57.631959124498735,37.059842040150016)
goal = (-50.50396181680435,110.13127202454308)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-50.50396181680435,110.13127202454308)
goal = (22.150444151282358,153.22664174255505)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 145"
path = []
start = (-166.67054195832628,-27.945667369679626)
goal = (-173.2809416363128,-25.871935335501917)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-173.2809416363128,-25.871935335501917)
goal = (-198.2073136441465,-37.13700991310054)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-198.2073136441465,-37.13700991310054)
goal = (-171.28775920520556,80.53204350384607)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-171.28775920520556,80.53204350384607)
goal = (-128.6641356204177,-186.009415554429)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-128.6641356204177,-186.009415554429)
goal = (74.46810102580548,67.74016900563987)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 145"
path = []
start = (-130.9044693818177,-71.72393073114853)
goal = (-142.32421401633334,-100.22980127021412)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-142.32421401633334,-100.22980127021412)
goal = (-117.07415105915464,-101.6968407248252)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-117.07415105915464,-101.6968407248252)
goal = (-98.58261550314297,-122.05253931318403)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-98.58261550314297,-122.05253931318403)
goal = (-109.73796355228723,-187.90527039478326)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-109.73796355228723,-187.90527039478326)
goal = (95.89672350067934,-55.337894056369436)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 145"
path = []
start = (-116.01848327593993,-66.79532206705164)
goal = (-98.35879197774551,-57.860689013598744)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-98.35879197774551,-57.860689013598744)
goal = (-97.51414551676905,-89.78636806620281)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-97.51414551676905,-89.78636806620281)
goal = (-88.44920846537825,-119.14869185187685)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-88.44920846537825,-119.14869185187685)
goal = (-51.9801835635358,-161.6624617976558)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-51.9801835635358,-161.6624617976558)
goal = (94.47214565349913,-33.62360246649749)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 145"
path = []
start = (-97.41897205591177,-28.154854035613482)
goal = (-59.570432642129134,-19.447656721975818)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-59.570432642129134,-19.447656721975818)
goal = (-30.872926879322137,-29.737362258608158)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-30.872926879322137,-29.737362258608158)
goal = (4.946251631287794,-80.3884922367759)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.946251631287794,-80.3884922367759)
goal = (94.73366589248133,-24.80983642400139)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 145"
path = []
start = (-92.75407859792928,2.5986849187442544)
goal = (-66.34510480720661,9.907645322248499)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-66.34510480720661,9.907645322248499)
goal = (-55.2877411057176,33.240978016820236)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-55.2877411057176,33.240978016820236)
goal = (16.776457382257178,39.46270886517286)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (16.776457382257178,39.46270886517286)
goal = (71.82495079291556,75.66551044293703)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 145"
path = []
start = (-146.9602768786348,-66.1103515180927)
goal = (-169.61336719277898,-88.07574693091698)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-169.61336719277898,-88.07574693091698)
goal = (-158.19469010839526,-143.8303281690189)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-158.19469010839526,-143.8303281690189)
goal = (-72.59313457551575,-176.9114405839569)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-72.59313457551575,-176.9114405839569)
goal = (99.9178331118024,-103.37796496936052)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 145"
path = []
start = (-125.63444790535318,13.250737422560547)
goal = (-111.93512441984615,56.05071311103245)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-111.93512441984615,56.05071311103245)
goal = (-67.2753965853544,55.1503517218236)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-67.2753965853544,55.1503517218236)
goal = (-45.787126350474324,109.44713652800147)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-45.787126350474324,109.44713652800147)
goal = (61.446195626559756,104.63225996576057)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 145"
path = []
start = (-149.91706535184846,7.808679792392979)
goal = (-168.90570239560918,47.9713882246088)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-168.90570239560918,47.9713882246088)
goal = (-138.33844082607865,90.99044375505281)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-138.33844082607865,90.99044375505281)
goal = (-160.59726561559557,149.57148632384872)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-160.59726561559557,149.57148632384872)
goal = (66.15979265105545,96.44532585086768)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 145"
path = []
start = (-167.86526913790505,-36.18903289314218)
goal = (-161.12976393880257,-94.71182813314094)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-161.12976393880257,-94.71182813314094)
goal = (-194.17819640045914,-138.3078297838234)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-194.17819640045914,-138.3078297838234)
goal = (-74.23007217892686,-180.83956915807568)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-74.23007217892686,-180.83956915807568)
goal = (93.56055621034903,6.203157092020831)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 145"
path = []
start = (-162.455275152176,-54.56445436793888)
goal = (-199.09813427885044,-100.81883438104846)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-199.09813427885044,-100.81883438104846)
goal = (-157.0553218866224,-150.53732164960678)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-157.0553218866224,-150.53732164960678)
goal = (-84.30393870820913,-189.0889365187624)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-84.30393870820913,-189.0889365187624)
goal = (95.1691533838287,10.967018732554777)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 145"
path = []
start = (-80.0417130500942,-20.754955329639188)
goal = (-52.35515257465565,-12.1757536558504)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-52.35515257465565,-12.1757536558504)
goal = (-44.11786429423154,13.46273464845953)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-44.11786429423154,13.46273464845953)
goal = (9.421612044027086,-74.8161516006935)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.421612044027086,-74.8161516006935)
goal = (95.60360830906097,22.703679293122775)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 145"
path = []
start = (-118.64916379776531,26.781840590752722)
goal = (-121.00828923893032,63.0317239194859)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-121.00828923893032,63.0317239194859)
goal = (-66.32355160562403,54.74972225987412)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-66.32355160562403,54.74972225987412)
goal = (-97.10297603595745,143.7033481114728)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-97.10297603595745,143.7033481114728)
goal = (32.78656327053048,160.69465517117123)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 145"
path = []
start = (-165.9020814628882,-21.51173657300464)
goal = (-199.75690953484997,42.12985628277545)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-199.75690953484997,42.12985628277545)
goal = (-193.30053420384036,88.43350148591554)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-193.30053420384036,88.43350148591554)
goal = (-198.5798536618161,143.10885257428822)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-198.5798536618161,143.10885257428822)
goal = (103.18747785797422,-1.933748914842596)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 145"
path = []
start = (-166.26410056121244,2.9803780497311436)
goal = (-168.7829612017341,74.46740551356163)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-168.7829612017341,74.46740551356163)
goal = (-176.58736382240764,98.02921925131074)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-176.58736382240764,98.02921925131074)
goal = (-186.3225245420268,147.6246694529391)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-186.3225245420268,147.6246694529391)
goal = (24.59630078188536,171.67776909070477)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 145"
path = []
start = (-108.42017734455078,22.999321696240315)
goal = (-90.21952491926922,50.551614123075694)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-90.21952491926922,50.551614123075694)
goal = (-80.35889702507268,73.28119241003219)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-80.35889702507268,73.28119241003219)
goal = (-17.963014154169286,92.06496556672545)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-17.963014154169286,92.06496556672545)
goal = (78.3057570502092,95.16964756037686)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 145"
path = []
start = (-173.2809416363128,-25.871935335501917)
goal = (-108.19480174981982,-102.89786842314604)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-108.19480174981982,-102.89786842314604)
goal = (-114.44894963619716,-141.13137213652195)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-114.44894963619716,-141.13137213652195)
goal = (-150.55018047551303,150.54755345497153)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-150.55018047551303,150.54755345497153)
goal = (105.93128241150424,-49.04553306591541)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 145"
path = []
start = (-142.32421401633334,-100.22980127021412)
goal = (-125.6291152682389,-117.52637063898996)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-125.6291152682389,-117.52637063898996)
goal = (-127.07045022916796,-153.55543767240403)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-127.07045022916796,-153.55543767240403)
goal = (-42.11686937775886,-160.53328714954188)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-42.11686937775886,-160.53328714954188)
goal = (104.53292111888862,-81.85301238091233)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 145"
path = []
start = (-98.35879197774551,-57.860689013598744)
goal = (-69.39977743354305,-68.17445046829113)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-69.39977743354305,-68.17445046829113)
goal = (-41.037154448131474,-65.50405289915739)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-41.037154448131474,-65.50405289915739)
goal = (-14.019786413596648,-119.18071904267143)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-14.019786413596648,-119.18071904267143)
goal = (104.52622821360893,-73.93569656123304)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 145"
path = []
start = (-84.67559502464593,-63.21295478031806)
goal = (-62.889009628581675,-50.4857634085231)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-62.889009628581675,-50.4857634085231)
goal = (-66.00904398332253,-111.0296213132937)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-66.00904398332253,-111.0296213132937)
goal = (-1.4547971067839285,-112.39325454387807)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.4547971067839285,-112.39325454387807)
goal = (106.58867868676725,-57.683868570349034)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 145"
path = []
start = (-72.0095535125012,-20.952951513758848)
goal = (-29.363056786837035,-44.09615436434632)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-29.363056786837035,-44.09615436434632)
goal = (22.498532762985064,-47.59473978499548)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (22.498532762985064,-47.59473978499548)
goal = (106.56764100590607,-8.253625771570285)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 145"
path = []
start = (-75.7875083595418,-5.394723995112514)
goal = (-26.892005366582595,0.6723443464645129)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-26.892005366582595,0.6723443464645129)
goal = (31.837254906131648,-5.6384346189536245)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (31.837254906131648,-5.6384346189536245)
goal = (108.03132618494857,-15.497506670226358)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 145"
path = []
start = (-179.29697412121462,-20.03924172546968)
goal = (-140.82840659362034,110.6734843120995)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-140.82840659362034,110.6734843120995)
goal = (-181.09017567228966,153.8462650529625)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-181.09017567228966,153.8462650529625)
goal = (69.43134640552404,115.64727636577959)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 145"
path = []
start = (-146.6597935673014,28.273665857061133)
goal = (-102.22517195220959,92.37169156450204)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-102.22517195220959,92.37169156450204)
goal = (-116.39690776483987,146.03426936146963)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-116.39690776483987,146.03426936146963)
goal = (44.03137535681569,163.8653629943002)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 145"
path = []
start = (-96.75450853717318,24.14242749576107)
goal = (-50.19032158013789,53.25764306433766)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-50.19032158013789,53.25764306433766)
goal = (-72.46635637891812,140.2041275062614)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-72.46635637891812,140.2041275062614)
goal = (78.29885634327854,98.89287413688339)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 145"
path = []
start = (-178.06743462430003,-8.97155284834406)
goal = (-82.79059190663082,81.47205763909227)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-82.79059190663082,81.47205763909227)
goal = (-162.4210902005729,153.98745152180322)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-162.4210902005729,153.98745152180322)
goal = (92.23679463409337,72.24944004444978)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 145"
path = []
start = (-99.97671587965522,-83.52952763224923)
goal = (-95.15240058870171,-148.98436915914522)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-95.15240058870171,-148.98436915914522)
goal = (-28.09893132791825,-143.73171021180355)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-28.09893132791825,-143.73171021180355)
goal = (112.0843293678717,-83.88988410936938)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 145"
path = []
start = (-80.33522998112738,7.439113370401429)
goal = (-36.805498589782104,34.14652395029992)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-36.805498589782104,34.14652395029992)
goal = (36.51194370689029,11.7208406247766)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (36.51194370689029,11.7208406247766)
goal = (90.81440358971003,79.69205909845766)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 145"
path = []
start = (-136.64513717213146,39.46994364527944)
goal = (-93.22349915053314,94.58884302245508)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-93.22349915053314,94.58884302245508)
goal = (-127.50555398109017,148.51519122199414)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-127.50555398109017,148.51519122199414)
goal = (53.23250597761367,158.15983004260505)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 145"
path = []
start = (-188.07652400224225,-33.302339802747696)
goal = (-179.77852890492105,-164.04361002472254)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-179.77852890492105,-164.04361002472254)
goal = (-158.6858703454991,154.99604358827253)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-158.6858703454991,154.99604358827253)
goal = (85.32143044055738,112.5898742738384)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 145"
path = []
start = (-170.70826644132765,31.919169993195595)
goal = (-114.38588921975095,107.7284930034445)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-114.38588921975095,107.7284930034445)
goal = (-159.77146307445773,158.22329917679644)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-159.77146307445773,158.22329917679644)
goal = (38.48805368490201,184.17974791373013)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 145"
path = []
start = (-170.79593086920138,32.39387738990084)
goal = (-137.612841967048,128.55423344261504)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-137.612841967048,128.55423344261504)
goal = (-162.114987586278,159.42535100242947)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-162.114987586278,159.42535100242947)
goal = (69.49983669111703,151.16638276933338)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 145"
path = []
start = (-198.2073136441465,-37.13700991310054)
goal = (-194.62516531181348,-172.5446891446322)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-194.62516531181348,-172.5446891446322)
goal = (-185.07533256761005,164.54356686090512)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-185.07533256761005,164.54356686090512)
goal = (103.67235998966561,75.39704430922012)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 145"
path = []
start = (-117.07415105915464,-101.6968407248252)
goal = (-114.08653192316764,-158.55028600943598)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-114.08653192316764,-158.55028600943598)
goal = (-65.19973764532122,-192.20225937955985)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-65.19973764532122,-192.20225937955985)
goal = (116.95664886908929,-130.48719760268034)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 145"
path = []
start = (-97.51414551676905,-89.78636806620281)
goal = (-24.509698637757765,-89.78903994230545)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.509698637757765,-89.78903994230545)
goal = (1.7790865945402174,-109.18300808891831)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.7790865945402174,-109.18300808891831)
goal = (120.93240958471165,-86.20268905584729)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 145"
path = []
start = (-59.570432642129134,-19.447656721975818)
goal = (-20.053466830971615,-38.07569956843318)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-20.053466830971615,-38.07569956843318)
goal = (26.070427080736692,-44.98341015788512)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (26.070427080736692,-44.98341015788512)
goal = (123.70783037631924,-20.698275498102362)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 145"
path = []
start = (-66.34510480720661,9.907645322248499)
goal = (-20.693461661185182,23.813965837188306)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-20.693461661185182,23.813965837188306)
goal = (38.2778996589544,17.470539484437438)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (38.2778996589544,17.470539484437438)
goal = (109.24098830338073,76.60134071889507)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 145"
path = []
start = (-169.61336719277898,-88.07574693091698)
goal = (-157.57869723154423,-168.2909224893661)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-157.57869723154423,-168.2909224893661)
goal = (-40.829471800561436,-198.60795317414542)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-40.829471800561436,-198.60795317414542)
goal = (123.10691133288464,-113.28942834175666)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 145"
path = []
start = (-111.93512441984615,56.05071311103245)
goal = (-79.17621508873233,90.87474250784544)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-79.17621508873233,90.87474250784544)
goal = (-70.44682988153923,141.48516204744254)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-70.44682988153923,141.48516204744254)
goal = (73.61010995035366,151.95344442249967)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 145"
path = []
start = (-168.90570239560918,47.9713882246088)
goal = (-105.96542327923694,114.41834304223914)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-105.96542327923694,114.41834304223914)
goal = (-125.41168143350494,162.2863817429551)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-125.41168143350494,162.2863817429551)
goal = (47.91792189577629,196.3335201405547)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 145"
path = []
start = (-161.12976393880257,-94.71182813314094)
goal = (-171.8690972743541,-177.08295737885157)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-171.8690972743541,-177.08295737885157)
goal = (-17.714746544015327,-165.5419245290445)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-17.714746544015327,-165.5419245290445)
goal = (126.08050513086482,-92.56917728581615)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 145"
path = []
start = (-199.09813427885044,-100.81883438104846)
goal = (-148.9493588200948,-177.6942592727827)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-148.9493588200948,-177.6942592727827)
goal = (-28.45880042685249,-193.4381292642877)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-28.45880042685249,-193.4381292642877)
goal = (126.6366302313225,-111.2245544324133)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 145"
path = []
start = (-52.35515257465565,-12.1757536558504)
goal = (-24.622911253985137,-51.93318447206369)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.622911253985137,-51.93318447206369)
goal = (18.68669724412578,-64.80108785089257)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (18.68669724412578,-64.80108785089257)
goal = (128.49679420070498,-19.30474033507278)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 145"
path = []
start = (-121.00828923893032,63.0317239194859)
goal = (-92.10115711832891,107.84132818141836)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-92.10115711832891,107.84132818141836)
goal = (-49.03993630517144,138.26761252436853)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-49.03993630517144,138.26761252436853)
goal = (70.16224693638611,161.15477872837096)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 145"
path = []
start = (-199.75690953484997,42.12985628277545)
goal = (-186.4112263574258,138.43417451940377)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-186.4112263574258,138.43417451940377)
goal = (-175.77122787024058,187.5543123925018)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-175.77122787024058,187.5543123925018)
goal = (75.1685163239901,150.97625976447205)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 145"
path = []
start = (-168.7829612017341,74.46740551356163)
goal = (-148.92987284574338,142.42415050555275)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-148.92987284574338,142.42415050555275)
goal = (-109.12671291621821,168.33811304618507)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-109.12671291621821,168.33811304618507)
goal = (62.418542511469184,180.69503339098986)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 145"
path = []
start = (-90.21952491926922,50.551614123075694)
goal = (-66.76820604585129,85.7575673842984)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-66.76820604585129,85.7575673842984)
goal = (-45.19213026775262,141.35097246352888)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-45.19213026775262,141.35097246352888)
goal = (100.01094698006779,111.4753721007861)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 145"
path = []
start = (-108.19480174981982,-102.89786842314604)
goal = (-106.14295321991602,-168.14893194291315)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-106.14295321991602,-168.14893194291315)
goal = (1.2896367819578245,-112.4240374096587)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.2896367819578245,-112.4240374096587)
goal = (124.4343581006487,-55.00389153555952)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 145"
path = []
start = (-125.6291152682389,-117.52637063898996)
goal = (-127.16428431320863,-170.7353991926725)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-127.16428431320863,-170.7353991926725)
goal = (4.131645218130188,-128.9691173920473)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.131645218130188,-128.9691173920473)
goal = (117.32377874519926,-186.6467612765378)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 145"
path = []
start = (-69.39977743354305,-68.17445046829113)
goal = (-21.213053052203094,-97.3596225427273)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-21.213053052203094,-97.3596225427273)
goal = (13.493786673127545,-94.82472183723796)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (13.493786673127545,-94.82472183723796)
goal = (134.60438482394756,-50.33783532682713)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 145"
path = []
start = (-62.889009628581675,-50.4857634085231)
goal = (-4.303949479074987,-55.463124756237164)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.303949479074987,-55.463124756237164)
goal = (22.61308093391108,-69.8423465078875)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (22.61308093391108,-69.8423465078875)
goal = (136.7455793797589,-37.71145994594622)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 145"
path = []
start = (-87.61625992719723,-97.46242390178166)
goal = (-74.90911382596374,-167.40648350882074)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-74.90911382596374,-167.40648350882074)
goal = (13.699469752153334,-94.53503589482412)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (13.699469752153334,-94.53503589482412)
goal = (134.5308472383075,-116.5890715620514)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 145"
path = []
start = (-56.848398074656814,-55.365034084899634)
goal = (26.99238223735199,-51.32881036746312)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (26.99238223735199,-51.32881036746312)
goal = (139.7859457315019,-71.78409572055696)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 145"
path = []
start = (-52.013901426541224,-0.9734825385189367)
goal = (33.933337240999066,-47.15016188817458)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (33.933337240999066,-47.15016188817458)
goal = (113.31249439951574,65.51671772984315)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 145"
path = []
start = (-48.23759872195566,-45.51574704536793)
goal = (36.40833269117988,-73.82816266380514)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (36.40833269117988,-73.82816266380514)
goal = (136.8221133910293,-2.94873408093045)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 145"
path = []
start = (-73.47565953863273,37.20115886331578)
goal = (-12.83494114874975,116.26466200981372)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-12.83494114874975,116.26466200981372)
goal = (109.8284918708946,96.03697110339454)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 145"
path = []
start = (-54.82378192484157,22.454533364161335)
goal = (42.586831980822126,46.87049119599814)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (42.586831980822126,46.87049119599814)
goal = (113.47956989638936,84.9187951172201)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 145"
path = []
start = (-108.00166831863467,-117.82100017533148)
goal = (-11.874734379273292,-185.89571067165394)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.874734379273292,-185.89571067165394)
goal = (120.55057916207176,-197.15096885110216)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 145"
path = []
start = (-113.85230835372985,-121.64651522128392)
goal = (6.501605537141728,-159.90615445814007)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.501605537141728,-159.90615445814007)
goal = (130.73649424968,-189.8405676371804)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 145"
path = []
start = (-40.63167937802211,-9.00901698451645)
goal = (47.58298493523489,1.3591733575547664)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (47.58298493523489,1.3591733575547664)
goal = (118.9034603565716,79.50736242258836)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 145"
path = []
start = (-104.42173368847398,71.62528375605814)
goal = (-79.66934802872375,161.37866508436878)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-79.66934802872375,161.37866508436878)
goal = (70.52916517186355,173.4918826676576)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 145"
path = []
start = (-150.14596847726017,74.79411136583838)
goal = (-133.21909200643782,185.51805725126422)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-133.21909200643782,185.51805725126422)
goal = (66.61863376594948,192.86170165305435)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 145"
path = []
start = (-72.6632640437582,54.57123268473049)
goal = (9.91070729298508,103.65340190210736)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.91070729298508,103.65340190210736)
goal = (76.82810530573983,172.1427502116785)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 145"
path = []
start = (-57.631959124498735,37.059842040150016)
goal = (4.552382889480867,118.1907176697809)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.552382889480867,118.1907176697809)
goal = (122.00878421390814,77.48381225794009)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 145"
path = []
start = (-171.28775920520556,80.53204350384607)
goal = (-155.51621642096012,188.36286514766385)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-155.51621642096012,188.36286514766385)
goal = (69.23208653684861,192.8477660298695)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 145"
path = []
start = (-98.58261550314297,-122.05253931318403)
goal = (14.615069682349144,-130.70299846555002)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (14.615069682349144,-130.70299846555002)
goal = (140.9212183979559,-123.71732584753326)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 145"
path = []
start = (-88.44920846537825,-119.14869185187685)
goal = (2.3674900833144363,-198.23203467194156)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.3674900833144363,-198.23203467194156)
goal = (141.82705766355446,-132.97231828665747)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 145"
path = []
start = (-30.872926879322137,-29.737362258608158)
goal = (49.26880075440613,-19.582571455483134)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (49.26880075440613,-19.582571455483134)
goal = (143.19646708578813,-23.227779535828347)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 145"
path = []
start = (-55.2877411057176,33.240978016820236)
goal = (44.50204739899735,44.288507090603304)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (44.50204739899735,44.288507090603304)
goal = (116.56201243060315,102.71856765789045)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 145"
path = []
start = (-158.19469010839526,-143.8303281690189)
goal = (31.35827781547721,-145.87002385123645)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (31.35827781547721,-145.87002385123645)
goal = (145.91057453485809,-160.4289340069148)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 145"
path = []
start = (-67.2753965853544,55.1503517218236)
goal = (-41.42779934384987,145.45902660641423)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-41.42779934384987,145.45902660641423)
goal = (105.18355605758506,130.23734699465132)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 145"
path = []
start = (-138.33844082607865,90.99044375505281)
goal = (-80.53093220696988,165.6740888925869)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-80.53093220696988,165.6740888925869)
goal = (80.32796271197265,183.70668882510614)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 145"
path = []
start = (-194.17819640045914,-138.3078297838234)
goal = (33.86072805167743,-141.8786324294417)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (33.86072805167743,-141.8786324294417)
goal = (145.89994361512117,-67.39656456980643)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 145"
path = []
start = (-157.0553218866224,-150.53732164960678)
goal = (32.9665669486412,-110.27696036214985)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.9665669486412,-110.27696036214985)
goal = (151.07462595974636,-183.31740162311445)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 145"
path = []
start = (-44.11786429423154,13.46273464845953)
goal = (58.00669323122463,-4.944935678585779)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (58.00669323122463,-4.944935678585779)
goal = (146.488441295673,-3.740023443722464)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 145"
path = []
start = (-66.32355160562403,54.74972225987412)
goal = (-9.70937137829057,132.10465234887545)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.70937137829057,132.10465234887545)
goal = (99.3886742438849,146.03888786627465)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 145"
path = []
start = (-193.30053420384036,88.43350148591554)
goal = (-92.10876381236895,196.6039409065619)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-92.10876381236895,196.6039409065619)
goal = (88.17980781958806,183.92226443127043)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 145"
path = []
start = (-176.58736382240764,98.02921925131074)
goal = (-57.21381436014869,163.52271851644963)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-57.21381436014869,163.52271851644963)
goal = (109.192331342297,136.28629837808347)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 145"
path = []
start = (-80.35889702507268,73.28119241003219)
goal = (9.902609772562442,113.85723537127296)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.902609772562442,113.85723537127296)
goal = (112.95267391168812,155.69708101289234)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 145"
path = []
start = (-114.44894963619716,-141.13137213652195)
goal = (37.54008893621901,-110.24082440190735)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (37.54008893621901,-110.24082440190735)
goal = (153.5322313618513,-96.28098279621611)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 145"
path = []
start = (-127.07045022916796,-153.55543767240403)
goal = (32.91946182956656,-197.8774642519668)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.91946182956656,-197.8774642519668)
goal = (157.03897193191295,-178.33412739571637)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 145"
path = []
start = (-41.037154448131474,-65.50405289915739)
goal = (49.33301768287478,-79.31912149584508)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (49.33301768287478,-79.31912149584508)
goal = (148.72764541727446,-2.18877765380779)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 145"
path = []
start = (-66.00904398332253,-111.0296213132937)
goal = (47.251059801602764,-132.53991957452675)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (47.251059801602764,-132.53991957452675)
goal = (167.46290919228602,-100.51551379166561)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 145"
path = []
start = (-29.363056786837035,-44.09615436434632)
goal = (59.753915791010684,-77.1959304911562)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (59.753915791010684,-77.1959304911562)
goal = (159.15220293409328,-40.62979290401407)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 145"
path = []
start = (-26.892005366582595,0.6723443464645129)
goal = (67.79702662102841,-28.14396460315365)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (67.79702662102841,-28.14396460315365)
goal = (146.78188058360723,27.929944789924264)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 145"
path = []
start = (-140.82840659362034,110.6734843120995)
goal = (-62.143482144483244,189.97543920874529)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-62.143482144483244,189.97543920874529)
goal = (117.29600477083437,198.15885022017852)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 145"
path = []
start = (-102.22517195220959,92.37169156450204)
goal = (-37.8152188755528,164.3468103482541)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-37.8152188755528,164.3468103482541)
goal = (134.44637654912185,89.00085521693478)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 145"
path = []
start = (-50.19032158013789,53.25764306433766)
goal = (12.900876659052557,111.16242402219297)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (12.900876659052557,111.16242402219297)
goal = (141.4688855539071,74.2446050749212)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 145"
path = []
start = (-82.79059190663082,81.47205763909227)
goal = (16.18249189002384,113.92232548188178)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (16.18249189002384,113.92232548188178)
goal = (133.2930233612783,144.05776273905718)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 145"
path = []
start = (-95.15240058870171,-148.98436915914522)
goal = (46.149668506418124,-155.83442923111588)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (46.149668506418124,-155.83442923111588)
goal = (167.66282384297756,-118.52476445356874)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 145"
path = []
start = (-36.805498589782104,34.14652395029992)
goal = (49.93513836057383,69.47033257530506)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (49.93513836057383,69.47033257530506)
goal = (150.03747516537788,61.17238219281853)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 145"
path = []
start = (-93.22349915053314,94.58884302245508)
goal = (-58.81430740824504,197.61160200761583)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-58.81430740824504,197.61160200761583)
goal = (127.10656559358699,166.24714043104586)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 145"
path = []
start = (-179.77852890492105,-164.04361002472254)
goal = (44.82182524595811,-183.12291803688953)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (44.82182524595811,-183.12291803688953)
goal = (168.6710768636641,-120.61949155355376)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 145"
path = []
start = (-114.38588921975095,107.7284930034445)
goal = (-40.23840944989132,197.88103332650098)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-40.23840944989132,197.88103332650098)
goal = (141.03817548805347,106.49915217729699)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 145"
path = []
start = (-137.612841967048,128.55423344261504)
goal = (-24.295990006946738,190.5595466692771)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.295990006946738,190.5595466692771)
goal = (152.07521535257678,90.5665442986803)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 145"
path = []
start = (-194.62516531181348,-172.5446891446322)
goal = (51.401040525488185,-129.28704988047872)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (51.401040525488185,-129.28704988047872)
goal = (177.86460447445205,-164.98750909027726)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 145"
path = []
start = (-114.08653192316764,-158.55028600943598)
goal = (54.24330650260799,-107.18467249259743)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (54.24330650260799,-107.18467249259743)
goal = (177.7531129654368,-118.50808549182044)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 145"
path = []
start = (-24.509698637757765,-89.78903994230545)
goal = (60.71899879043667,-107.7990756535506)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (60.71899879043667,-107.7990756535506)
goal = (166.14471349380426,-58.9880508327642)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 145"
path = []
start = (-20.053466830971615,-38.07569956843318)
goal = (68.9873161757825,-56.89457553191289)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (68.9873161757825,-56.89457553191289)
goal = (164.90483338376458,-49.171339211813205)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 145"
path = []
start = (-20.693461661185182,23.813965837188306)
goal = (50.543310132254874,82.50948445821643)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (50.543310132254874,82.50948445821643)
goal = (156.40040415890041,46.16524791234241)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 145"
path = []
start = (-157.57869723154423,-168.2909224893661)
goal = (63.35235413550936,-173.44267766788474)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (63.35235413550936,-173.44267766788474)
goal = (180.95160169962998,-122.92160699709123)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 145"
path = []
start = (-79.17621508873233,90.87474250784544)
goal = (6.552124820104723,150.99689493322887)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.552124820104723,150.99689493322887)
goal = (156.98654058276975,124.91528293499255)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 145"
path = []
start = (-105.96542327923694,114.41834304223914)
goal = (-22.52483066207867,195.00501955518814)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.52483066207867,195.00501955518814)
goal = (151.38743264885755,182.04082023624244)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 145"
path = []
start = (-171.8690972743541,-177.08295737885157)
goal = (64.252542135836,-147.95209527258604)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (64.252542135836,-147.95209527258604)
goal = (181.10189196580365,-116.75093506721205)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 145"
path = []
start = (-148.9493588200948,-177.6942592727827)
goal = (69.5831995441713,-166.40624471809642)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (69.5831995441713,-166.40624471809642)
goal = (180.35644440418162,-106.68230613390439)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 145"
path = []
start = (-24.622911253985137,-51.93318447206369)
goal = (76.674218101044,-73.0032935797004)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (76.674218101044,-73.0032935797004)
goal = (175.80397605097716,-16.69150313581497)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 145"
path = []
start = (-92.10115711832891,107.84132818141836)
goal = (-2.836943605727072,170.84114148737626)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.836943605727072,170.84114148737626)
goal = (162.52177885181163,133.83869491716155)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 145"
path = []
start = (-186.4112263574258,138.43417451940377)
goal = (1.726225827542379,168.5419137529051)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.726225827542379,168.5419137529051)
goal = (158.829168904725,161.0911503821278)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 145"
path = []
start = (-148.92987284574338,142.42415050555275)
goal = (-0.03680388922845168,186.8531541533996)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.03680388922845168,186.8531541533996)
goal = (155.263526051412,199.7824765130154)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 145"
path = []
start = (-66.76820604585129,85.7575673842984)
goal = (5.057349005941802,164.71807613493763)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.057349005941802,164.71807613493763)
goal = (164.2219895552666,107.08830307739203)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 145"
path = []
start = (-106.14295321991602,-168.14893194291315)
goal = (70.18565487853641,-185.74697397927108)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (70.18565487853641,-185.74697397927108)
goal = (191.28613529402116,-104.21502510876066)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 124 of 145"
path = []
start = (-127.16428431320863,-170.7353991926725)
goal = (83.45849006258942,-145.3883972613642)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (83.45849006258942,-145.3883972613642)
goal = (198.40897639817615,-146.6861075325814)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 125 of 145"
path = []
start = (-21.213053052203094,-97.3596225427273)
goal = (75.09045549424525,-81.18995269235936)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (75.09045549424525,-81.18995269235936)
goal = (186.96001407307955,-73.35443088939128)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 126 of 145"
path = []
start = (-4.303949479074987,-55.463124756237164)
goal = (78.90823426288375,-33.78468996637852)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (78.90823426288375,-33.78468996637852)
goal = (182.08924896850516,-35.6756095016714)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 127 of 145"
path = []
start = (-74.90911382596374,-167.40648350882074)
goal = (84.85567779992209,-125.65463839708838)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (84.85567779992209,-125.65463839708838)
goal = (192.72034428881233,-79.52749334050662)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 128 of 145"
path = []
start = (1.6436153777304412,-32.894169177396066)
goal = (75.05717223556763,-22.178525468540187)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (75.05717223556763,-22.178525468540187)
goal = (174.45009847661947,5.62632948423277)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 129 of 145"
path = []
start = (6.364021825461464,-56.65615477510485)
goal = (170.65726086199288,23.01733101019198)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 130 of 145"
path = []
start = (-67.99142521044618,90.2703358461676)
goal = (168.76389647403653,48.93960483242671)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 131 of 145"
path = []
start = (-11.969162831903418,-105.66526765735125)
goal = (194.7201177390807,-17.613663189491945)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 132 of 145"
path = []
start = (-45.239306861897774,78.99855043945257)
goal = (162.48953861190677,158.65417894376907)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 133 of 145"
path = []
start = (-103.07044262286959,118.5956943271961)
goal = (167.13411306698242,172.89006186686555)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 134 of 145"
path = []
start = (-144.0858295564318,-176.0498360149448)
goal = (188.10563248660605,1.7608711463033444)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 135 of 145"
path = []
start = (-150.9050696109323,-180.2313067797798)
goal = (190.8733176652346,14.209642903384264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 136 of 145"
path = []
start = (9.59244893939416,-44.622263414625706)
goal = (199.39791766751102,16.277857857910362)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 137 of 145"
path = []
start = (-59.945285853797515,110.54669396368843)
goal = (172.34347432906947,156.7806360472183)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 138 of 145"
path = []
start = (-111.43938107550983,135.91830445139846)
goal = (169.3193464682593,188.1815992106463)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 139 of 145"
path = []
start = (-55.40749719781624,108.11226349024219)
goal = (170.02434437910927,193.8460530335791)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 140 of 145"
path = []
start = (-50.50396181680435,110.13127202454308)
goal = (186.27092224538262,131.40803018442546)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 141 of 145"
path = []
start = (-128.6641356204177,-186.009415554429)
goal = (197.90793929080297,25.098112678985274)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 142 of 145"
path = []
start = (-109.73796355228723,-187.90527039478326)
goal = (194.82563169596256,131.7940828464782)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 143 of 145"
path = []
start = (-51.9801835635358,-161.6624617976558)
goal = (197.29507266790637,145.24711968175154)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 144 of 145"
path = []
start = (4.946251631287794,-80.3884922367759)
goal = (196.96470905199817,162.67711764214192)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 145 of 145"
path = []
start = (16.776457382257178,39.46270886517286)
goal = (190.44517200279955,185.16756227225568)
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
f = open('smo2sol-10.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
