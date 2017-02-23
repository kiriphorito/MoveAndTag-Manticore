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

obstacleList = [[(-0.11812502132352139,-0.07381399903411563),(1.7417586206376456,0.6615993802059312),(1.3740519310176222,1.5915412011865149),(0.444110110037039,1.2238345115664915),(-0.2913032692030079,3.0837181535276583),(-1.2212450901835914,2.7160114639076354),(-0.8535384005635681,1.7860696429270517),(-1.7834802215441519,1.4183629533070286),(-0.4858317109435447,0.856127821946468)],[(0.663644701244425,0.05921578053057661),(1.2239390763816864,-1.8606983355666888),(2.183896134430318,-1.5805511479980596),(1.9037489468616884,-0.620594089949427),(3.8236630629589548,-0.06029971481216682),(3.543515875390324,0.8996573432364662),(2.583558817341691,0.6195101556678362),(2.3034116297730605,1.5794672137164714),(1.6236017592930578,0.3393629680992064)],[(0.6128525607751204,-0.014713514261331499),(-1.3141735342211764,-0.5500362479494515),(-0.7788508005330569,-2.4770623429457492)],[(-1.2784470603290559,1.8403317379259416),(-2.029518735613629,3.69394748521156),(-2.956326609256439,3.3184116475692744),(-2.5807907716141525,2.3916037739264655),(-4.434406518899772,1.640532098641893),(-4.058870681257483,0.7137242249990787),(-3.1320628076146733,1.0892600626413667),(-2.756526969972386,0.1624521889985575),(-2.2052549339718652,1.4647959002836548)],[(0.7122525770223924,-4.041627045838036),(-0.5339623399873088,-5.605899522301934),(0.24817389824463615,-6.229006980806787),(0.8712813567494899,-5.446870742574838),(2.43555383321339,-6.693085659584537),(3.0586612917182423,-5.910949421352589),(2.2765250534862904,-5.287841962847736),(2.8996325119911437,-4.50570572461579),(4.463904988455042,-5.751920641625493),(5.087012446959891,-4.969784403393543),(4.3048762087279435,-4.346676944888691),(4.927983667232791,-3.5645407066567243),(3.5227399704959943,-3.723569486383838),(2.740603732264043,-3.100462027878984),(1.4943888152543414,-4.664734504342886)],[(0.2579043658520406,2.528991009120713),(1.9516392099420372,1.4653828679731247),(2.483443280515834,2.3122502900181208),(1.636575858470834,2.844054360591918),(2.7001839996184254,4.537789204681914),(1.8533165775734268,5.069593275255707),(1.321512506999631,4.22272585321071),(0.4746450849546353,4.7545299237845065),(0.789708436425836,3.375858431165711)],[(2.0733699865927,2.7956955987418235),(3.5552546457287484,1.4525582799924601),(4.898391964478116,2.934442939128516)],[(-1.559781122095868,-2.848935193283779),(-2.0229864932379007,-0.9033142681945252),(-2.9957969557825317,-1.1349169537655408),(-2.7641942702115143,-2.1077274163101682),(-4.709815195300768,-2.5709327874522),(-4.478212509729751,-3.543743249996829),(-3.5054020471851217,-3.312140564425814),(-3.273799361614106,-4.284951026970443),(-2.532591584640495,-3.0805378788547957)],[(-4.17939283522604,-1.383253932757841),(-3.609483456283639,0.5338280424830406),(-4.5680244439040765,0.8187827319542449),(-4.85297913337528,-0.13975825566619782),(-6.770061108616156,0.4301511232762114),(-7.055015798087366,-0.5283898643442407),(-6.096474810466922,-0.8133445538154415),(-6.381429499938125,-1.771885541435879),(-5.137933822846482,-1.098299243286641)],[(4.8763162212289,5.884527809447072),(3.311783825283987,4.638639218103982),(3.9347281209555334,3.8563730201315236),(4.7169943189279895,4.47931731580307),(5.9628829102710785,2.9147849198581572),(6.745149108243538,3.5377292155297035),(6.12220481257199,4.31999541350216),(6.9044710105444445,4.942939709173697),(8.150359601887534,3.3784073132287893),(8.932625799859991,4.0013516089003245),(8.309681504188445,4.783617806872785),(9.091947702160901,5.4065621025443225),(10.337836293503994,3.8420297065994133),(11.12010249147645,4.464974002270968),(10.497158195804907,5.2472402002434),(11.279424393777363,5.870184495914967),(9.87421390013336,6.029506398215873),(9.25126960446182,6.81177259618833),(7.686737208516903,5.565884004845237),(7.06379291284536,6.3481502028177035),(5.499260516900446,5.102261611474617)],[(1.714932538836905,5.3507657982269485),(2.658646924535163,7.11411578000202),(1.7769719336476213,7.585972972851154),(1.3051147407984969,6.704297981963617),(-0.45823524097657886,7.648012367661874),(-0.930092433825707,6.766337376774333),(-0.048417442938168076,6.294480183925208),(-0.5202746357873007,5.412805193037669),(0.8332575479493682,5.822622991076077)]]
rand = (-8,13)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 194"
path = []
start = (6.061567412712732,-1.407872896475956)
goal = (5.889953617652834,-1.4823332118357495)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.889953617652834,-1.4823332118357495)
goal = (6.844387808243249,-1.8253713269119256)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 194"
path = []
start = (5.889953617652834,-1.4823332118357495)
goal = (5.125325645181874,-1.3396519985823065)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.125325645181874,-1.3396519985823065)
goal = (5.682291412205224,-2.552166008437931)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 194"
path = []
start = (5.125325645181874,-1.3396519985823065)
goal = (4.740649020181603,-0.08769594037837347)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 194"
path = []
start = (6.844387808243249,-1.8253713269119256)
goal = (7.324598800000576,-2.2022270157082495)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 194"
path = []
start = (7.324598800000576,-2.2022270157082495)
goal = (7.7170089628804295,-2.334721359510138)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.7170089628804295,-2.334721359510138)
goal = (7.720261541344648,-1.5691525363096064)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 194"
path = []
start = (7.720261541344648,-1.5691525363096064)
goal = (7.810229775500787,-1.4628931229489828)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.810229775500787,-1.4628931229489828)
goal = (7.6541044803965566,-1.4407700678957172)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 194"
path = []
start = (7.810229775500787,-1.4628931229489828)
goal = (8.723117097024488,-1.4758501775005142)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 194"
path = []
start = (7.6541044803965566,-1.4407700678957172)
goal = (7.45676334075945,-1.3109890921375253)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 194"
path = []
start = (7.45676334075945,-1.3109890921375253)
goal = (7.5142096099356745,-1.0907658700672567)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 194"
path = []
start = (7.5142096099356745,-1.0907658700672567)
goal = (7.789631382984048,-1.0455291664879827)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.789631382984048,-1.0455291664879827)
goal = (7.314651392166951,-0.42936911340936135)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 194"
path = []
start = (7.789631382984048,-1.0455291664879827)
goal = (8.066826245961662,-0.41867583515492957)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 194"
path = []
start = (7.314651392166951,-0.42936911340936135)
goal = (7.4295799228737565,0.09134111689606073)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 194"
path = []
start = (7.4295799228737565,0.09134111689606073)
goal = (6.904076098407952,0.7174483349873322)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 194"
path = []
start = (6.904076098407952,0.7174483349873322)
goal = (7.170168499148756,1.2303391449319)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 194"
path = []
start = (7.170168499148756,1.2303391449319)
goal = (7.257980834815141,1.350461006307909)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 194"
path = []
start = (7.257980834815141,1.350461006307909)
goal = (7.951305524223693,1.7859341367260955)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 194"
path = []
start = (7.951305524223693,1.7859341367260955)
goal = (8.050837440417405,1.7621452490460294)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 194"
path = []
start = (8.050837440417405,1.7621452490460294)
goal = (8.966376271032944,1.4042336796918802)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.966376271032944,1.4042336796918802)
goal = (8.586985044424186,2.754985581286724)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 194"
path = []
start = (8.723117097024488,-1.4758501775005142)
goal = (9.005250840768793,-2.335948924428612)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 194"
path = []
start = (9.005250840768793,-2.335948924428612)
goal = (9.66163786247989,-1.9361401478243963)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 194"
path = []
start = (9.66163786247989,-1.9361401478243963)
goal = (9.805173145099578,-1.8242566781176297)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 194"
path = []
start = (9.805173145099578,-1.8242566781176297)
goal = (10.686625307667507,-2.198608297771422)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 194"
path = []
start = (10.686625307667507,-2.198608297771422)
goal = (11.170155785396856,-1.1052104415689588)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 194"
path = []
start = (8.966376271032944,1.4042336796918802)
goal = (9.008974253870003,0.6726169274789147)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 194"
path = []
start = (9.008974253870003,0.6726169274789147)
goal = (8.736101228112146,0.6766547688888211)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.736101228112146,0.6766547688888211)
goal = (9.8167487540455,0.23609276854885053)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 194"
path = []
start = (9.8167487540455,0.23609276854885053)
goal = (10.152217821532764,0.6616982399110736)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 194"
path = []
start = (10.152217821532764,0.6616982399110736)
goal = (10.394641554414045,0.44800835385711757)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.394641554414045,0.44800835385711757)
goal = (10.646749297473265,1.5447333670319745)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 194"
path = []
start = (10.394641554414045,0.44800835385711757)
goal = (10.29121306577994,-0.1242323680223576)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 194"
path = []
start = (10.646749297473265,1.5447333670319745)
goal = (11.189592170323047,1.7246158748021623)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.189592170323047,1.7246158748021623)
goal = (10.254264731120065,2.0528794818643883)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 194"
path = []
start = (10.254264731120065,2.0528794818643883)
goal = (10.800662058015758,2.8270132132419645)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 194"
path = []
start = (5.682291412205224,-2.552166008437931)
goal = (5.7574607789909305,-2.7115005176322704)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.7574607789909305,-2.7115005176322704)
goal = (4.8232282399163235,-2.398607979782409)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 194"
path = []
start = (5.7574607789909305,-2.7115005176322704)
goal = (5.96544265059117,-2.580627614185425)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.96544265059117,-2.580627614185425)
goal = (6.161031801909344,-3.1750495537396497)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 194"
path = []
start = (6.161031801909344,-3.1750495537396497)
goal = (6.088862984580276,-3.764665016360451)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.088862984580276,-3.764665016360451)
goal = (6.764640063753593,-3.510438827294863)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 194"
path = []
start = (6.088862984580276,-3.764665016360451)
goal = (6.686011503167652,-4.732377381791098)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 194"
path = []
start = (4.8232282399163235,-2.398607979782409)
goal = (4.320008621823234,-3.3425642437686998)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 194"
path = []
start = (4.320008621823234,-3.3425642437686998)
goal = (3.4858724036783375,-3.0811803257995356)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.4858724036783375,-3.0811803257995356)
goal = (4.551778146487102,-4.257692502965319)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 194"
path = []
start = (3.4858724036783375,-3.0811803257995356)
goal = (3.364382213513913,-2.612590874099655)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 194"
path = []
start = (3.364382213513913,-2.612590874099655)
goal = (3.0532985719132686,-2.767902401144351)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.0532985719132686,-2.767902401144351)
goal = (3.7441888856795433,-2.3507535929906593)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 194"
path = []
start = (3.0532985719132686,-2.767902401144351)
goal = (1.8562753815843909,-3.271643525854552)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 194"
path = []
start = (3.7441888856795433,-2.3507535929906593)
goal = (3.1048768839982994,-1.3654642684744722)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 194"
path = []
start = (4.551778146487102,-4.257692502965319)
goal = (5.005566158474464,-4.0717618760067245)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.005566158474464,-4.0717618760067245)
goal = (4.597757222644898,-5.6074177986824685)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 194"
path = []
start = (8.586985044424186,2.754985581286724)
goal = (8.495310197177883,3.261263997139751)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 194"
path = []
start = (8.495310197177883,3.261263997139751)
goal = (8.535620227870243,3.5179892350285806)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.535620227870243,3.5179892350285806)
goal = (8.198986450576637,3.240632882883136)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 194"
path = []
start = (8.535620227870243,3.5179892350285806)
goal = (8.967147345757818,3.709562030579872)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 194"
path = []
start = (8.198986450576637,3.240632882883136)
goal = (7.017943495739932,4.022170007193405)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 194"
path = []
start = (8.967147345757818,3.709562030579872)
goal = (8.992810294484057,3.673310600308695)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.992810294484057,3.673310600308695)
goal = (9.196672148263803,4.599904719955412)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 194"
path = []
start = (9.196672148263803,4.599904719955412)
goal = (9.02153212195376,4.724318605669491)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 194"
path = []
start = (9.02153212195376,4.724318605669491)
goal = (8.968566692981929,4.745798359333276)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 194"
path = []
start = (6.686011503167652,-4.732377381791098)
goal = (6.977894252471822,-4.974892796422408)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.977894252471822,-4.974892796422408)
goal = (5.981194846061758,-4.930628751205559)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 194"
path = []
start = (6.977894252471822,-4.974892796422408)
goal = (7.893186028356201,-4.765429741745553)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.893186028356201,-4.765429741745553)
goal = (6.598327130355891,-6.116703370724474)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 194"
path = []
start = (7.893186028356201,-4.765429741745553)
goal = (8.980949562986021,-4.652914217139042)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 194"
path = []
start = (8.980949562986021,-4.652914217139042)
goal = (9.724614169911474,-4.0488235583603664)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.724614169911474,-4.0488235583603664)
goal = (8.499690451246131,-3.75702311981645)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 194"
path = []
start = (9.724614169911474,-4.0488235583603664)
goal = (9.988034007254504,-4.194810089930838)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.988034007254504,-4.194810089930838)
goal = (10.047793116183,-3.462286132937973)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 194"
path = []
start = (9.988034007254504,-4.194810089930838)
goal = (9.997207626391168,-4.607971952905668)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 194"
path = []
start = (9.997207626391168,-4.607971952905668)
goal = (10.378345585470116,-4.852608294038647)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 194"
path = []
start = (10.378345585470116,-4.852608294038647)
goal = (10.370784230806718,-5.0906773472265945)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.370784230806718,-5.0906773472265945)
goal = (10.949988654009077,-4.582550985645335)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 194"
path = []
start = (10.370784230806718,-5.0906773472265945)
goal = (10.13131630592792,-5.669853822705868)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 194"
path = []
start = (10.13131630592792,-5.669853822705868)
goal = (10.401196730479215,-6.117879834501659)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.401196730479215,-6.117879834501659)
goal = (9.565582506620045,-5.976176349447453)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 194"
path = []
start = (10.401196730479215,-6.117879834501659)
goal = (10.771671418761757,-6.269619371142501)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.771671418761757,-6.269619371142501)
goal = (10.131890889899454,-6.446969075580658)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 194"
path = []
start = (10.771671418761757,-6.269619371142501)
goal = (10.79223348493761,-5.889570103095183)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 194"
path = []
start = (10.131890889899454,-6.446969075580658)
goal = (10.09785097911381,-6.533211080586458)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 194"
path = []
start = (10.949988654009077,-4.582550985645335)
goal = (11.248881721776167,-4.666963623271747)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 194"
path = []
start = (10.047793116183,-3.462286132937973)
goal = (11.12831845248247,-3.3503100247294073)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 194"
path = []
start = (8.499690451246131,-3.75702311981645)
goal = (8.49900358954497,-3.6053773301873124)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 194"
path = []
start = (3.1048768839982994,-1.3654642684744722)
goal = (2.4028512666419655,-1.7037550509412034)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 194"
path = []
start = (6.598327130355891,-6.116703370724474)
goal = (6.802559270334299,-6.327662151118546)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 194"
path = []
start = (1.8562753815843909,-3.271643525854552)
goal = (1.1865292350181136,-2.9075928285999013)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 194"
path = []
start = (1.1865292350181136,-2.9075928285999013)
goal = (0.3753596586296313,-3.1146686528173246)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 194"
path = []
start = (0.3753596586296313,-3.1146686528173246)
goal = (0.40552232216695305,-1.8582755912738191)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.40552232216695305,-1.8582755912738191)
goal = (-0.758849537841626,-3.82888868504663)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 194"
path = []
start = (0.40552232216695305,-1.8582755912738191)
goal = (0.6063713245488254,-0.5632168905704589)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 194"
path = []
start = (4.740649020181603,-0.08769594037837347)
goal = (5.017910976786824,0.09775207496929372)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 194"
path = []
start = (5.017910976786824,0.09775207496929372)
goal = (5.229299746891789,0.3936398904839873)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 194"
path = []
start = (5.229299746891789,0.3936398904839873)
goal = (5.680830506798497,0.01854699085562661)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.680830506798497,0.01854699085562661)
goal = (4.671364425728434,0.8814180440158124)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 194"
path = []
start = (4.671364425728434,0.8814180440158124)
goal = (4.9478907788975555,1.2081493716053089)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.9478907788975555,1.2081493716053089)
goal = (2.8983961282358752,0.7410131240841489)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 194"
path = []
start = (4.9478907788975555,1.2081493716053089)
goal = (4.436531070591217,1.811613959873113)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 194"
path = []
start = (4.436531070591217,1.811613959873113)
goal = (5.575855873895353,2.5639794481482037)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 194"
path = []
start = (0.6063713245488254,-0.5632168905704589)
goal = (-0.07061204217086736,-0.1390058531557532)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 194"
path = []
start = (-0.07061204217086736,-0.1390058531557532)
goal = (-0.8170537189446101,0.7380179239892248)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 194"
path = []
start = (-0.8170537189446101,0.7380179239892248)
goal = (-0.9867232621967021,0.9595623309589874)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.9867232621967021,0.9595623309589874)
goal = (0.4674521579205919,1.7641445849692472)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 194"
path = []
start = (-0.9867232621967021,0.9595623309589874)
goal = (-1.890202816337279,0.926803620096039)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 194"
path = []
start = (-1.890202816337279,0.926803620096039)
goal = (-2.1761245673680447,0.5193934252798647)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 194"
path = []
start = (-2.1761245673680447,0.5193934252798647)
goal = (-2.9488710284673205,0.18858129295424853)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 194"
path = []
start = (-2.9488710284673205,0.18858129295424853)
goal = (-2.8970315879557846,-0.7659329111134783)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 194"
path = []
start = (-2.8970315879557846,-0.7659329111134783)
goal = (-3.6352488437027346,-0.5143520980494101)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.6352488437027346,-0.5143520980494101)
goal = (-1.9539999913261648,-0.9100419141603222)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 194"
path = []
start = (-3.6352488437027346,-0.5143520980494101)
goal = (-5.00914952545128,0.5766596867721203)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 194"
path = []
start = (-0.758849537841626,-3.82888868504663)
goal = (-1.513504147953114,-4.282199458332755)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 194"
path = []
start = (-1.513504147953114,-4.282199458332755)
goal = (-1.8363661675462488,-4.86904136896576)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.8363661675462488,-4.86904136896576)
goal = (-1.646338346460566,-3.3937068138304034)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 194"
path = []
start = (-1.8363661675462488,-4.86904136896576)
goal = (-2.0857623619107706,-5.03291913906304)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 194"
path = []
start = (-2.0857623619107706,-5.03291913906304)
goal = (-2.907356297882541,-4.75309534895159)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.907356297882541,-4.75309534895159)
goal = (-2.569416140111163,-5.763821495266264)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 194"
path = []
start = (-2.907356297882541,-4.75309534895159)
goal = (-2.9616087497523793,-4.5388523447509925)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.9616087497523793,-4.5388523447509925)
goal = (-3.789120170541621,-5.079041652746398)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 194"
path = []
start = (-2.569416140111163,-5.763821495266264)
goal = (-3.343849061041273,-6.0758662876882354)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.343849061041273,-6.0758662876882354)
goal = (-2.33247391437998,-6.617478053205466)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 194"
path = []
start = (-3.343849061041273,-6.0758662876882354)
goal = (-3.337783199955228,-6.2769178068876155)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 194"
path = []
start = (-3.337783199955228,-6.2769178068876155)
goal = (-3.7350491091557534,-6.5526586365150505)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 194"
path = []
start = (-2.33247391437998,-6.617478053205466)
goal = (-1.7724950778250248,-6.473361233519791)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 194"
path = []
start = (-1.7724950778250248,-6.473361233519791)
goal = (-1.1701920369197296,-6.6620414062088225)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 194"
path = []
start = (-1.1701920369197296,-6.6620414062088225)
goal = (-0.1886939619338852,-6.212646599198041)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 194"
path = []
start = (-1.646338346460566,-3.3937068138304034)
goal = (-1.4720605304068606,-3.266297467482248)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.4720605304068606,-3.266297467482248)
goal = (-2.2887630848087523,-3.821821089174517)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 194"
path = []
start = (-1.4720605304068606,-3.266297467482248)
goal = (-1.2867696441615761,-2.8139245709027745)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 194"
path = []
start = (-1.2867696441615761,-2.8139245709027745)
goal = (-1.2338660376912731,-2.1582036188615836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 194"
path = []
start = (-1.2338660376912731,-2.1582036188615836)
goal = (-1.180585326237309,-2.15962191496938)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 194"
path = []
start = (-3.789120170541621,-5.079041652746398)
goal = (-4.53969430615023,-4.9998505433339435)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 194"
path = []
start = (-4.53969430615023,-4.9998505433339435)
goal = (-4.732944136856425,-4.70649659192975)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.732944136856425,-4.70649659192975)
goal = (-4.917482645034079,-6.065186273238256)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 194"
path = []
start = (-4.732944136856425,-4.70649659192975)
goal = (-4.565761111463596,-3.8330140389735288)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 194"
path = []
start = (-4.565761111463596,-3.8330140389735288)
goal = (-4.621700866707747,-3.2998481073173576)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.621700866707747,-3.2998481073173576)
goal = (-4.083411949857366,-3.593071455793336)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.083411949857366,-3.593071455793336)
goal = (-5.3428903824820555,-3.6990498972885866)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 194"
path = []
start = (-4.621700866707747,-3.2998481073173576)
goal = (-4.345552905315145,-2.200971155437535)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 194"
path = []
start = (-4.083411949857366,-3.593071455793336)
goal = (-3.7881278315423432,-3.620181836529433)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 194"
path = []
start = (-5.3428903824820555,-3.6990498972885866)
goal = (-6.10071335591711,-4.636596380798305)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 194"
path = []
start = (-4.345552905315145,-2.200971155437535)
goal = (-5.459677740899355,-2.2309179985350323)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 194"
path = []
start = (-6.10071335591711,-4.636596380798305)
goal = (-6.474510015150584,-4.803353974748074)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 194"
path = []
start = (-6.474510015150584,-4.803353974748074)
goal = (-6.6291774364726646,-3.988140921917554)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.6291774364726646,-3.988140921917554)
goal = (-6.228724538357445,-6.045362179524049)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 194"
path = []
start = (-6.6291774364726646,-3.988140921917554)
goal = (-6.573633544606223,-2.7621270814045142)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 194"
path = []
start = (-6.573633544606223,-2.7621270814045142)
goal = (-7.042463121144098,-1.5516121317402778)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 194"
path = []
start = (-6.228724538357445,-6.045362179524049)
goal = (-6.717006156767582,-6.420915551594833)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 194"
path = []
start = (-7.042463121144098,-1.5516121317402778)
goal = (-6.934157250897604,-1.0403771428193114)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 194"
path = []
start = (-6.934157250897604,-1.0403771428193114)
goal = (-6.312612466887127,-1.19617570697935)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 194"
path = []
start = (4.597757222644898,-5.6074177986824685)
goal = (4.024895216280932,-6.099668060793427)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.024895216280932,-6.099668060793427)
goal = (5.292756586376537,-6.573131278070255)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 194"
path = []
start = (4.024895216280932,-6.099668060793427)
goal = (3.347559369609634,-6.056190644291568)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 194"
path = []
start = (3.347559369609634,-6.056190644291568)
goal = (2.788908907691585,-4.891264347188919)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 194"
path = []
start = (2.788908907691585,-4.891264347188919)
goal = (3.1397298583073994,-4.728580981608678)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 194"
path = []
start = (5.575855873895353,2.5639794481482037)
goal = (5.2659370614344585,3.7599221563932455)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 194"
path = []
start = (5.2659370614344585,3.7599221563932455)
goal = (4.663641298596701,3.971410334974798)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 194"
path = []
start = (4.663641298596701,3.971410334974798)
goal = (4.5336388445679106,3.9278055338857927)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 194"
path = []
start = (4.5336388445679106,3.9278055338857927)
goal = (3.9292872522373212,3.657720453804491)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 124 of 194"
path = []
start = (3.9292872522373212,3.657720453804491)
goal = (3.6724605940118895,3.61976793911425)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.6724605940118895,3.61976793911425)
goal = (3.994446561834044,3.3990429992132514)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 125 of 194"
path = []
start = (3.6724605940118895,3.61976793911425)
goal = (3.5072843082808607,3.578732402088428)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.5072843082808607,3.578732402088428)
goal = (3.668191942373527,4.100069796908287)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 126 of 194"
path = []
start = (3.994446561834044,3.3990429992132514)
goal = (4.227850530054603,3.31820378019746)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 127 of 194"
path = []
start = (3.668191942373527,4.100069796908287)
goal = (3.3335933050136752,4.378690266808094)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 128 of 194"
path = []
start = (3.3335933050136752,4.378690266808094)
goal = (3.581494200436456,5.300571059934582)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 129 of 194"
path = []
start = (3.581494200436456,5.300571059934582)
goal = (3.047778919912661,6.0510177554730555)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 130 of 194"
path = []
start = (3.047778919912661,6.0510177554730555)
goal = (2.5790129678435125,5.451657778292634)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.5790129678435125,5.451657778292634)
goal = (3.7670594453017507,6.411234155618314)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.7670594453017507,6.411234155618314)
goal = (2.6375854178049156,7.273799400576428)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 131 of 194"
path = []
start = (2.5790129678435125,5.451657778292634)
goal = (2.566230873447509,5.087326864836272)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 132 of 194"
path = []
start = (3.7670594453017507,6.411234155618314)
goal = (4.241013792706294,7.0340782479761765)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 133 of 194"
path = []
start = (4.241013792706294,7.0340782479761765)
goal = (4.755207188615245,6.792761125055946)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 134 of 194"
path = []
start = (4.755207188615245,6.792761125055946)
goal = (4.919059148217324,7.383693757177524)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.919059148217324,7.383693757177524)
goal = (4.78380890560196,5.853438792617429)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 135 of 194"
path = []
start = (4.919059148217324,7.383693757177524)
goal = (6.056754557264252,7.448645571857926)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 136 of 194"
path = []
start = (4.78380890560196,5.853438792617429)
goal = (5.210291721221541,5.87232751059546)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 137 of 194"
path = []
start = (5.210291721221541,5.87232751059546)
goal = (5.213267103764677,5.822442291480843)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 138 of 194"
path = []
start = (5.213267103764677,5.822442291480843)
goal = (6.306564046653541,5.755608117821889)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 139 of 194"
path = []
start = (6.056754557264252,7.448645571857926)
goal = (6.358413125651714,7.114035705658902)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 140 of 194"
path = []
start = (6.358413125651714,7.114035705658902)
goal = (7.1898487803305695,6.482897219010886)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 141 of 194"
path = []
start = (7.1898487803305695,6.482897219010886)
goal = (7.7730696277059845,6.076413596632903)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 142 of 194"
path = []
start = (7.7730696277059845,6.076413596632903)
goal = (8.484504847111742,6.705565720911452)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 143 of 194"
path = []
start = (8.484504847111742,6.705565720911452)
goal = (9.684991784844206,7.341487488486794)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 144 of 194"
path = []
start = (2.6375854178049156,7.273799400576428)
goal = (2.746205543354117,7.517316404298966)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.746205543354117,7.517316404298966)
goal = (2.394001801473527,7.403612026057863)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 145 of 194"
path = []
start = (2.394001801473527,7.403612026057863)
goal = (0.7969231060295483,7.321547365663945)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 146 of 194"
path = []
start = (9.684991784844206,7.341487488486794)
goal = (10.189907819841986,7.011853974828781)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 147 of 194"
path = []
start = (10.189907819841986,7.011853974828781)
goal = (10.337814088663388,7.590081733516203)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.337814088663388,7.590081733516203)
goal = (11.076314297818714,5.535877224723351)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 148 of 194"
path = []
start = (0.4674521579205919,1.7641445849692472)
goal = (0.482758467175346,2.199526196536868)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 149 of 194"
path = []
start = (0.482758467175346,2.199526196536868)
goal = (0.1974869683716145,2.3868385454520267)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 150 of 194"
path = []
start = (11.076314297818714,5.535877224723351)
goal = (10.995144772023883,4.9023504375310525)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 151 of 194"
path = []
start = (-5.00914952545128,0.5766596867721203)
goal = (-5.6508313054039645,1.2123317502340374)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 152 of 194"
path = []
start = (-5.6508313054039645,1.2123317502340374)
goal = (-5.930114162595871,1.2320521054101787)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.930114162595871,1.2320521054101787)
goal = (-4.577633691340171,1.9846207232714947)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 153 of 194"
path = []
start = (-5.930114162595871,1.2320521054101787)
goal = (-6.483041495856531,1.2617904260487807)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 154 of 194"
path = []
start = (-6.483041495856531,1.2617904260487807)
goal = (-6.5859284240604286,1.6636489641051826)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 155 of 194"
path = []
start = (-4.577633691340171,1.9846207232714947)
goal = (-4.641996053660153,2.4626900594797467)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 156 of 194"
path = []
start = (-4.641996053660153,2.4626900594797467)
goal = (-4.4176458902934055,2.498309685286217)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 157 of 194"
path = []
start = (-4.4176458902934055,2.498309685286217)
goal = (-3.768745426476536,2.3785734678491677)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 158 of 194"
path = []
start = (-3.768745426476536,2.3785734678491677)
goal = (-3.386521282759093,2.856581589619098)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 159 of 194"
path = []
start = (-3.386521282759093,2.856581589619098)
goal = (-3.7210738611253165,3.027528906228148)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.7210738611253165,3.027528906228148)
goal = (-2.3918177597459778,3.718010009372702)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 160 of 194"
path = []
start = (-3.7210738611253165,3.027528906228148)
goal = (-4.1992088163271495,3.85247175021414)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 161 of 194"
path = []
start = (-2.3918177597459778,3.718010009372702)
goal = (-2.4167085559200077,3.93547733978395)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.4167085559200077,3.93547733978395)
goal = (-1.8131832271655597,3.1603195661027064)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 162 of 194"
path = []
start = (-1.8131832271655597,3.1603195661027064)
goal = (-0.9325683407330221,4.0660912634264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 163 of 194"
path = []
start = (-0.9325683407330221,4.0660912634264)
goal = (-0.6406239184601672,4.696617742243146)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 164 of 194"
path = []
start = (-0.6406239184601672,4.696617742243146)
goal = (-0.8315767735696777,4.940454163015336)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8315767735696777,4.940454163015336)
goal = (-0.32359816275809905,4.555868942584845)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 165 of 194"
path = []
start = (-0.8315767735696777,4.940454163015336)
goal = (-1.4502873857156988,5.332923310673848)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 166 of 194"
path = []
start = (-0.32359816275809905,4.555868942584845)
goal = (-0.1897237454616274,4.367037284932915)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.1897237454616274,4.367037284932915)
goal = (-0.2157452062202525,4.784079311227253)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 167 of 194"
path = []
start = (-0.1897237454616274,4.367037284932915)
goal = (0.17131346587098317,4.380332148520767)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 168 of 194"
path = []
start = (-1.4502873857156988,5.332923310673848)
goal = (-2.1516506321542233,5.481820406438)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 169 of 194"
path = []
start = (-2.1516506321542233,5.481820406438)
goal = (-2.4844832058349358,5.796971959516088)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 170 of 194"
path = []
start = (-2.4844832058349358,5.796971959516088)
goal = (-2.237062075953939,6.290291159512695)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.237062075953939,6.290291159512695)
goal = (-2.981359832547195,6.1601725262549545)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 171 of 194"
path = []
start = (-2.237062075953939,6.290291159512695)
goal = (-1.7117731676541226,6.558704419855354)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 172 of 194"
path = []
start = (-1.7117731676541226,6.558704419855354)
goal = (-1.4443965170280748,6.337746392107156)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.4443965170280748,6.337746392107156)
goal = (-1.9125889106494434,7.135892201999591)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 173 of 194"
path = []
start = (-1.4443965170280748,6.337746392107156)
goal = (-1.0608192728097583,6.208118383630416)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 174 of 194"
path = []
start = (-1.0608192728097583,6.208118383630416)
goal = (-0.9545020536873619,6.236253777462678)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 175 of 194"
path = []
start = (-0.9545020536873619,6.236253777462678)
goal = (-0.8911653544383888,5.791958413483907)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 176 of 194"
path = []
start = (-0.8911653544383888,5.791958413483907)
goal = (-0.49020831967118106,5.802814448980812)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 177 of 194"
path = []
start = (-1.9125889106494434,7.135892201999591)
goal = (-2.2216903101823124,7.054273894491417)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 178 of 194"
path = []
start = (-2.2216903101823124,7.054273894491417)
goal = (-2.327789572889162,7.178851238447785)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 179 of 194"
path = []
start = (-2.327789572889162,7.178851238447785)
goal = (-2.467331872277657,7.629535864197211)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 180 of 194"
path = []
start = (-2.467331872277657,7.629535864197211)
goal = (-3.260610059933854,7.463743489766823)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 181 of 194"
path = []
start = (-2.981359832547195,6.1601725262549545)
goal = (-3.5116601596533568,5.732714364580747)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 182 of 194"
path = []
start = (-3.5116601596533568,5.732714364580747)
goal = (-4.003106164309087,5.7221153209925815)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 183 of 194"
path = []
start = (-4.003106164309087,5.7221153209925815)
goal = (-4.836092556196736,6.403758460782657)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 184 of 194"
path = []
start = (-4.836092556196736,6.403758460782657)
goal = (-4.868776755365005,7.154570960768081)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.868776755365005,7.154570960768081)
goal = (-5.5313079254562485,5.932381601897894)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 185 of 194"
path = []
start = (-5.5313079254562485,5.932381601897894)
goal = (-5.626450011806315,5.2345564801538496)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 186 of 194"
path = []
start = (-5.626450011806315,5.2345564801538496)
goal = (-5.6663666247060345,5.103257601526049)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 187 of 194"
path = []
start = (-5.6663666247060345,5.103257601526049)
goal = (-6.499380906161857,5.366563055792942)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 188 of 194"
path = []
start = (-6.499380906161857,5.366563055792942)
goal = (-6.864962374755747,4.749477248442777)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.864962374755747,4.749477248442777)
goal = (-6.847589689983113,6.190305934460636)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 189 of 194"
path = []
start = (-6.864962374755747,4.749477248442777)
goal = (-6.432262675010541,4.142178726710612)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 190 of 194"
path = []
start = (-6.432262675010541,4.142178726710612)
goal = (-6.4663325469451225,3.8054471646601646)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 191 of 194"
path = []
start = (-6.4663325469451225,3.8054471646601646)
goal = (-6.796004514310067,3.1744221098982512)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 192 of 194"
path = []
start = (-6.847589689983113,6.190305934460636)
goal = (-6.802953497222142,7.483095203582322)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 193 of 194"
path = []
start = (2.8983961282358752,0.7410131240841489)
goal = (2.7289611190444667,0.9008780384049686)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 194 of 194"
path = []
start = (2.7289611190444667,0.9008780384049686)
goal = (2.61500199018667,1.4696556476263458)
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
f = open('smo2sol-15.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
