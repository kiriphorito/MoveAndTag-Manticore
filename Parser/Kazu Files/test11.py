#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""
@brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT) 

@author: AtsushiSakai

@license: MIT

"""

import shapely
from shapely.geometry import Polygon, LineString, Point
import matplotlib.pyplot as plt
from ast import literal_eval

import random
import math
import copy

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
                print("Goal!!")
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
        p2 = LineString([(nearestNode.x,nearestNode.y),(node.x,node.y)])
        for poly in obstacleList:
            p1 = Polygon(poly)
            if p1.intersects(p2):
#                print "collision"
                return False
#        print "safe"
        return True

def LineCollisionCheck(first,second, obstacleList):
    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a=y2-y1
        b=-(x2-x1)
        c=y2*(x2-x1)-x2*(y2-y1)
    except ZeroDivisionError:
        return False

    p2 = LineString([(x1,y1),(x2,y2)])
    for poly in obstacleList:
        p1 = Polygon(poly)
        if p1.intersects(p2):
#            print "collision"
            return False
#    print "safe"
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
#    rrt.DrawGaph()
    plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')
#    print path
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-y')
    print smoothiePath
    return smoothiePath

#obstacleList = [[(1,2),(1,4),(3,4),(3,2)],[(8,1),(4,1),(4,4),(5,2)]]
#start = (0,0)
#goal = (5,10)
#rand = (-2,15)


obstacleList = [[(-0.0246174585342621,0.16891705339831045),(1.9301998869031127,-0.25379933613156713),(2.1415580816680513,0.7236093365871203),(1.1641494089493643,0.9349675313520591),(1.5868657984792418,2.8897848767894345),(0.6094571257605544,3.1011430715543726),(0.39809893099561555,2.123734398835685),(-0.5793097417230719,2.335092593600624),(-0.15659335219319442,4.289909939037998),(-1.1340020249118816,4.501268133802937),(-1.7680766092066984,1.5690421156468752),(0.18674073623067672,1.146325726116998)],[(-0.05901157522490006,-0.09731020795587667),(-1.492534431882603,-1.4919471569285974),(-0.7952159573962381,-2.2087085852574506),(-0.0784545290673882,-1.5113901107710888),(1.3161824199053331,-2.9449129674287895),(2.0329438482341837,-2.247594492942429),(1.335625373747822,-1.5308330646135777),(2.052386802076673,-0.8335145901272166),(0.638306899261461,-0.8140716362847271)],[(-1.8985138332922995,0.07593614482383877),(-3.888733837631598,0.27348192262654597),(-3.987506726532953,-0.7216280795431058),(-2.992396724363303,-0.8204009684444589),(-3.189942502166012,-2.8106209727837586),(-2.194832499996357,-2.9093938616851127),(-2.0960596110950043,-1.9142838595154614),(-1.100949608925354,-2.013056748416814),(-1.9972867221936528,-0.9191738573458109)],[(3.558366313174796,-0.7469747877882957),(4.200548350375849,-2.641071467236391),(5.147596690099898,-2.3199804486358633),(4.826505671499372,-1.3729321089118178),(6.720602350947466,-0.7307500717107629),(6.39951133234694,0.21629826801328667),(5.452462992622889,-0.10479275058724113),(5.1313719740223664,0.8422555891368066),(4.5054146528988435,-0.4258837691877702)],[(-0.1292924782198224,-2.838239291135715),(-1.8695413496807514,-3.8239034658823434),(-1.3767092623074424,-4.6940279016128095),(-0.5065848265769743,-4.201195814239495),(0.47907934816965103,-5.941444685700426),(1.3492037839001179,-5.448612598327115),(0.8563716965268057,-4.5784881625966465),(1.72649613225727,-4.0856560752233335),(0.36353960915349137,-3.70836372686618)],[(0.25492505294213214,6.481748034941264),(0.21323463991774919,8.481313465093988),(-1.786330790234977,8.439623052069605)],[(1.802666880196373,2.3290618407232597),(2.727656395595662,0.5558178088987102),(3.6142784115079385,1.0183125665983537),(3.151783653808293,1.904934582510628),(4.925027685632841,2.8299240979099256),(4.462532927933195,3.7165461138222016),(3.5759109120209187,3.2540513561225515),(3.1134161543212775,4.140673372034827),(2.6892888961086463,2.791556598422905)],[(3.9747552446326915,6.354629051101332),(2.451543302495492,5.0585482745559),(3.0995836907682035,4.296942303487298),(3.861189661836809,4.944982691760014),(5.15727043838224,3.42177074962281),(5.918876409450841,4.0698111378955275),(5.2708360211781216,4.83141710896413),(6.032441992246724,5.47945749723684),(7.328522768792157,3.956245555099641),(8.090128739860756,4.604285943372348),(7.442088351588037,5.365891914440944),(8.203694322656643,6.01393230271367),(6.794047963315328,6.12749788550956),(6.1460075750426135,6.889103856578155),(4.622795632905406,5.593023080032731)],[(-3.775070566373749,2.0380637954647742),(-3.5175923907117252,4.021420795366142),(-4.509270890662411,4.150159883197155),(-4.638009978493424,3.158481383246468),(-6.621366978394792,3.4159595589084892),(-6.7501060662258014,2.4242810589578054),(-5.758427566275118,2.295541971126795),(-5.88716665410613,1.3038634711761112),(-4.766749066324433,2.1668028832957846)],[(-2.8928868598181494,1.3522478460536007),(-4.84903978333255,1.7687400306454113),(-5.265531967924365,-0.18741289286898621)],[(-3.8010223726087475,-4.555270651862383),(-4.295555878780673,-2.6173757601633265),(-6.233450770479729,-3.1119092663352483)]]

start = (-6.453060574822895,-5.554029587082576)
goal = (0.5267146642115579,-0.4931108621138689)
rand = (-10,10)

rrtpath(obstacleList,start,goal,rand) # rrt  returns smoothie path
#rrtpath(obstacleList,gaol,start,rand)
drawPolygons(obstacleList) # draw map
#drawPolygonsNoFill(obstacleList)
plt.axis('scaled')
plt.grid(True)
plt.pause(0.01)  # Need for Mac
plt.show()
