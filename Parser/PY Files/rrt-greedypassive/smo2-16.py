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

obstacleList = [[(0.04621876508645919,0.10060994795514129),(-0.6446192651600535,1.9775070724574721),(-1.5830678274112187,1.6320880573342162),(-1.237648812287963,0.6936394950830502),(-3.114545936790294,0.0028014648365369405),(-2.7691269216670373,-0.9356470974146281),(-1.8306783594158718,-0.5902280822913717),(-1.4852593442926152,-1.5286766445425375),(-0.8922297971647063,-0.24480906716811524)],[(0.5153913628667811,-0.3690068564670438),(1.7582431042906053,-1.9359527868806516),(3.3251890347042132,-0.6931010454568327)],[(1.627866996998279,-0.4277808703441106),(3.6163437239982,-0.2133973211159187),(3.5091519493841066,0.7808410423840408),(2.514913585884145,0.6736492677699455),(2.3005300366559576,2.662125994769868),(1.3062916731559966,2.554934220155773),(1.4134834477700926,1.5606958566558107),(0.41924508427012985,1.453504082041717),(1.5206752223841848,0.5664574931558504)],[(-0.04086923215943707,-4.124302675704636),(1.080849990361062,-5.780124521746053),(2.7366718364024796,-4.658405299225552)],[(2.703113130158038,-2.182116662408627),(3.601180940907023,-3.9691460990146727),(5.388210377513069,-3.0710782882656904)],[(-0.39095719828752673,-0.857400308456637),(-1.9586744446963817,-2.099278980192421),(-1.3377351088284921,-2.883137603396849),(-0.5538764856240623,-2.262198267528957),(0.6880021861117224,-3.8299155139378125),(1.4718608093161474,-3.2089761780699217),(0.8509214734482574,-2.4251175548654924),(1.6347800966526873,-1.8041782189975988),(0.22998213758036412,-1.6412589316610646)],[(-5.513249026884157,1.3337860792151164),(-5.968098810819156,3.2813773284392136),(-6.941894435431208,3.0539524364717145),(-6.714469543463708,2.0801568118596636),(-8.662060792687805,1.6253070279246633),(-8.434635900720306,0.6515114033126155),(-7.460840276108255,0.8789362952801162),(-7.233415384140756,-0.0948593293319333),(-6.4870446514962055,1.1063611872476158)],[(0.7263068914334939,2.0660555157102944),(1.4621343485319183,3.925775374554649),(0.5322744191097389,4.29368910310386),(0.1643606905605287,3.363829173681684),(-1.6953591682838254,4.0996566307801094),(-2.0632728968330376,3.1697967013579307),(-1.1334129674108602,2.801882972808719),(-1.5013266959600742,1.8720230433865415),(-0.2035530379886833,2.4339692442595067)],[(-1.9958510002753678,-3.8289513268731428),(-3.7097224588260778,-4.859797883624077),(-3.1942991804506145,-5.716733612899432),(-2.3373634511752566,-5.201310334523965),(-1.306516894424325,-6.915181793074678),(-0.4495811651489645,-6.399758514699209),(-0.9650044435244329,-5.542822785423852),(-0.10806871424907549,-5.027399507048385),(-1.4804277218999002,-4.6858870561484975)],[(-3.2934808399796816,0.944592838294841),(-1.877218629409676,2.356754780464134),(-2.5832996004943247,3.0648858857491375),(-3.2914307057793257,2.358804914664492),(-4.703592647948625,3.7750671252344983),(-5.411723753233623,3.0689861541498527),(-4.7056427821489715,2.360855048864849),(-5.413773887433975,1.6547740777802074),(-3.9995618110643276,1.652723943579844)],[(0.4494406174209904,4.80579070796561),(2.1027575803316574,3.6803827729696597),(2.6654615478296333,4.507041254424991),(1.8388030663742991,5.069745221922973),(2.964211001370244,6.7230621848336405),(2.1375525199149035,7.285766152331618),(1.5748485524169311,6.45910767087628),(0.748190070961598,7.021811638374254),(1.012144584918961,5.632449189420945)],[(-3.875586117053559,-0.3721963798411403),(-5.396487954078655,0.9265944432453495),(-6.045883365621902,0.1661435247328007),(-5.285432447109352,-0.4832518868104466),(-6.584223270195842,-2.004153723835545),(-5.823772351683297,-2.6535491353787855),(-5.17437694014005,-1.8930982168662358),(-4.413926021627502,-2.542493628409482),(-4.524981528596802,-1.1326472983536886)],[(2.3220333131342192,2.9125941560715867),(4.183340219634795,3.644397818137617),(3.451536557568759,5.505704724638196)],[(-2.7843898540847896,6.590949390343138),(-4.089696862446023,5.0756363487827345),(-2.574383820885602,3.770329340421502)],[(-5.950302147743739,3.514348373371524),(-4.640301691213715,5.025605731961093),(-6.151559049803278,6.335606188491122)]]
rand = (-10,9)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 191"
path = []
start = (-7.616610569978709,-3.1920622435973343)
goal = (-7.494007623786018,-3.1341757281633846)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.494007623786018,-3.1341757281633846)
goal = (-7.532968234498578,-3.5066090371119527)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.532968234498578,-3.5066090371119527)
goal = (-8.60639880156461,-2.7896286436069246)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 191"
path = []
start = (-7.494007623786018,-3.1341757281633846)
goal = (-7.312655149560837,-3.095125740655507)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 191"
path = []
start = (-7.532968234498578,-3.5066090371119527)
goal = (-7.570422610073352,-3.8782192796052106)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 191"
path = []
start = (-7.570422610073352,-3.8782192796052106)
goal = (-7.083817574123716,-3.897781686065606)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.083817574123716,-3.897781686065606)
goal = (-8.12011909940036,-4.310524536033822)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 191"
path = []
start = (-7.083817574123716,-3.897781686065606)
goal = (-6.944451426562614,-4.328527719711497)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 191"
path = []
start = (-6.944451426562614,-4.328527719711497)
goal = (-6.153384094578561,-4.595297757093679)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 191"
path = []
start = (-8.12011909940036,-4.310524536033822)
goal = (-7.63332954408761,-4.851867245132165)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 191"
path = []
start = (-7.63332954408761,-4.851867245132165)
goal = (-7.922818149642629,-5.896199543364338)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 191"
path = []
start = (-6.153384094578561,-4.595297757093679)
goal = (-5.867431417963736,-4.266608049047148)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 191"
path = []
start = (-5.867431417963736,-4.266608049047148)
goal = (-5.721935393126712,-4.12695614034425)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 191"
path = []
start = (-5.721935393126712,-4.12695614034425)
goal = (-5.716711174913446,-3.924603240590139)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 191"
path = []
start = (-5.716711174913446,-3.924603240590139)
goal = (-6.22104532088233,-3.5744868710560347)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 191"
path = []
start = (-6.22104532088233,-3.5744868710560347)
goal = (-6.045001827828083,-2.9876567300362105)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 191"
path = []
start = (-6.045001827828083,-2.9876567300362105)
goal = (-5.938957630191607,-3.0082342433262745)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 191"
path = []
start = (-5.938957630191607,-3.0082342433262745)
goal = (-5.699501563631004,-2.9381299157002805)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 191"
path = []
start = (-5.699501563631004,-2.9381299157002805)
goal = (-5.339695751031098,-3.302302168607268)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 191"
path = []
start = (-5.339695751031098,-3.302302168607268)
goal = (-5.185196008657498,-3.5622739616908214)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.185196008657498,-3.5622739616908214)
goal = (-4.670503159345165,-2.634494921289729)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 191"
path = []
start = (-5.185196008657498,-3.5622739616908214)
goal = (-5.074182841110654,-3.97903198206073)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 191"
path = []
start = (-8.60639880156461,-2.7896286436069246)
goal = (-8.113304930575392,-2.1148085360923945)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 191"
path = []
start = (-8.113304930575392,-2.1148085360923945)
goal = (-8.467601238113746,-1.9603040162030343)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.467601238113746,-1.9603040162030343)
goal = (-8.012073287761462,-1.6969923255808457)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 191"
path = []
start = (-8.012073287761462,-1.6969923255808457)
goal = (-7.218273375613851,-1.9698765422683993)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 191"
path = []
start = (-7.218273375613851,-1.9698765422683993)
goal = (-6.651897432160924,-1.2987837480807416)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 191"
path = []
start = (-6.651897432160924,-1.2987837480807416)
goal = (-6.879780174974587,-0.7671384925151319)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 191"
path = []
start = (-6.879780174974587,-0.7671384925151319)
goal = (-7.028993017310095,-0.22385684431033948)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.028993017310095,-0.22385684431033948)
goal = (-6.234809676505937,-0.7321591877264906)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 191"
path = []
start = (-7.028993017310095,-0.22385684431033948)
goal = (-7.540614328092508,0.4181674574207115)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 191"
path = []
start = (-6.234809676505937,-0.7321591877264906)
goal = (-5.8579295430011875,-0.8046643950466725)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.8579295430011875,-0.8046643950466725)
goal = (-6.353789768182143,-0.2181111316724369)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 191"
path = []
start = (-5.8579295430011875,-0.8046643950466725)
goal = (-5.973893281467019,-1.0507100248143173)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.973893281467019,-1.0507100248143173)
goal = (-5.6371275544067405,-0.6047652784826392)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 191"
path = []
start = (-5.6371275544067405,-0.6047652784826392)
goal = (-5.598514552841731,-0.32082310276311077)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 191"
path = []
start = (-6.353789768182143,-0.2181111316724369)
goal = (-6.295508305891476,-0.12072040962878461)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 191"
path = []
start = (-6.295508305891476,-0.12072040962878461)
goal = (-6.30362416310043,0.07896805297669474)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 191"
path = []
start = (-6.30362416310043,0.07896805297669474)
goal = (-6.297929828392041,0.37008243173812616)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 191"
path = []
start = (-6.297929828392041,0.37008243173812616)
goal = (-6.186999442122932,0.5358966114925607)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 191"
path = []
start = (-6.186999442122932,0.5358966114925607)
goal = (-5.680628729780301,0.7909742318267714)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 191"
path = []
start = (-5.680628729780301,0.7909742318267714)
goal = (-5.6176661242012775,0.7690699265079175)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.6176661242012775,0.7690699265079175)
goal = (-5.619517534988901,1.2365657325420187)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 191"
path = []
start = (-5.6176661242012775,0.7690699265079175)
goal = (-5.109986778299507,0.6924973394225997)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 191"
path = []
start = (-7.540614328092508,0.4181674574207115)
goal = (-7.697750730476685,0.5488750847115593)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.697750730476685,0.5488750847115593)
goal = (-8.092702578322527,-0.28573949990881164)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 191"
path = []
start = (-8.092702578322527,-0.28573949990881164)
goal = (-8.565166823997792,-0.6587779437155064)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 191"
path = []
start = (-7.922818149642629,-5.896199543364338)
goal = (-7.560140397207439,-6.255221691881703)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 191"
path = []
start = (-7.560140397207439,-6.255221691881703)
goal = (-7.730768067568595,-6.451567465209081)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.730768067568595,-6.451567465209081)
goal = (-7.229007701060005,-6.501047791396234)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 191"
path = []
start = (-7.730768067568595,-6.451567465209081)
goal = (-8.523439591267076,-6.5645304127875885)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 191"
path = []
start = (-7.229007701060005,-6.501047791396234)
goal = (-7.113448848079783,-6.619357035745311)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.113448848079783,-6.619357035745311)
goal = (-7.035768898880473,-6.238841468150579)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 191"
path = []
start = (-7.113448848079783,-6.619357035745311)
goal = (-7.199369635148116,-6.863802986585745)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.199369635148116,-6.863802986585745)
goal = (-6.7663131646821855,-6.7354514324082935)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 191"
path = []
start = (-7.035768898880473,-6.238841468150579)
goal = (-6.887828862044886,-5.776851947983709)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 191"
path = []
start = (-6.7663131646821855,-6.7354514324082935)
goal = (-6.61668431008823,-6.910721782270482)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 191"
path = []
start = (-6.61668431008823,-6.910721782270482)
goal = (-5.850780604176286,-6.87041034488576)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 191"
path = []
start = (-6.887828862044886,-5.776851947983709)
goal = (-6.456650430846333,-5.986709674434204)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 191"
path = []
start = (-5.850780604176286,-6.87041034488576)
goal = (-5.687222381537895,-6.218696644601792)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 191"
path = []
start = (-5.687222381537895,-6.218696644601792)
goal = (-5.412986469997032,-5.792518334657927)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 191"
path = []
start = (-5.412986469997032,-5.792518334657927)
goal = (-4.883301129109456,-5.155092841695394)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 191"
path = []
start = (-4.883301129109456,-5.155092841695394)
goal = (-4.374213657802583,-5.288334998008685)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 191"
path = []
start = (-4.374213657802583,-5.288334998008685)
goal = (-3.987364521238862,-4.565371966643745)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.987364521238862,-4.565371966643745)
goal = (-3.5415008177729996,-5.332128547190103)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 191"
path = []
start = (-3.987364521238862,-4.565371966643745)
goal = (-3.8606782473251586,-3.5710573825171954)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 191"
path = []
start = (-3.5415008177729996,-5.332128547190103)
goal = (-3.05528037137453,-5.736520825345187)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 191"
path = []
start = (-3.05528037137453,-5.736520825345187)
goal = (-2.9172339826668896,-6.035904797555333)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 191"
path = []
start = (-2.9172339826668896,-6.035904797555333)
goal = (-2.654380173518881,-5.90759620512999)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.654380173518881,-5.90759620512999)
goal = (-3.0123494401847015,-6.40899626645933)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 191"
path = []
start = (-2.654380173518881,-5.90759620512999)
goal = (-2.254101572955565,-5.766179791923145)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 191"
path = []
start = (-3.0123494401847015,-6.40899626645933)
goal = (-2.9061865028284997,-6.6155165043289745)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.9061865028284997,-6.6155165043289745)
goal = (-3.342568995746009,-6.53637167602137)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 191"
path = []
start = (-2.9061865028284997,-6.6155165043289745)
goal = (-2.3762395307344493,-6.8022979713702405)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 191"
path = []
start = (-3.342568995746009,-6.53637167602137)
goal = (-3.32529898024469,-6.84559547202776)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.32529898024469,-6.84559547202776)
goal = (-3.8319576642485567,-6.308210574826004)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 191"
path = []
start = (-2.254101572955565,-5.766179791923145)
goal = (-1.7628848091737028,-6.24600272098128)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 191"
path = []
start = (-3.8319576642485567,-6.308210574826004)
goal = (-4.576415499763075,-6.546674364267572)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 191"
path = []
start = (-1.7628848091737028,-6.24600272098128)
goal = (-0.5533543234218516,-6.509900110591763)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 191"
path = []
start = (-3.8606782473251586,-3.5710573825171954)
goal = (-3.7726859980894734,-3.4032958961112243)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.7726859980894734,-3.4032958961112243)
goal = (-3.200427921919186,-3.825200172868198)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 191"
path = []
start = (-3.200427921919186,-3.825200172868198)
goal = (-2.8742370860155138,-4.083732295911098)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 191"
path = []
start = (-2.8742370860155138,-4.083732295911098)
goal = (-2.6003040421348143,-3.522601713369313)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 191"
path = []
start = (-2.6003040421348143,-3.522601713369313)
goal = (-1.9158217670823632,-3.174373633279552)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 191"
path = []
start = (-1.9158217670823632,-3.174373633279552)
goal = (-1.2821042426914735,-3.566293725759531)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.2821042426914735,-3.566293725759531)
goal = (-1.7047703172553081,-2.4423130729463898)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 191"
path = []
start = (-1.2821042426914735,-3.566293725759531)
goal = (-1.256080912022095,-4.026849057517625)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.256080912022095,-4.026849057517625)
goal = (-0.29843942963292314,-3.1389740896028053)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 191"
path = []
start = (-1.256080912022095,-4.026849057517625)
goal = (-1.3428112889555903,-4.236101922200566)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 191"
path = []
start = (-1.7047703172553081,-2.4423130729463898)
goal = (-2.511027457210006,-2.3314633492733394)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 191"
path = []
start = (-2.511027457210006,-2.3314633492733394)
goal = (-2.9138640477453883,-2.071116778873403)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 191"
path = []
start = (-2.9138640477453883,-2.071116778873403)
goal = (-3.133251412486464,-2.4033022634424137)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.133251412486464,-2.4033022634424137)
goal = (-2.3605201758443775,-1.06895470182367)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 191"
path = []
start = (-3.133251412486464,-2.4033022634424137)
goal = (-3.7946247045650168,-1.994605545076178)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 191"
path = []
start = (-0.29843942963292314,-3.1389740896028053)
goal = (0.09422288754572605,-4.424303375274283)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 191"
path = []
start = (-2.3605201758443775,-1.06895470182367)
goal = (-2.0401730790685635,-0.9942512466286475)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 191"
path = []
start = (-2.0401730790685635,-0.9942512466286475)
goal = (-1.9512453664808351,-1.3241769993835772)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 191"
path = []
start = (-1.9512453664808351,-1.3241769993835772)
goal = (-1.3242985272962446,-1.2278923168226124)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 191"
path = []
start = (-1.3242985272962446,-1.2278923168226124)
goal = (-0.2409778665484783,-0.48669979516830697)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 191"
path = []
start = (-0.2409778665484783,-0.48669979516830697)
goal = (0.930425586449914,0.14039982636335235)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 191"
path = []
start = (0.09422288754572605,-4.424303375274283)
goal = (1.1497558641604702,-4.051959180298361)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 191"
path = []
start = (1.1497558641604702,-4.051959180298361)
goal = (1.8218105917013183,-3.9135008435204695)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 191"
path = []
start = (1.8218105917013183,-3.9135008435204695)
goal = (1.9930754998256788,-3.6046129692316065)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.9930754998256788,-3.6046129692316065)
goal = (2.4136267744206403,-4.252487687400084)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 191"
path = []
start = (1.9930754998256788,-3.6046129692316065)
goal = (1.942655088360313,-2.9335169506486736)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 191"
path = []
start = (1.942655088360313,-2.9335169506486736)
goal = (1.9963442172510923,-2.3713051721363696)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.9963442172510923,-2.3713051721363696)
goal = (2.5006162863076327,-3.034297697190439)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 191"
path = []
start = (1.9963442172510923,-2.3713051721363696)
goal = (1.9016984717177756,-2.271921046758983)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.9016984717177756,-2.271921046758983)
goal = (2.4007341202714993,-2.3427314187991763)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 191"
path = []
start = (1.9016984717177756,-2.271921046758983)
goal = (1.8492553207111673,-1.9699820441139622)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.8492553207111673,-1.9699820441139622)
goal = (1.3405591984903769,-2.4598701889847314)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 191"
path = []
start = (2.4007341202714993,-2.3427314187991763)
goal = (2.6089767978950533,-2.04523638965063)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 191"
path = []
start = (2.6089767978950533,-2.04523638965063)
goal = (3.54164708816292,-1.8549238297343598)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 191"
path = []
start = (2.4136267744206403,-4.252487687400084)
goal = (2.431262486656827,-4.241477708502515)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 191"
path = []
start = (2.431262486656827,-4.241477708502515)
goal = (2.597769898738166,-4.435538968835094)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 191"
path = []
start = (2.597769898738166,-4.435538968835094)
goal = (2.797352898231564,-4.269208692523653)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.797352898231564,-4.269208692523653)
goal = (3.018465805419865,-5.1282007509247585)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 191"
path = []
start = (2.797352898231564,-4.269208692523653)
goal = (3.0742133509232463,-3.603256665269789)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 191"
path = []
start = (3.018465805419865,-5.1282007509247585)
goal = (2.5280267790598767,-6.14919441522007)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 191"
path = []
start = (3.54164708816292,-1.8549238297343598)
goal = (4.1410925159584675,-1.9259267109753253)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 191"
path = []
start = (4.1410925159584675,-1.9259267109753253)
goal = (4.047542900111862,-1.5105260868404606)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.047542900111862,-1.5105260868404606)
goal = (4.729132097369044,-2.0867386187845476)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.729132097369044,-2.0867386187845476)
goal = (4.2352780102448975,-2.5641149516319084)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 191"
path = []
start = (4.047542900111862,-1.5105260868404606)
goal = (4.357238266453722,-1.1486955963879986)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 191"
path = []
start = (4.357238266453722,-1.1486955963879986)
goal = (4.6412591713697395,-0.8448216567897626)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 191"
path = []
start = (4.6412591713697395,-0.8448216567897626)
goal = (5.056906334568579,-0.7800755829435069)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.056906334568579,-0.7800755829435069)
goal = (4.32658372979534,-0.5330691886158814)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 191"
path = []
start = (5.056906334568579,-0.7800755829435069)
goal = (5.058424383036632,-1.0713560422919155)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 191"
path = []
start = (5.058424383036632,-1.0713560422919155)
goal = (5.020473453941392,-1.4537288835547697)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 191"
path = []
start = (4.32658372979534,-0.5330691886158814)
goal = (4.336713844600164,-0.42828238155438747)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 191"
path = []
start = (4.336713844600164,-0.42828238155438747)
goal = (4.101519118149023,-0.32725841523481414)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 191"
path = []
start = (4.101519118149023,-0.32725841523481414)
goal = (3.570183936048714,0.5523108089470465)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 191"
path = []
start = (4.2352780102448975,-2.5641149516319084)
goal = (4.330580697780862,-3.7698412114315403)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 191"
path = []
start = (3.570183936048714,0.5523108089470465)
goal = (3.744434996318752,1.2073814415785478)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.744434996318752,1.2073814415785478)
goal = (2.86023968443342,-0.42369687076317764)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 191"
path = []
start = (3.744434996318752,1.2073814415785478)
goal = (3.8555475324115474,1.8251060125292824)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 191"
path = []
start = (3.8555475324115474,1.8251060125292824)
goal = (3.105012770316037,1.748209043964227)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.105012770316037,1.748209043964227)
goal = (4.586200889330975,1.456901297308356)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 191"
path = []
start = (3.105012770316037,1.748209043964227)
goal = (2.49560308160434,2.062639725904999)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 191"
path = []
start = (2.49560308160434,2.062639725904999)
goal = (2.9289366627407887,2.7325290494791634)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.9289366627407887,2.7325290494791634)
goal = (1.5537683568924905,2.7440153396354585)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 191"
path = []
start = (2.9289366627407887,2.7325290494791634)
goal = (3.4397725064942097,2.871430376006229)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 191"
path = []
start = (3.4397725064942097,2.871430376006229)
goal = (4.495795601809128,2.7638289547512436)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 191"
path = []
start = (4.495795601809128,2.7638289547512436)
goal = (4.657094677267365,2.9063659336142695)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 191"
path = []
start = (4.657094677267365,2.9063659336142695)
goal = (4.859650307236141,3.7258291529196814)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 191"
path = []
start = (4.859650307236141,3.7258291529196814)
goal = (5.346832922255672,4.131676782758417)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 191"
path = []
start = (5.346832922255672,4.131676782758417)
goal = (4.997665243531367,4.910494413201437)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 191"
path = []
start = (4.997665243531367,4.910494413201437)
goal = (4.9230810458685905,5.107456976248235)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 191"
path = []
start = (4.9230810458685905,5.107456976248235)
goal = (4.197342389474782,5.22710745475926)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 191"
path = []
start = (4.197342389474782,5.22710745475926)
goal = (3.8585196428947324,5.14234721585394)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.8585196428947324,5.14234721585394)
goal = (4.571011070268867,5.7596302667890065)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 191"
path = []
start = (3.8585196428947324,5.14234721585394)
goal = (3.8884410709513375,4.872645498114027)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 191"
path = []
start = (4.571011070268867,5.7596302667890065)
goal = (4.88840974611302,5.929714679256362)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.88840974611302,5.929714679256362)
goal = (4.1211207100787846,6.0134220300076)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 191"
path = []
start = (4.1211207100787846,6.0134220300076)
goal = (3.9537108044841727,6.009966102551116)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.9537108044841727,6.009966102551116)
goal = (4.107788821403613,6.740594162014954)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 191"
path = []
start = (3.9537108044841727,6.009966102551116)
goal = (3.2204017091364,5.678167284847113)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 191"
path = []
start = (4.107788821403613,6.740594162014954)
goal = (4.3691938095908345,6.972773248785636)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.3691938095908345,6.972773248785636)
goal = (3.616175652027694,7.116059706772198)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 124 of 191"
path = []
start = (3.2204017091364,5.678167284847113)
goal = (2.9175377932876785,5.658050419940414)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 125 of 191"
path = []
start = (2.9175377932876785,5.658050419940414)
goal = (2.5827767892686033,5.3053303639067035)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 126 of 191"
path = []
start = (2.5827767892686033,5.3053303639067035)
goal = (2.4320950954287195,5.151629079763676)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 127 of 191"
path = []
start = (2.5280267790598767,-6.14919441522007)
goal = (3.107884572491521,-6.710327203036505)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.107884572491521,-6.710327203036505)
goal = (1.6824746484189639,-5.839127700990904)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 128 of 191"
path = []
start = (1.6824746484189639,-5.839127700990904)
goal = (1.4687655211497521,-5.692576016498359)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.4687655211497521,-5.692576016498359)
goal = (1.5933898003941316,-6.4361252565389275)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 129 of 191"
path = []
start = (1.4687655211497521,-5.692576016498359)
goal = (0.5247209191593001,-5.586904947327773)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 130 of 191"
path = []
start = (1.5537683568924905,2.7440153396354585)
goal = (1.427962016549694,2.647084189430931)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 131 of 191"
path = []
start = (1.427962016549694,2.647084189430931)
goal = (1.1441530947708785,2.7586207492484203)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 132 of 191"
path = []
start = (1.1441530947708785,2.7586207492484203)
goal = (0.2411284666035165,1.84578877209972)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 133 of 191"
path = []
start = (4.330580697780862,-3.7698412114315403)
goal = (5.057209155766653,-3.5197289134610266)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.057209155766653,-3.5197289134610266)
goal = (4.446072971410242,-4.834454958874597)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 134 of 191"
path = []
start = (4.446072971410242,-4.834454958874597)
goal = (4.69473365701039,-4.852471307428095)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 135 of 191"
path = []
start = (4.69473365701039,-4.852471307428095)
goal = (5.0955703946591555,-5.376949124262429)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 136 of 191"
path = []
start = (5.0955703946591555,-5.376949124262429)
goal = (5.305562956063815,-5.384040312678388)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.305562956063815,-5.384040312678388)
goal = (4.857154686236218,-5.767773444271283)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 137 of 191"
path = []
start = (5.305562956063815,-5.384040312678388)
goal = (5.312389945245476,-5.588657458605415)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 138 of 191"
path = []
start = (5.312389945245476,-5.588657458605415)
goal = (5.374502554114056,-6.09943568080264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 139 of 191"
path = []
start = (4.857154686236218,-5.767773444271283)
goal = (4.230066719736609,-5.7259548216567175)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 140 of 191"
path = []
start = (5.374502554114056,-6.09943568080264)
goal = (5.374352565820994,-6.578273749722536)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 141 of 191"
path = []
start = (5.374352565820994,-6.578273749722536)
goal = (4.78529033137136,-6.5714914922723855)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 142 of 191"
path = []
start = (0.2411284666035165,1.84578877209972)
goal = (-0.2822302812987747,1.7743267689610853)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 143 of 191"
path = []
start = (-0.2822302812987747,1.7743267689610853)
goal = (-1.2095656100517633,1.9278917798000546)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 144 of 191"
path = []
start = (-1.2095656100517633,1.9278917798000546)
goal = (-1.749036401250299,2.234935850381543)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 145 of 191"
path = []
start = (-1.749036401250299,2.234935850381543)
goal = (-1.6494557520553608,2.491518208319448)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.6494557520553608,2.491518208319448)
goal = (-2.0714973056579202,1.5292662122480465)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 146 of 191"
path = []
start = (-1.6494557520553608,2.491518208319448)
goal = (-1.7150026947667492,2.6349473059129895)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 147 of 191"
path = []
start = (-1.7150026947667492,2.6349473059129895)
goal = (-2.1049972049606307,2.6878083793176284)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.1049972049606307,2.6878083793176284)
goal = (-1.1241477976016538,3.8839910236765167)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 148 of 191"
path = []
start = (-2.1049972049606307,2.6878083793176284)
goal = (-3.0214550674276106,2.7636560975802613)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 149 of 191"
path = []
start = (-2.0714973056579202,1.5292662122480465)
goal = (-2.1869156812831223,1.3406536359274988)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 150 of 191"
path = []
start = (-2.1869156812831223,1.3406536359274988)
goal = (-2.4689361614799186,1.0964318093876209)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.4689361614799186,1.0964318093876209)
goal = (-1.8289160063175522,0.6723726781414907)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 151 of 191"
path = []
start = (-2.4689361614799186,1.0964318093876209)
goal = (-2.502419886292736,1.1191811184846667)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 152 of 191"
path = []
start = (-2.502419886292736,1.1191811184846667)
goal = (-3.4392422433191854,0.9384441788873605)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 153 of 191"
path = []
start = (-3.4392422433191854,0.9384441788873605)
goal = (-3.821101511588477,0.020800180291694836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 154 of 191"
path = []
start = (-1.1241477976016538,3.8839910236765167)
goal = (-0.385136603672251,4.283015160385062)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.385136603672251,4.283015160385062)
goal = (-2.2171776378178,4.2298769069829785)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 155 of 191"
path = []
start = (-0.385136603672251,4.283015160385062)
goal = (-0.20434767698801082,4.415830046167084)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.20434767698801082,4.415830046167084)
goal = (-0.04350422050295677,3.546591968056635)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 156 of 191"
path = []
start = (-0.20434767698801082,4.415830046167084)
goal = (0.19102490963110874,4.437931138291246)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.19102490963110874,4.437931138291246)
goal = (-0.05797607385996173,4.889754246580889)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 157 of 191"
path = []
start = (-0.05797607385996173,4.889754246580889)
goal = (0.142644245628043,4.953950006998436)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.142644245628043,4.953950006998436)
goal = (-0.6855094173515859,5.3565726300312875)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 158 of 191"
path = []
start = (-0.6855094173515859,5.3565726300312875)
goal = (-0.7487886723443511,5.649904753937882)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 159 of 191"
path = []
start = (-0.7487886723443511,5.649904753937882)
goal = (-0.9835006592455278,5.9275807933078735)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 160 of 191"
path = []
start = (-0.9835006592455278,5.9275807933078735)
goal = (-1.6089272987334082,5.984524847791562)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.6089272987334082,5.984524847791562)
goal = (-0.8848060856400215,6.554472814732829)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 161 of 191"
path = []
start = (-1.6089272987334082,5.984524847791562)
goal = (-1.741587815056615,5.348047617978363)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.741587815056615,5.348047617978363)
goal = (-2.6539131883866256,6.5927720546908155)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 162 of 191"
path = []
start = (-0.8848060856400215,6.554472814732829)
goal = (1.2258753082161817,7.125425774572886)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 163 of 191"
path = []
start = (-1.741587815056615,5.348047617978363)
goal = (-1.56718458745891,5.248480560154507)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 164 of 191"
path = []
start = (-1.56718458745891,5.248480560154507)
goal = (-1.0747453416207744,4.827994368716086)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 165 of 191"
path = []
start = (-2.2171776378178,4.2298769069829785)
goal = (-3.298816492103146,4.213019240843538)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 166 of 191"
path = []
start = (-3.298816492103146,4.213019240843538)
goal = (-4.345034111540027,4.348772312293349)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 167 of 191"
path = []
start = (-4.345034111540027,4.348772312293349)
goal = (-4.529391724997864,4.516113759023504)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 168 of 191"
path = []
start = (-4.529391724997864,4.516113759023504)
goal = (-4.928561242906252,4.169599683525933)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.928561242906252,4.169599683525933)
goal = (-4.116467292205498,4.966089257166017)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 169 of 191"
path = []
start = (-4.928561242906252,4.169599683525933)
goal = (-4.830388449525914,3.726173125833837)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.830388449525914,3.726173125833837)
goal = (-6.123827859605373,3.8099913675802224)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 170 of 191"
path = []
start = (-2.6539131883866256,6.5927720546908155)
goal = (-2.064697079964173,7.242762812180523)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.064697079964173,7.242762812180523)
goal = (-3.56529238241625,7.1421563311676675)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 171 of 191"
path = []
start = (-3.56529238241625,7.1421563311676675)
goal = (-4.528614453505079,7.199287732083993)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 172 of 191"
path = []
start = (-6.123827859605373,3.8099913675802224)
goal = (-6.385104595841781,3.7617612128592537)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 173 of 191"
path = []
start = (-6.385104595841781,3.7617612128592537)
goal = (-6.82180851387848,3.93839551764421)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 174 of 191"
path = []
start = (-6.82180851387848,3.93839551764421)
goal = (-6.630036345191415,4.597822853477442)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 175 of 191"
path = []
start = (-6.630036345191415,4.597822853477442)
goal = (-6.377984500145175,5.396300784778166)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.377984500145175,5.396300784778166)
goal = (-7.561618269026916,4.93293868827023)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 176 of 191"
path = []
start = (-6.377984500145175,5.396300784778166)
goal = (-6.128835632265561,5.592129251901678)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.128835632265561,5.592129251901678)
goal = (-6.912178670727236,6.16028156272977)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 177 of 191"
path = []
start = (-6.128835632265561,5.592129251901678)
goal = (-5.420878044701133,5.848165333164024)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 178 of 191"
path = []
start = (-6.912178670727236,6.16028156272977)
goal = (-7.061814823319533,6.597701109143944)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 179 of 191"
path = []
start = (-7.061814823319533,6.597701109143944)
goal = (-6.460404183559374,6.589717975906094)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.460404183559374,6.589717975906094)
goal = (-7.729985371585673,7.0638477772804364)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 180 of 191"
path = []
start = (-6.460404183559374,6.589717975906094)
goal = (-6.472300169094789,7.021648022242278)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 181 of 191"
path = []
start = (-6.472300169094789,7.021648022242278)
goal = (-6.6383024156587656,7.029296977703918)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 182 of 191"
path = []
start = (-7.561618269026916,4.93293868827023)
goal = (-7.67285453460094,4.618253137246025)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.67285453460094,4.618253137246025)
goal = (-7.94487629568995,5.035900441476759)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 183 of 191"
path = []
start = (-7.67285453460094,4.618253137246025)
goal = (-8.268403488412526,4.374660711737622)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 184 of 191"
path = []
start = (-7.94487629568995,5.035900441476759)
goal = (-7.856279565490793,5.470938038282977)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 185 of 191"
path = []
start = (-7.856279565490793,5.470938038282977)
goal = (-8.309090929044867,5.872097988684901)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 186 of 191"
path = []
start = (-8.309090929044867,5.872097988684901)
goal = (-8.386679549676279,5.852281211413539)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.386679549676279,5.852281211413539)
goal = (-8.147269232303593,6.134394540590479)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 187 of 191"
path = []
start = (-8.268403488412526,4.374660711737622)
goal = (-8.635848467958363,4.44173853660924)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.635848467958363,4.44173853660924)
goal = (-8.362956368651343,3.888548613131757)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 188 of 191"
path = []
start = (-8.362956368651343,3.888548613131757)
goal = (-8.158941339318433,3.336775824326674)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 189 of 191"
path = []
start = (-8.158941339318433,3.336775824326674)
goal = (-7.193607478491405,2.597333323508348)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 190 of 191"
path = []
start = (-7.193607478491405,2.597333323508348)
goal = (-8.063830102695928,1.8978453881129447)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 191 of 191"
path = []
start = (-8.063830102695928,1.8978453881129447)
goal = (-8.41954134327548,1.935174233290489)
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
f = open('smo2sol-16.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
