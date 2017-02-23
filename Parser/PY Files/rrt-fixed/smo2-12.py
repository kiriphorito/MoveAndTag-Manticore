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

obstacleList = [[(0.027921418322840365,-0.1830299372950328),(1.7964909357135048,0.7508661287404507),(1.3295429026957633,1.635150887435783),(0.44525814400043096,1.1682028544180412),(-0.4886379220350525,2.936772371808706),(-1.3729226807303845,2.469824338790964),(-0.9059746477126429,1.5855395800956318),(-1.7902594064079753,1.11859154707789),(-2.7241554724434587,2.887161064468554),(-3.608440231138791,2.420213031450813),(-2.2075961320855657,-0.2326412446351836),(-0.4390266146949013,0.7012548214002995)],[(3.2667894218746163,-0.9293276751313315),(3.4163915262659503,-2.923724629190798),(4.413590003295685,-2.8489235769951335),(4.3387889511000175,-1.8517250999653987),(6.333185905159487,-1.702122995574062),(6.258384852963818,-0.7049245185443274),(5.2611863759340824,-0.7797255707399942),(5.186385323738417,0.2174729062897387),(7.180782277797881,0.3670750106810783),(7.105981225602218,1.364273487710809),(6.108782748572482,1.2894724355151381),(6.033981696376816,2.286670912544872),(5.11158427154275,1.2146713833194713),(4.114385794513018,1.1398703311238023),(4.263987898904349,-0.8545266229356628)],[(0.1477600880102665,-0.38369629930319715),(-1.8521625922360219,-0.40128245967152837),(-1.8433695120518563,-1.4012437997946727),(-0.8434081719287114,-1.392450719610508),(-0.825822011560381,-3.3923733998567958),(0.1741393285627631,-3.38358031967263),(0.16534624837859757,-2.3836189795494853),(1.1653075885017385,-2.374825899365322),(0.15655316819443205,-1.3836576394263413)],[(-4.563997143186996,0.8900626785417137),(-6.536549868935029,1.2202682874341066),(-6.866755477827425,-0.7522844383139272)],[(0.9770672007274795,0.1125220795952703),(1.3969838164696002,-1.8428986004180306),(2.3746941564762536,-1.6329402925469705),(2.1647358486051895,-0.6552299525403225),(4.120156528618494,-0.23531333679820224),(3.910198220747431,0.7423970032084566),(2.932487880740778,0.5324386953373947),(2.7225295728697234,1.5101490353440385),(1.9547775407341295,0.32248038746633023)],[(-2.7217987212462944,-0.25386203183501677),(-4.085694638381039,1.2089393652189955),(-4.8170953369080465,0.5269914066516261),(-4.135147378340674,-0.2044092918753825),(-5.597948775394688,-1.568305209010128),(-4.916000816827314,-2.2997059075371338),(-4.184600118300308,-1.617757948969761),(-3.502652159732941,-2.349158647496767),(-3.453199419773301,-0.935809990402389)],[(-4.7756336778783295,2.937843288987294),(-4.816459382343165,4.937426561035482),(-5.816251018367256,4.91701370880307),(-5.7958381661348435,3.9172220727789737),(-7.795421438183031,3.876396368314145),(-7.775008585950617,2.8766047322900525),(-6.775216949926518,2.8970175845224655),(-6.754804097694104,1.897225948498364),(-5.775425313902424,2.917430436754879)],[(-0.5084691217977574,4.737720380854732),(-2.1883934449156897,5.823010327641649),(-3.273683391702611,4.143086004523719)],[(1.1750874452596143,-4.436934212974815),(-0.8184441004193554,-4.597657504326156),(-0.7380824547436882,-5.594423277165642),(0.2586833180957948,-5.514061631489968),(0.41940660944713637,-7.507593177168945),(1.4161723822866152,-7.427231531493277),(1.3358107366109497,-6.430465758653785),(2.332576509450437,-6.35010411297812),(2.4932998008017755,-8.343635658657089),(3.4900655736412594,-8.263274012981427),(3.2489806366142595,-5.272976694462961),(1.255449090935282,-5.4336999858142985)],[(-2.6687620135851873,-3.806657257270425),(-4.6687464707193715,-3.7987723843052095),(-4.67268890720198,-4.798764612872306),(-3.6726966786348862,-4.80270704935491),(-3.6805815516001035,-6.8026915064890945),(-2.680589323033011,-6.806633942971702),(-2.6766468865504067,-5.806641714404609),(-1.676654657983314,-5.81058415088722),(-2.6727044500677968,-4.806649485837517)],[(1.6999654603240348,1.8922045906847533),(3.6201441123462414,1.3328174757829359),(3.89983766979715,2.2929068017940377),(2.939748343786048,2.572600359244947),(3.499135458687869,4.492779011267153),(2.539046132676768,4.772472568718062),(2.259352575225855,3.812383242706959),(1.2992632492147564,4.09207680015787),(1.9796590177749451,2.852293916695856)],[(8.924741223152287,3.525093572371839),(6.9854111067580025,4.013968504891513),(6.74097364049816,3.044303446694375),(7.7106386986953,2.7998659804345363),(7.221763766175628,0.8605358640402447),(8.191428824372784,0.6160983977804082),(8.43586629063262,1.5857634559775537),(9.405531348829763,1.3413259897177134),(8.68030375689245,2.5554285141746957)],[(-3.770456394509376,7.0105161125944235),(-4.928559655075919,8.641097860887081),(-5.743850529222253,8.062046230603805),(-5.1647988989389795,7.246755356457475),(-6.7953806472316325,6.088652095890927),(-6.216329016948357,5.2733612217446),(-5.401038142802029,5.852412852027875),(-4.821986512518751,5.037121977881544),(-4.585747268655703,6.431464482311149)]]
rand = (-8, 10)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (7.6821142007759216,-3.142603247726372)
goal = (7.78365880818954,-3.4483479697935593)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.78365880818954,-3.4483479697935593)
goal = (7.247696385490086,-2.193714815400223)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.247696385490086,-2.193714815400223)
goal = (8.627835954294845,-2.234198701537699)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.627835954294845,-2.234198701537699)
goal = (8.371624219731054,-0.7601206801530838)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.371624219731054,-0.7601206801530838)
goal = (6.2798695167028304,-5.649791827022035)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.2798695167028304,-5.649791827022035)
goal = (7.683766480465935,-7.562944441017253)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.683766480465935,-7.562944441017253)
goal = (0.9519447731170088,3.2512712317105557)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (7.78365880818954,-3.4483479697935593)
goal = (6.610787027686544,-2.8205991177879524)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.610787027686544,-2.8205991177879524)
goal = (8.258449113334542,-4.799413198805823)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.258449113334542,-4.799413198805823)
goal = (9.044395030272305,-5.111614043822955)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.044395030272305,-5.111614043822955)
goal = (6.7396795474811,-5.438077184115825)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.7396795474811,-5.438077184115825)
goal = (5.374983307762112,-5.061232278974865)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.374983307762112,-5.061232278974865)
goal = (6.419989150803661,-7.688629958697222)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.419989150803661,-7.688629958697222)
goal = (5.146192006974119,6.095865608140683)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (6.610787027686544,-2.8205991177879524)
goal = (5.577817291046089,-3.0668683039888442)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.577817291046089,-3.0668683039888442)
goal = (5.566293527348884,-3.432843272760386)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.566293527348884,-3.432843272760386)
goal = (5.140620113452413,-2.2045555788403366)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.140620113452413,-2.2045555788403366)
goal = (5.159447632311655,-4.968104374164314)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.159447632311655,-4.968104374164314)
goal = (2.09368267578112,-2.599694731941784)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.09368267578112,-2.599694731941784)
goal = (-2.015534076294675,-0.7811783581025589)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (7.247696385490086,-2.193714815400223)
goal = (7.140210630625551,-1.6861445215195232)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.140210630625551,-1.6861445215195232)
goal = (7.74092259539803,-1.2104528818865807)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.74092259539803,-1.2104528818865807)
goal = (5.325332075689899,-1.795035934007145)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.325332075689899,-1.795035934007145)
goal = (7.326354019769551,0.120713688947097)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.326354019769551,0.120713688947097)
goal = (5.712805332761625,-7.548739531422378)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.712805332761625,-7.548739531422378)
goal = (4.584073202716692,5.973861425067037)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (8.258449113334542,-4.799413198805823)
goal = (8.65180739169455,-5.432924434998573)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.65180739169455,-5.432924434998573)
goal = (7.317120129397964,-5.884628082362708)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.317120129397964,-5.884628082362708)
goal = (7.48583159401633,-6.481404157142331)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.48583159401633,-6.481404157142331)
goal = (8.54386029301051,-7.880374284516762)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.54386029301051,-7.880374284516762)
goal = (-2.2892298420566304,-1.252208420895868)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (5.577817291046089,-3.0668683039888442)
goal = (4.810670704974643,-2.6173397029957766)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.810670704974643,-2.6173397029957766)
goal = (4.2642764162160605,-3.7141666091773056)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.2642764162160605,-3.7141666091773056)
goal = (4.5433626789473704,-5.14031598080557)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.5433626789473704,-5.14031598080557)
goal = (3.9055220365324237,-6.887987393418964)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.9055220365324237,-6.887987393418964)
goal = (-2.9944247492369547,-2.8165341319636035)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (7.140210630625551,-1.6861445215195232)
goal = (8.217350193270164,-0.8534220402183186)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.217350193270164,-0.8534220402183186)
goal = (6.1462283743585955,-0.5588513708519018)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.1462283743585955,-0.5588513708519018)
goal = (5.54478295000066,-0.22991192073241074)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.54478295000066,-0.22991192073241074)
goal = (2.1353842105433847,0.8116109666115108)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.1353842105433847,0.8116109666115108)
goal = (2.939119968962152,5.4999809220274685)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (8.627835954294845,-2.234198701537699)
goal = (8.743552636573892,-0.7521764890753122)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.743552636573892,-0.7521764890753122)
goal = (8.891789544563746,-0.7399953557012724)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.891789544563746,-0.7399953557012724)
goal = (9.252726485855185,1.0448738266307362)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.252726485855185,1.0448738266307362)
goal = (8.069484088309107,-8.212977176798056)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.069484088309107,-8.212977176798056)
goal = (5.737680201656475,6.6322170699573455)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (9.044395030272305,-5.111614043822955)
goal = (9.302493901940263,-5.960969037606572)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.302493901940263,-5.960969037606572)
goal = (9.27849739605886,-6.370493056233946)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.27849739605886,-6.370493056233946)
goal = (7.848535503219066,-8.319934012141212)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.848535503219066,-8.319934012141212)
goal = (-3.087181049682096,-3.416789509231588)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (5.566293527348884,-3.432843272760386)
goal = (5.098541283437375,-4.451241706421511)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.098541283437375,-4.451241706421511)
goal = (3.3201134942073702,-4.557775577580541)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.3201134942073702,-4.557775577580541)
goal = (4.165619051154031,-7.040747295993559)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.165619051154031,-7.040747295993559)
goal = (-3.2238109732427684,-3.5165569279733946)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (7.74092259539803,-1.2104528818865807)
goal = (7.38935067439106,-0.14128448160103524)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.38935067439106,-0.14128448160103524)
goal = (6.749814061646705,1.9036447605722202)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.749814061646705,1.9036447605722202)
goal = (5.042317993093383,3.5435884403746343)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.042317993093383,3.5435884403746343)
goal = (6.477694151884578,7.045582605172694)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (8.65180739169455,-5.432924434998573)
goal = (8.186699224717422,-6.428739909646334)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.186699224717422,-6.428739909646334)
goal = (8.146336074655643,-6.6755178581577335)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.146336074655643,-6.6755178581577335)
goal = (4.604059932286181,-7.31775307893116)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.604059932286181,-7.31775307893116)
goal = (-3.2412265168175187,-2.098039694344635)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (4.810670704974643,-2.6173397029957766)
goal = (5.601762032308945,-0.7144448791490223)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.601762032308945,-0.7144448791490223)
goal = (2.7030542564712485,-3.5231484775748934)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.7030542564712485,-3.5231484775748934)
goal = (1.4648196670661973,-4.589632231767439)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.4648196670661973,-4.589632231767439)
goal = (-1.7993096294928899,1.7206080792464693)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (8.217350193270164,-0.8534220402183186)
goal = (8.1958430134109,-0.12139773477623095)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.1958430134109,-0.12139773477623095)
goal = (8.95952650093703,2.6808519574481817)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.95952650093703,2.6808519574481817)
goal = (8.961187118252358,4.516657409130357)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.961187118252358,4.516657409130357)
goal = (4.5573552603042,7.103879800475303)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (8.743552636573892,-0.7521764890753122)
goal = (9.321626855427624,-0.04978279824589116)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.321626855427624,-0.04978279824589116)
goal = (7.491843936655193,2.8075061125642815)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.491843936655193,2.8075061125642815)
goal = (7.687608975429918,5.123220208568274)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.687608975429918,5.123220208568274)
goal = (8.01863094418086,8.072533819767074)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (8.371624219731054,-0.7601206801530838)
goal = (7.232727162237755,-0.17511971001195903)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.232727162237755,-0.17511971001195903)
goal = (6.270799910449518,2.3921271006711518)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.270799910449518,2.3921271006711518)
goal = (8.137401992974862,5.49994221623661)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.137401992974862,5.49994221623661)
goal = (3.3308365308594388,6.449740616333619)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (6.7396795474811,-5.438077184115825)
goal = (6.378035183213094,-6.003108561784058)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.378035183213094,-6.003108561784058)
goal = (4.330532706667959,-7.7141372437897795)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.330532706667959,-7.7141372437897795)
goal = (-3.198196802297689,-1.2778647415293367)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (5.140620113452413,-2.2045555788403366)
goal = (2.5551853485816833,-1.88817110530072)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.5551853485816833,-1.88817110530072)
goal = (1.1003139126065227,-1.1574742544638115)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1003139126065227,-1.1574742544638115)
goal = (-0.25794407086140847,3.5895525418902636)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (5.325332075689899,-1.795035934007145)
goal = (4.1528773027870605,0.4270932838469612)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.1528773027870605,0.4270932838469612)
goal = (1.0482104991472,-0.986181328009966)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.0482104991472,-0.986181328009966)
goal = (-0.22954795137443096,3.9956271116563578)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (7.317120129397964,-5.884628082362708)
goal = (7.396370526784085,-6.483059104492736)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.396370526784085,-6.483059104492736)
goal = (4.093743498119525,-7.565095972189655)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.093743498119525,-7.565095972189655)
goal = (-1.9093007387677687,1.6916740752165165)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (4.2642764162160605,-3.7141666091773056)
goal = (2.469213806389594,-4.183813780946509)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.469213806389594,-4.183813780946509)
goal = (1.654674185412504,-4.971232882210193)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.654674185412504,-4.971232882210193)
goal = (-1.6497346392141132,2.8677804563086795)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (6.1462283743585955,-0.5588513708519018)
goal = (5.562651633735988,1.9307319104899339)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.562651633735988,1.9307319104899339)
goal = (2.2214061659644075,1.5855219981843227)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.2214061659644075,1.5855219981843227)
goal = (2.8164707531792876,6.203876401573796)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (8.891789544563746,-0.7399953557012724)
goal = (9.292888118643265,3.055588809776788)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.292888118643265,3.055588809776788)
goal = (9.08667467555098,5.5494730981797815)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.08667467555098,5.5494730981797815)
goal = (6.649604751742884,7.997451204097917)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (9.302493901940263,-5.960969037606572)
goal = (8.783095835588632,-7.337741498139486)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.783095835588632,-7.337741498139486)
goal = (3.7894303485956753,-8.029020829741128)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.7894303485956753,-8.029020829741128)
goal = (0.8005249815561397,5.032914566000315)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (5.098541283437375,-4.451241706421511)
goal = (4.940263115326927,-6.08037751279765)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.940263115326927,-6.08037751279765)
goal = (2.1066293217574055,-7.996007452068367)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.1066293217574055,-7.996007452068367)
goal = (-4.680343487767187,-3.5281377807113907)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (7.38935067439106,-0.14128448160103524)
goal = (4.655242594504206,1.6760151774734204)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.655242594504206,1.6760151774734204)
goal = (5.944666814825621,5.139548412093086)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.944666814825621,5.139548412093086)
goal = (3.221146021758824,6.568960113238145)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (8.186699224717422,-6.428739909646334)
goal = (7.476609074153619,-6.955787442947004)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.476609074153619,-6.955787442947004)
goal = (0.4587499352261242,-3.8566527945239253)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.4587499352261242,-3.8566527945239253)
goal = (-4.725856596829488,-3.0539526368239667)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (5.601762032308945,-0.7144448791490223)
goal = (2.382544454551291,-1.7015648089754407)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.382544454551291,-1.7015648089754407)
goal = (0.8505236343626281,-0.952750617671585)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.8505236343626281,-0.952750617671585)
goal = (-0.7717077174678124,4.0369980604412685)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (8.1958430134109,-0.12139773477623095)
goal = (9.04826241728654,3.7036745609407067)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.04826241728654,3.7036745609407067)
goal = (7.088522739501156,5.687626292371384)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.088522739501156,5.687626292371384)
goal = (4.245336867951841,7.275486916586985)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (9.321626855427624,-0.04978279824589116)
goal = (9.375790680525533,4.169756961043822)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.375790680525533,4.169756961043822)
goal = (7.324691565217216,5.854949856434258)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.324691565217216,5.854949856434258)
goal = (3.046098105241412,6.87968894310783)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (7.232727162237755,-0.17511971001195903)
goal = (5.331869379290169,2.982659084050246)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.331869379290169,2.982659084050246)
goal = (3.856801609477036,4.108063668850743)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.856801609477036,4.108063668850743)
goal = (1.19744841436757,5.966603415942711)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (6.2798695167028304,-5.649791827022035)
goal = (5.8215225714659455,-5.9580499168446535)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.8215225714659455,-5.9580499168446535)
goal = (-0.18938122385642586,-4.460345410170558)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.18938122385642586,-4.460345410170558)
goal = (-5.144115042019992,-4.916630913914929)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (5.374983307762112,-5.061232278974865)
goal = (-0.2815645320419362,-3.429190694093492)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.2815645320419362,-3.429190694093492)
goal = (-4.719210146932047,-2.264901200466184)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (5.159447632311655,-4.968104374164314)
goal = (-0.07717144079110483,-7.751085877816356)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.07717144079110483,-7.751085877816356)
goal = (-5.146072803414965,-2.9169209273246937)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (7.326354019769551,0.120713688947097)
goal = (5.924427915628104,5.465921502406344)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.924427915628104,5.465921502406344)
goal = (5.819657252347578,8.62113211181996)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (7.48583159401633,-6.481404157142331)
goal = (0.0456262913011507,-8.149918458735568)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.0456262913011507,-8.149918458735568)
goal = (-5.0899441170900825,-8.28426277654686)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (4.5433626789473704,-5.14031598080557)
goal = (-0.4997821991176865,-7.0744563623762335)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.4997821991176865,-7.0744563623762335)
goal = (-5.331859238721161,-5.560308597715808)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (5.54478295000066,-0.22991192073241074)
goal = (1.9064649363744293,2.9682925149977724)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.9064649363744293,2.9682925149977724)
goal = (-1.7935453736692537,2.939140142764913)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (9.252726485855185,1.0448738266307362)
goal = (8.727051035323509,6.2446002000914245)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.727051035323509,6.2446002000914245)
goal = (3.4040381533635165,8.16431699712291)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (9.27849739605886,-6.370493056233946)
goal = (-0.8368094601461182,-7.074807330195499)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8368094601461182,-7.074807330195499)
goal = (-5.395145482390309,-6.112232300050884)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (3.3201134942073702,-4.557775577580541)
goal = (-0.8446053949766119,-4.225593780130989)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8446053949766119,-4.225593780130989)
goal = (-5.3962327329907795,-4.617750927224565)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (6.749814061646705,1.9036447605722202)
goal = (5.103891488076653,5.415942690205117)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.103891488076653,5.415942690205117)
goal = (1.914891879446242,7.033689004842456)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (8.146336074655643,-6.6755178581577335)
goal = (-0.8233301517755436,-7.7389830869513165)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8233301517755436,-7.7389830869513165)
goal = (-5.790471221779809,-6.288804510175186)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (2.7030542564712485,-3.5231484775748934)
goal = (0.3449505914492592,-0.6852251273524761)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.3449505914492592,-0.6852251273524761)
goal = (-3.3408096944935473,1.2765473038226656)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (8.95952650093703,2.6808519574481817)
goal = (8.137448689376605,6.582037542919185)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.137448689376605,6.582037542919185)
goal = (2.249563739955275,8.254390757080415)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (7.491843936655193,2.8075061125642815)
goal = (7.16401658471925,6.250397211677342)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.16401658471925,6.250397211677342)
goal = (0.9527013426575985,6.365693712192989)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (6.270799910449518,2.3921271006711518)
goal = (5.185985885947987,5.498206609623342)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.185985885947987,5.498206609623342)
goal = (-0.03828609795193838,5.609757168569464)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (6.378035183213094,-6.003108561784058)
goal = (-1.4992700178443261,-6.7906158990373475)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.4992700178443261,-6.7906158990373475)
goal = (-5.703396538812173,-4.999835077388437)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (2.5551853485816833,-1.88817110530072)
goal = (-0.2809932851711068,-0.14372989898545185)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.2809932851711068,-0.14372989898545185)
goal = (-3.925983562167175,1.1402358286740029)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (4.1528773027870605,0.4270932838469612)
goal = (1.064120913442646,3.081726934492732)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.064120913442646,3.081726934492732)
goal = (-0.9006843981021166,4.632576925341818)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (7.396370526784085,-6.483059104492736)
goal = (-1.2804296829253907,-8.095087514336058)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.2804296829253907,-8.095087514336058)
goal = (-5.977840049126939,-4.324874865916809)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (2.469213806389594,-4.183813780946509)
goal = (-1.691127125386708,-2.7426088952611565)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.691127125386708,-2.7426088952611565)
goal = (-5.297405297267645,-1.0614543417646445)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (5.562651633735988,1.9307319104899339)
goal = (3.4655944869015913,5.073728705571655)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.4655944869015913,5.073728705571655)
goal = (-0.21743997290762618,5.504537479157456)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (9.292888118643265,3.055588809776788)
goal = (8.923155873862239,6.824842589552135)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.923155873862239,6.824842589552135)
goal = (0.8070121028294723,7.024037678610911)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (8.783095835588632,-7.337741498139486)
goal = (-1.6034477477136768,-8.20361746281792)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.6034477477136768,-8.20361746281792)
goal = (-6.357378320144063,-5.547470724719226)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (4.940263115326927,-6.08037751279765)
goal = (-1.7672513410221535,-7.011604280233568)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.7672513410221535,-7.011604280233568)
goal = (-6.625286749016157,-6.39286503225026)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (4.655242594504206,1.6760151774734204)
goal = (2.2174559888963,4.871234144903033)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.2174559888963,4.871234144903033)
goal = (-0.6722480798523458,5.586794674717098)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (7.476609074153619,-6.955787442947004)
goal = (-2.269679996021603,-7.8920481431784)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.269679996021603,-7.8920481431784)
goal = (-3.8665533127745886,1.6348401244493793)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (2.382544454551291,-1.7015648089754407)
goal = (0.33875907030922736,1.980171180888794)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.33875907030922736,1.980171180888794)
goal = (-2.91620937262325,2.9662214616239044)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (9.04826241728654,3.7036745609407067)
goal = (6.490392562469052,6.161355651384534)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.490392562469052,6.161355651384534)
goal = (-0.22755119944493263,7.617081160428009)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (9.375790680525533,4.169756961043822)
goal = (7.513238589219591,7.60132312135161)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.513238589219591,7.60132312135161)
goal = (-0.3446613202916611,7.71871304992715)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (5.331869379290169,2.982659084050246)
goal = (4.879600499759301,5.90208456610328)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.879600499759301,5.90208456610328)
goal = (-0.8354154797758895,6.129293834065752)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (5.8215225714659455,-5.9580499168446535)
goal = (-2.468663521478466,-6.41814495328922)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.468663521478466,-6.41814495328922)
goal = (-6.45670406858441,-3.2919754283531795)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (7.683766480465935,-7.562944441017253)
goal = (-3.594084103197253,-8.278425126784786)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.594084103197253,-8.278425126784786)
goal = (-6.788710219247133,-6.809914576183789)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (6.419989150803661,-7.688629958697222)
goal = (-7.180828576065353,-7.15307814911443)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (2.09368267578112,-2.599694731941784)
goal = (-2.074544164634987,3.9933587338083267)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (5.712805332761625,-7.548739531422378)
goal = (-7.149283611753054,-5.92837166987176)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (8.54386029301051,-7.880374284516762)
goal = (-7.083533416257447,-5.426417760692637)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (3.9055220365324237,-6.887987393418964)
goal = (-7.155999137186815,-4.8825634975809304)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (2.1353842105433847,0.8116109666115108)
goal = (-2.4502630975649895,4.019002219158061)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (8.069484088309107,-8.212977176798056)
goal = (-7.044601280886091,-3.982867643748409)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (7.848535503219066,-8.319934012141212)
goal = (-7.734908628140652,-7.514595499920434)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (4.165619051154031,-7.040747295993559)
goal = (-6.980854490140525,-3.3906283781280306)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (5.042317993093383,3.5435884403746343)
goal = (-0.5509716238203763,8.223244256377331)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (4.604059932286181,-7.31775307893116)
goal = (-7.20737875023794,-3.4990688427462757)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (1.4648196670661973,-4.589632231767439)
goal = (-3.7599563547246966,2.720917918443499)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (8.961187118252358,4.516657409130357)
goal = (-2.0786224080993483,6.2129036190974105)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (7.687608975429918,5.123220208568274)
goal = (-2.133944530139626,7.479275286692548)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (8.137401992974862,5.49994221623661)
goal = (-2.4284818718739336,7.960773737356131)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (4.330532706667959,-7.7141372437897795)
goal = (-7.743249375750193,-5.787384952539038)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (1.1003139126065227,-1.1574742544638115)
goal = (-3.14978257978194,3.3779278293856017)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (1.0482104991472,-0.986181328009966)
goal = (-4.368351891129993,2.6615618480757863)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (4.093743498119525,-7.565095972189655)
goal = (-7.730587833929091,-4.831483111591755)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (1.654674185412504,-4.971232882210193)
goal = (-7.39890513864496,-2.9491457484344945)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (2.2214061659644075,1.5855219981843227)
goal = (-3.3547321499186022,3.874239183922283)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (9.08667467555098,5.5494730981797815)
goal = (-2.8468652301573583,5.57329513307217)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (3.7894303485956753,-8.029020829741128)
goal = (-7.4412155165744815,-2.1181949015372625)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (2.1066293217574055,-7.996007452068367)
goal = (-7.000443832272749,-1.3454983081283682)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (5.944666814825621,5.139548412093086)
goal = (-2.8428844671861864,7.325311325192821)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (0.4587499352261242,-3.8566527945239253)
goal = (-7.424295757755009,-1.9268652665089059)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (0.8505236343626281,-0.952750617671585)
goal = (-4.644778321085605,2.8899292199198108)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (7.088522739501156,5.687626292371384)
goal = (-2.740653340354023,8.141881311315885)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (7.324691565217216,5.854949856434258)
goal = (-3.1911837906221026,6.987711840778401)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (3.856801609477036,4.108063668850743)
goal = (-3.334789255046844,4.968868484756765)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (-0.18938122385642586,-4.460345410170558)
goal = (-5.814208605213642,1.8985063690038828)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (-0.2815645320419362,-3.429190694093492)
goal = (-4.592238644277522,3.0316619167432073)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (-0.07717144079110483,-7.751085877816356)
goal = (-6.881197641888339,1.2169422608087253)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (5.924427915628104,5.465921502406344)
goal = (-3.8363630462954967,4.8489633305049225)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (0.0456262913011507,-8.149918458735568)
goal = (-6.503841478289496,1.801739786033453)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (-0.4997821991176865,-7.0744563623762335)
goal = (-6.426095435262343,2.1342378414569225)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (1.9064649363744293,2.9682925149977724)
goal = (-4.087630810768225,4.2123575537687294)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (8.727051035323509,6.2446002000914245)
goal = (-3.5753013175251063,8.004942199171982)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (-0.8368094601461182,-7.074807330195499)
goal = (-5.7928216354855,2.6887308412620374)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (-0.8446053949766119,-4.225593780130989)
goal = (-6.02574679284762,4.619647944119848)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (5.103891488076653,5.415942690205117)
goal = (-4.529644763976411,5.490795166772164)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-0.8233301517755436,-7.7389830869513165)
goal = (-6.344960317326084,5.101468629442879)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (0.3449505914492592,-0.6852251273524761)
goal = (-5.251454417022723,5.556557131742785)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (8.137448689376605,6.582037542919185)
goal = (-6.135037170772724,6.942347605545164)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (7.16401658471925,6.250397211677342)
goal = (-6.201721811645252,6.872138138019096)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (5.185985885947987,5.498206609623342)
goal = (-6.265339682341303,6.796610461226068)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (-1.4992700178443261,-6.7906158990373475)
goal = (-7.528886792210389,4.663811819619728)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (-0.2809932851711068,-0.14372989898545185)
goal = (-7.0478046246420245,5.325197521680828)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (1.064120913442646,3.081726934492732)
goal = (-6.581820982212836,6.6147729770969725)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (-1.2804296829253907,-8.095087514336058)
goal = (-7.609839906644986,5.739163422584786)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (-1.691127125386708,-2.7426088952611565)
goal = (-7.421128499090359,6.471246355849001)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (3.4655944869015913,5.073728705571655)
goal = (-6.4338644451853995,7.643830412415074)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (8.923155873862239,6.824842589552135)
goal = (-6.21365528385758,8.631517798650869)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (-1.6034477477136768,-8.20361746281792)
goal = (-6.508852548538802,7.4247668690062)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (-1.7672513410221535,-7.011604280233568)
goal = (-6.483120620605889,7.884938835580435)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (2.2174559888963,4.871234144903033)
goal = (-6.909225463710415,8.157080495377816)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-2.269679996021603,-7.8920481431784)
goal = (-7.11305603081267,8.079559706373612)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (0.33875907030922736,1.980171180888794)
goal = (-6.8383014246278755,8.362180824436198)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (6.490392562469052,6.161355651384534)
goal = (-7.69870508348387,8.075155173441729)
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
f = open('smo2sol-12.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
