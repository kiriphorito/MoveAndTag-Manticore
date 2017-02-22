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
#    rrt.DrawGaph()
#    plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')
#    print path
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-y')
    smoothiePath.reverse()
    #print smoothiePath
    return smoothiePath

obstacleList = [[(0.04230063257291906,-0.023309859713154173),(-0.6006170507409412,1.870537244063267),(-1.5475406026291516,1.5490784024063373),(-1.2260817609722214,0.6021548505181268),(-3.119928864748643,-0.040762832795733166),(-2.7984700230917134,-0.9876863846839437),(-1.8515464712035024,-0.6662275430270138),(-1.5300876295465724,-1.6131510949152248),(-0.9046229193152917,-0.34476870137008403)],[(0.021050584626286218,-0.1246147185685608),(1.6074002920510229,-1.3426026519051162),(2.825388225387578,0.2437470555196207)],[(-1.9054061666614464,0.8259739231831521),(-1.069591751062994,2.6429533569351737),(-1.9780814679390044,3.0608605647343996),(-2.3959886757382316,2.1523708478583887),(-4.212968109490255,2.9881852634568395),(-4.63087531728948,2.0796955465808313),(-3.7223856004134674,1.6617883387816044),(-4.140292808212695,0.7532986219055937),(-5.957272241964716,1.5891130375040423),(-6.375179449763942,0.6806233206280314),(-3.6497102991359105,-0.5730983027696438),(-2.813895883537457,1.2438811309823778)],[(-0.5076452870661072,-1.7463225640676328),(-2.4995106012460138,-1.5661210930220897),(-2.5896113367687876,-2.562053750112043),(-1.5936786796788316,-2.6521544856348167),(-1.7738801507243807,-4.6440197998147195),(-0.7779474936344221,-4.734120535337494),(-0.6878467581116503,-3.7381878782475386),(0.30808589897830496,-3.8282886137703103),(-0.5977460225888789,-2.742255221157586)],[(0.3145664430887909,0.12268109134210509),(1.6110669740358399,1.6455357703098714),(0.0882122950680774,2.9420363012569215)],[(3.125831586905756,1.9032886111206666),(5.124947160892275,1.962760636851655),(5.095211148026783,2.9623184238449136),(4.095653361033522,2.932582410979421),(4.036181335302539,4.931697984965939),(3.036623548309279,4.9019619721004455),(3.0663595611747696,3.902404185107184),(2.0668017741815103,3.8726681722416956),(2.0073297484505197,5.871783746228211),(1.0077719614572693,5.842047733362724),(1.0375079743227582,4.8424899463694615),(0.03795018732950117,4.81275393350397),(-0.021521838401487692,6.8118695074904885),(-1.021079625394747,6.782133494624996),(-0.931871586798267,3.783460133645219),(1.0672439871882524,3.842932159376203),(1.096980000053743,2.843374372382939),(3.0960955740402625,2.9028463981139248)],[(0.9957233475665478,-2.1548202924721807),(0.6414240497995395,-4.123188146203237),(1.6256079766650617,-4.300337795086742),(1.8027576255485664,-3.3161538682212166),(3.771125479279624,-3.6704531659882287),(3.9482751281631336,-2.6862692391226934),(2.964091201297603,-2.5091195902391927),(3.1412408501811098,-1.5249356633736668),(1.9799072744320758,-2.3319699413556867)],[(-5.260975596065909,-5.212491611368591),(-6.402583874376196,-6.854664117195375),(-5.581497621462799,-7.425468256350522),(-5.010693482307659,-6.604382003437127),(-3.3685209764808723,-7.745990281747413),(-2.797716837325732,-6.924904028834025),(-3.618803090239128,-6.354099889678878),(-3.047998951083982,-5.533013636765493),(-4.439889343152517,-5.783295750523735)],[(-3.2569534581939106,2.6082715000716346),(-1.7518409038858023,3.9253273830409587),(-3.0688967868551282,5.430439937349074)],[(-2.22175104070234,-3.277538031857934),(-3.7098957999148476,-4.613736061952405),(-2.3736977698203763,-6.101880821164915)]]
rand = ()

content = ""
print "Path 1 of 123"
path = []
start = (-0.8040387650673786,-6.937588772034066)
goal = (-1.061516005317677,-6.768980994246642)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.061516005317677,-6.768980994246642)
goal = (-0.8165469323486505,-7.521330231277359)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.8165469323486505,-7.521330231277359)
goal = (0.022564139302638786,-7.1532746075298785)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.022564139302638786,-7.1532746075298785)
goal = (-0.18055228027303016,-6.055357926857768)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.18055228027303016,-6.055357926857768)
goal = (0.6582656196446202,-6.103986250237439)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.6582656196446202,-6.103986250237439)
goal = (-0.020614520550903848,-3.258097169967175)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.020614520550903848,-3.258097169967175)
goal = (-3.4699446918809387,-0.8349444966175694)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (-1.061516005317677,-6.768980994246642)
goal = (-1.2362558848795064,-6.926854457315117)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.2362558848795064,-6.926854457315117)
goal = (-1.1466190512563594,-6.453107034585642)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.1466190512563594,-6.453107034585642)
goal = (-0.8712650131255053,-6.046370346467935)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.8712650131255053,-6.046370346467935)
goal = (-2.142714368378508,-6.068648686534862)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.142714368378508,-6.068648686534862)
goal = (0.5722427489180033,-5.760619054671951)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.5722427489180033,-5.760619054671951)
goal = (-2.1618668199978233,-3.3443035448960297)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.1618668199978233,-3.3443035448960297)
goal = (-3.668974321647343,-0.6349623856195095)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (-1.2362558848795064,-6.926854457315117)
goal = (-1.6317545099234838,-7.063469331803776)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.6317545099234838,-7.063469331803776)
goal = (-1.859372464107178,-6.209967366331641)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.859372464107178,-6.209967366331641)
goal = (0.21549724880487542,-6.475700475849825)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.21549724880487542,-6.475700475849825)
goal = (0.7754994969194522,-5.423251874995392)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.7754994969194522,-5.423251874995392)
goal = (-3.1470211727594153,-3.580045952182412)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.1470211727594153,-3.580045952182412)
goal = (-3.3750691391550793,-0.2907736440699207)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (-0.8165469323486505,-7.521330231277359)
goal = (-1.5369271443593782,-7.25412295073071)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.5369271443593782,-7.25412295073071)
goal = (0.36561251468930944,-7.060810971893714)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.36561251468930944,-7.060810971893714)
goal = (0.6816639012920982,-7.405635152365205)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.6816639012920982,-7.405635152365205)
goal = (2.0031152975978745,-6.985600874072653)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.0031152975978745,-6.985600874072653)
goal = (3.540800000440747,-6.429252610674144)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.540800000440747,-6.429252610674144)
goal = (2.4459594876977526,-0.7179750098569935)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (-1.1466190512563594,-6.453107034585642)
goal = (-1.9255693869959778,-6.268111939466629)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.9255693869959778,-6.268111939466629)
goal = (0.08426000681291246,-6.069762374775077)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.08426000681291246,-6.069762374775077)
goal = (0.62297614266034,-4.834819508052645)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.62297614266034,-4.834819508052645)
goal = (0.2914603018468318,-3.1425710082964295)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.2914603018468318,-3.1425710082964295)
goal = (-3.417503129174072,-0.1763907252489263)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (-1.6317545099234838,-7.063469331803776)
goal = (-2.331211817959585,-7.173049683482322)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.331211817959585,-7.173049683482322)
goal = (-2.523816710029136,-5.995523693509273)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.523816710029136,-5.995523693509273)
goal = (-4.54777891759484,-7.133114784801966)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.54777891759484,-7.133114784801966)
goal = (-2.5107423748329283,-3.0695360763510475)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.5107423748329283,-3.0695360763510475)
goal = (-5.253545808733608,-1.0521478702208373)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (-1.5369271443593782,-7.25412295073071)
goal = (-2.4206284875068182,-6.4331098547024)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.4206284875068182,-6.4331098547024)
goal = (-2.8451634413220637,-6.113723785866034)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.8451634413220637,-6.113723785866034)
goal = (-4.680997185872109,-7.378265145125503)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.680997185872109,-7.378265145125503)
goal = (-5.45737036871612,-5.476637834061128)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.45737036871612,-5.476637834061128)
goal = (-5.538701869834626,-0.8865318416739152)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (0.022564139302638786,-7.1532746075298785)
goal = (0.4649898603131568,-7.317820026526276)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.4649898603131568,-7.317820026526276)
goal = (0.8736843135339019,-7.575964858203205)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.8736843135339019,-7.575964858203205)
goal = (1.4813740268524667,-5.767111832203025)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.4813740268524667,-5.767111832203025)
goal = (3.6281053566604413,-6.6145938011846255)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.6281053566604413,-6.6145938011846255)
goal = (3.4271592990743027,-1.163131848400396)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (-0.8712650131255053,-6.046370346467935)
goal = (-0.6857164368794457,-4.945946853545038)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.6857164368794457,-4.945946853545038)
goal = (0.5151303352668561,-4.639215628783838)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.5151303352668561,-4.639215628783838)
goal = (-0.47720147861629947,-2.738404490690474)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.47720147861629947,-2.738404490690474)
goal = (2.209186676830111,-0.603115974141506)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (-1.859372464107178,-6.209967366331641)
goal = (-2.69639833538845,-5.833318362422521)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.69639833538845,-5.833318362422521)
goal = (-4.024706872659191,-5.140405479811036)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.024706872659191,-5.140405479811036)
goal = (-5.014701751516594,-4.7060009238847105)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.014701751516594,-4.7060009238847105)
goal = (-2.268691616163257,0.33696674631793666)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (0.36561251468930944,-7.060810971893714)
goal = (0.8136880969242499,-6.286262538861715)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.8136880969242499,-6.286262538861715)
goal = (1.6891583631486284,-5.9404064389974245)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.6891583631486284,-5.9404064389974245)
goal = (3.351494380653442,-5.257330862367969)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.351494380653442,-5.257330862367969)
goal = (2.7626014303803466,-0.7779296903447692)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (-1.9255693869959778,-6.268111939466629)
goal = (-3.130989889788276,-6.190236829753678)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.130989889788276,-6.190236829753678)
goal = (-4.45629169517109,-5.2403268993777115)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.45629169517109,-5.2403268993777115)
goal = (-4.0271967467300485,-3.4235453876691233)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.0271967467300485,-3.4235453876691233)
goal = (-5.545016690649609,-0.4145813434804033)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (-2.331211817959585,-7.173049683482322)
goal = (-3.118107048934916,-5.945174241646068)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.118107048934916,-5.945174241646068)
goal = (-4.806638885822611,-7.698769117758298)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.806638885822611,-7.698769117758298)
goal = (-6.075874409380002,-5.998277216937728)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.075874409380002,-5.998277216937728)
goal = (-5.289088371096185,-0.20727874023301407)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (-2.4206284875068182,-6.4331098547024)
goal = (-3.6407447048321675,-5.064149581318752)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.6407447048321675,-5.064149581318752)
goal = (-4.492430920533476,-5.3209451198919275)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.492430920533476,-5.3209451198919275)
goal = (-2.216108919391343,-2.6752768997858754)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.216108919391343,-2.6752768997858754)
goal = (-5.3840086123127175,-0.03606683567951663)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (0.4649898603131568,-7.317820026526276)
goal = (1.4835063061839318,-6.9411247616919605)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.4835063061839318,-6.9411247616919605)
goal = (2.1929324874937652,-7.680300787217279)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.1929324874937652,-7.680300787217279)
goal = (3.8668556598742203,-6.543769955221952)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.8668556598742203,-6.543769955221952)
goal = (2.9508841846347202,-0.5500295990154447)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (-0.18055228027303016,-6.055357926857768)
goal = (0.39537645185327985,-5.7000147374763745)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.39537645185327985,-5.7000147374763745)
goal = (1.0016790042523915,-5.0299028488064845)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.0016790042523915,-5.0299028488064845)
goal = (3.0791555047401236,-4.050849967933444)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.0791555047401236,-4.050849967933444)
goal = (0.2578738003975056,0.4256159805388888)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (-2.142714368378508,-6.068648686534862)
goal = (-3.176720424526848,-3.7672097412194603)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.176720424526848,-3.7672097412194603)
goal = (-3.379530842971472,-2.7074751252863454)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.379530842971472,-2.7074751252863454)
goal = (-2.313651577544367,0.7511768544870661)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (0.21549724880487542,-6.475700475849825)
goal = (0.895000903495272,-4.865630452838244)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.895000903495272,-4.865630452838244)
goal = (3.5725761225735333,-4.970055756819315)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.5725761225735333,-4.970055756819315)
goal = (4.472804094465255,-1.3221160369682217)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (0.6816639012920982,-7.405635152365205)
goal = (2.1220005318297,-5.873487854655098)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.1220005318297,-5.873487854655098)
goal = (4.163783831230488,-7.356600822922366)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.163783831230488,-7.356600822922366)
goal = (4.437161340985405,-1.098200695538524)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (0.08426000681291246,-6.069762374775077)
goal = (0.569246214465907,-4.497245476316399)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.569246214465907,-4.497245476316399)
goal = (3.2057581464078826,-4.2530152423486705)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.2057581464078826,-4.2530152423486705)
goal = (4.951800018068358,-1.383715913706471)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (-2.523816710029136,-5.995523693509273)
goal = (-4.368720871514671,-4.59655022510008)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.368720871514671,-4.59655022510008)
goal = (-2.6891087317394042,-2.431814089436698)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.6891087317394042,-2.431814089436698)
goal = (-5.72862841485295,-0.2004757014833256)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (-2.8451634413220637,-6.113723785866034)
goal = (-4.811954198599629,-5.1082709329863)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.811954198599629,-5.1082709329863)
goal = (-5.428629848080151,-3.836823280186713)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.428629848080151,-3.836823280186713)
goal = (-5.4444001258780546,0.1574847746568908)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (0.8736843135339019,-7.575964858203205)
goal = (2.0679055736983027,-5.742672403985558)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.0679055736983027,-5.742672403985558)
goal = (3.9666334554867007,-6.05540868984209)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.9666334554867007,-6.05540868984209)
goal = (3.4099083872301694,-0.031660968496168174)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (-0.6857164368794457,-4.945946853545038)
goal = (-0.3120168861325432,-3.8195940425608748)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.3120168861325432,-3.8195940425608748)
goal = (0.25850466173794917,-2.1039158280441193)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.25850466173794917,-2.1039158280441193)
goal = (1.407212953052631,0.6627243237802292)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (-2.69639833538845,-5.833318362422521)
goal = (-4.121668154219819,-4.3061758834185735)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.121668154219819,-4.3061758834185735)
goal = (-5.515086100234171,-3.869414490546955)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.515086100234171,-3.869414490546955)
goal = (-2.7432350458903927,1.0795912452993557)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (0.8136880969242499,-6.286262538861715)
goal = (1.6124886001270653,-4.995335573725371)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.6124886001270653,-4.995335573725371)
goal = (4.091721393129161,-6.157626186531291)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.091721393129161,-6.157626186531291)
goal = (3.3335390705856494,0.006129601646764904)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (-3.130989889788276,-6.190236829753678)
goal = (-4.922456055858952,-7.51414329586567)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.922456055858952,-7.51414329586567)
goal = (-5.957229678522643,-4.1800028595654215)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.957229678522643,-4.1800028595654215)
goal = (-2.679933783619046,2.3294628980058123)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (-3.118107048934916,-5.945174241646068)
goal = (-3.791032982438224,-3.815462565662975)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.791032982438224,-3.815462565662975)
goal = (-4.4109268781855695,-2.5252333206212)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.4109268781855695,-2.5252333206212)
goal = (-5.726951255253529,1.9923130340635007)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (-3.6407447048321675,-5.064149581318752)
goal = (-4.599095914181085,-4.124609891248458)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.599095914181085,-4.124609891248458)
goal = (-5.0526468441717265,-2.6485918247787534)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.0526468441717265,-2.6485918247787534)
goal = (-6.12591595847529,1.904814235238116)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (1.4835063061839318,-6.9411247616919605)
goal = (3.0208095206629277,-5.8667305118370265)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.0208095206629277,-5.8667305118370265)
goal = (4.412190189579734,-6.027885874832855)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.412190189579734,-6.027885874832855)
goal = (4.969367706691637,-0.42032014766736836)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (0.39537645185327985,-5.7000147374763745)
goal = (1.1839876101144142,-4.44237349198064)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.1839876101144142,-4.44237349198064)
goal = (3.2901632986712066,-3.922723642209359)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.2901632986712066,-3.922723642209359)
goal = (1.784595729162362,0.8774934124508063)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (0.6582656196446202,-6.103986250237439)
goal = (0.15178521084914376,-3.396456042088797)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.15178521084914376,-3.396456042088797)
goal = (3.3211557584458156,-3.8933139040770692)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.3211557584458156,-3.8933139040770692)
goal = (2.058624238151223,0.9307061387487705)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (0.5722427489180033,-5.760619054671951)
goal = (3.4327159845740844,-3.932536185883671)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.4327159845740844,-3.932536185883671)
goal = (4.6583428290915645,-0.11853142980656273)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (0.7754994969194522,-5.423251874995392)
goal = (3.3114691560966953,-3.6674669892650567)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.3114691560966953,-3.6674669892650567)
goal = (2.960850626308808,0.8622056012963997)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (2.0031152975978745,-6.985600874072653)
goal = (4.3175375437521115,-5.544718497136639)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.3175375437521115,-5.544718497136639)
goal = (4.590095867592476,0.2177314517578699)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (0.62297614266034,-4.834819508052645)
goal = (0.6969876815367035,-1.8885174602325003)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.6969876815367035,-1.8885174602325003)
goal = (1.7448689844157546,1.9362863571050317)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (-4.54777891759484,-7.133114784801966)
goal = (-5.558605850903534,-2.6990360398296573)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.558605850903534,-2.6990360398296573)
goal = (-5.153965527149372,2.2233376184820335)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (-4.680997185872109,-7.378265145125503)
goal = (-5.852642701545564,-2.4507755921821843)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.852642701545564,-2.4507755921821843)
goal = (-5.616307189230472,2.2575942659046246)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (1.4813740268524667,-5.767111832203025)
goal = (3.8158436735665546,-4.25127623034405)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.8158436735665546,-4.25127623034405)
goal = (3.5670585361020546,1.7286455111256531)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (0.5151303352668561,-4.639215628783838)
goal = (3.7640543447331876,-3.7928640401359623)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.7640543447331876,-3.7928640401359623)
goal = (-0.36748394576004895,1.969005620774344)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (-4.024706872659191,-5.140405479811036)
goal = (-4.911230102494121,-2.318522223734342)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.911230102494121,-2.318522223734342)
goal = (-4.6103776827051,2.482729868549426)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (1.6891583631486284,-5.9404064389974245)
goal = (4.110785954436645,-3.797027483901421)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.110785954436645,-3.797027483901421)
goal = (1.6911815062717732,2.2041089540196674)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (-4.45629169517109,-5.2403268993777115)
goal = (-4.955217935905756,-2.280036503246131)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.955217935905756,-2.280036503246131)
goal = (-4.417194826084241,2.777639265966857)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (-4.806638885822611,-7.698769117758298)
goal = (-3.2916561170136953,-2.2507153123662436)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.2916561170136953,-2.2507153123662436)
goal = (-6.266106172990206,2.726901118711976)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (-4.492430920533476,-5.3209451198919275)
goal = (-3.5798712691246113,-2.159510305042742)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.5798712691246113,-2.159510305042742)
goal = (-2.998168129409699,2.6918834269885794)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (2.1929324874937652,-7.680300787217279)
goal = (4.961717111843749,-6.1020131818036765)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.961717111843749,-6.1020131818036765)
goal = (2.5331703661268055,2.2114385183881184)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (1.0016790042523915,-5.0299028488064845)
goal = (0.9291111538451471,-1.2016922128226755)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.9291111538451471,-1.2016922128226755)
goal = (-0.7712435985149426,2.2137882089104535)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (-3.176720424526848,-3.7672097412194603)
goal = (-2.79128770535563,-2.26037822798574)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.79128770535563,-2.26037822798574)
goal = (-0.7515518479949392,2.3235402902713487)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (0.895000903495272,-4.865630452838244)
goal = (2.0969662340676294,-1.2834343550157241)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.0969662340676294,-1.2834343550157241)
goal = (-0.03789675292556449,2.3549665056945956)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (2.1220005318297,-5.873487854655098)
goal = (4.993017573141971,-3.3935907167336365)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.993017573141971,-3.3935907167336365)
goal = (2.1998622752418875,2.53767168182251)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (0.569246214465907,-4.497245476316399)
goal = (0.5844048511167816,-1.077704635731287)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.5844048511167816,-1.077704635731287)
goal = (0.04369552203463911,2.4433799672060763)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (-4.368720871514671,-4.59655022510008)
goal = (-2.8208769156580606,-2.165872177712484)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.8208769156580606,-2.165872177712484)
goal = (-5.739241789731854,2.837253571112151)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (-4.811954198599629,-5.1082709329863)
goal = (-4.86658794051971,-1.6279602359778043)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.86658794051971,-1.6279602359778043)
goal = (-4.729793482067191,3.30293716828204)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (2.0679055736983027,-5.742672403985558)
goal = (4.697817369895661,-2.9420053104551567)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.697817369895661,-2.9420053104551567)
goal = (1.8460875885598007,2.744859735359433)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (-0.3120168861325432,-3.8195940425608748)
goal = (-0.49254675502392953,-1.1557718736753886)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.49254675502392953,-1.1557718736753886)
goal = (-0.7903065209870004,2.6677234799920564)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (-4.121668154219819,-4.3061758834185735)
goal = (-2.668388821279307,-1.8923839515381493)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.668388821279307,-1.8923839515381493)
goal = (-2.2004314330540007,3.1972363445170187)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (1.6124886001270653,-4.995335573725371)
goal = (2.5722018489078806,-1.1985714886016874)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.5722018489078806,-1.1985714886016874)
goal = (-0.8081058045165106,3.124791181441119)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (-4.922456055858952,-7.51414329586567)
goal = (-6.302070929147672,-1.9086477115684657)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.302070929147672,-1.9086477115684657)
goal = (-6.272426703386303,3.295107427746875)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (-3.791032982438224,-3.815462565662975)
goal = (-4.669052388829748,-1.3042790186965831)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.669052388829748,-1.3042790186965831)
goal = (-4.191243470090621,3.4518961735112894)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (-4.599095914181085,-4.124609891248458)
goal = (-6.231378935382043,-1.513171694954556)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.231378935382043,-1.513171694954556)
goal = (-4.679554404800953,3.4479564989630545)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (3.0208095206629277,-5.8667305118370265)
goal = (4.826116942018398,-2.715497536530047)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.826116942018398,-2.715497536530047)
goal = (4.968758123670193,3.0547628224278123)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (1.1839876101144142,-4.44237349198064)
goal = (3.5904553414751366,-1.5291346654740012)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.5904553414751366,-1.5291346654740012)
goal = (0.8331607172342421,3.497874532196179)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (0.15178521084914376,-3.396456042088797)
goal = (-1.7434241976948188,-1.2634153607229228)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.7434241976948188,-1.2634153607229228)
goal = (-0.44662977682254734,3.391685244952721)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (-0.020614520550903848,-3.258097169967175)
goal = (-2.0501197655666408,-0.9761800631776865)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.0501197655666408,-0.9761800631776865)
goal = (0.11565003342066582,3.4337731212830986)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (-2.1618668199978233,-3.3443035448960297)
goal = (-3.3242785816398297,3.6961128790796467)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (-3.1470211727594153,-3.580045952182412)
goal = (-4.463246983588093,3.515575590332353)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (3.540800000440747,-6.429252610674144)
goal = (4.62763310384566,3.23558904132135)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (0.2914603018468318,-3.1425710082964295)
goal = (-0.3033860067716203,3.683921783693127)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (-2.5107423748329283,-3.0695360763510475)
goal = (-3.9351680945661167,3.5998255758157667)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (-5.45737036871612,-5.476637834061128)
goal = (-5.055601705614307,3.9496103933245115)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (3.6281053566604413,-6.6145938011846255)
goal = (4.576260348839304,3.9267345786651235)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (-0.47720147861629947,-2.738404490690474)
goal = (0.5164488388790041,3.7363642968184516)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (-5.014701751516594,-4.7060009238847105)
goal = (-3.5762156180098645,3.842384465413036)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (3.351494380653442,-5.257330862367969)
goal = (4.9141036939062435,4.103598002233276)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (-4.0271967467300485,-3.4235453876691233)
goal = (-5.214436876315749,4.042118774362213)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (-6.075874409380002,-5.998277216937728)
goal = (-6.363209426518332,4.256180935819004)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (-2.216108919391343,-2.6752768997858754)
goal = (-1.7915275659448682,3.870507416508299)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (3.8668556598742203,-6.543769955221952)
goal = (4.943328761334982,4.703475748269388)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (3.0791555047401236,-4.050849967933444)
goal = (2.0593612279008386,4.844357728135315)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (-3.379530842971472,-2.7074751252863454)
goal = (-3.6596803891088445,4.255847176956467)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (3.5725761225735333,-4.970055756819315)
goal = (4.182524086259276,4.998472694052148)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (4.163783831230488,-7.356600822922366)
goal = (2.1606764137133823,4.975756959373425)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (3.2057581464078826,-4.2530152423486705)
goal = (1.017629165146805,5.22952411172541)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (-2.6891087317394042,-2.431814089436698)
goal = (-3.14507593554804,4.319896280122685)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (-5.428629848080151,-3.836823280186713)
goal = (-4.566011802022212,4.343002364781049)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (3.9666334554867007,-6.05540868984209)
goal = (2.606105951819014,5.581622269108107)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (0.25850466173794917,-2.1039158280441193)
goal = (-2.25737598770666,4.83285074516255)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (-5.515086100234171,-3.869414490546955)
goal = (-4.6061104195519,4.374917892705375)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (4.091721393129161,-6.157626186531291)
goal = (2.5624308167519683,5.691962360591858)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (-5.957229678522643,-4.1800028595654215)
goal = (-5.503733260271534,4.72734744164038)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (-4.4109268781855695,-2.5252333206212)
goal = (-4.416862733231763,4.741762423687869)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (-5.0526468441717265,-2.6485918247787534)
goal = (-4.520872354871218,4.822399207306526)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (4.412190189579734,-6.027885874832855)
goal = (2.220237220314381,5.695364116699532)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (3.2901632986712066,-3.922723642209359)
goal = (2.169700349107515,5.804726406630048)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (3.3211557584458156,-3.8933139040770692)
goal = (0.8448490082504332,5.688552677999923)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (3.4327159845740844,-3.932536185883671)
goal = (3.322771041190789,6.109362564012314)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (3.3114691560966953,-3.6674669892650567)
goal = (0.12924733374246866,5.578274584765819)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (4.3175375437521115,-5.544718497136639)
goal = (4.5183418970618,6.0794012205332235)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (0.6969876815367035,-1.8885174602325003)
goal = (-2.5642218976672764,4.900211592260876)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (-5.558605850903534,-2.6990360398296573)
goal = (-4.909246190698175,5.009226016791049)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (-5.852642701545564,-2.4507755921821843)
goal = (-5.6880133509567194,5.313815793122955)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (3.8158436735665546,-4.25127623034405)
goal = (3.717813541582789,6.202398063836356)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (3.7640543447331876,-3.7928640401359623)
goal = (0.5062394948131264,5.834619833558642)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (-4.911230102494121,-2.318522223734342)
goal = (-4.786330918648644,5.262471692361099)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (4.110785954436645,-3.797027483901421)
goal = (2.4302790998566213,6.520418284720473)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-4.955217935905756,-2.280036503246131)
goal = (-3.969946841946675,5.40763925456445)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (-3.2916561170136953,-2.2507153123662436)
goal = (-2.8439100321690876,5.73511777258554)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-3.5798712691246113,-2.159510305042742)
goal = (-3.185483470875211,5.803050669933673)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (4.961717111843749,-6.1020131818036765)
goal = (3.767016073272435,6.796084116209374)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (0.9291111538451471,-1.2016922128226755)
goal = (-1.6628528752495866,5.941146135191841)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (-2.79128770535563,-2.26037822798574)
goal = (-4.831996886820292,5.794178121995345)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (2.0969662340676294,-1.2834343550157241)
goal = (0.42612993971664004,6.478556486320968)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (4.993017573141971,-3.3935907167336365)
goal = (-2.525278785378516,6.283380825029728)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (0.5844048511167816,-1.077704635731287)
goal = (-2.9662853794898316,6.169537264478197)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (-2.8208769156580606,-2.165872177712484)
goal = (-3.7020364451549277,6.10823258538644)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (-4.86658794051971,-1.6279602359778043)
goal = (-5.619172190603545,5.871398747316603)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (4.697817369895661,-2.9420053104551567)
goal = (-3.0299087438918106,6.279626907662183)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (-0.49254675502392953,-1.1557718736753886)
goal = (-3.113952630728397,6.526185151339617)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (-2.668388821279307,-1.8923839515381493)
goal = (-3.0839901018645146,6.5398146249559845)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (2.5722018489078806,-1.1985714886016874)
goal = (-4.034564611397277,6.570268489884332)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-6.302070929147672,-1.9086477115684657)
goal = (-5.971093415291587,6.2172255155039755)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (-4.669052388829748,-1.3042790186965831)
goal = (-5.220413302806468,6.633165489646806)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (-6.231378935382043,-1.513171694954556)
goal = (-4.259170085625758,6.799642926338419)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
content = content[:-1]
f = open('smo2sol-18.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
