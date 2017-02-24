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
import numpy

import random
import math
import copy

global global_unawakenRobots
global global_path
global global_robots_queue

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

def samLines(coords):
    for lines in coords:
        #print lines
        prex = "f"
        prey = "f"
        randColor = numpy.random.rand(3,1)
        for (x,y) in lines:
            #print (x,y)
            if prex == "f":
                prex = x
                prey = y
            else:
                line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None,edgecolor = randColor)
                plt.gca().add_patch(line)
                prex = x
                prey = y
                #print("connecting",(x,y),"to",(prex,prey))

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
            #print (ips,type(ips))
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

    def Planning(self,animation=False):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """
        robots = self.robots
        minDis = 100000
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
            for (gx,gy) in self.robots:
                #Find closest robot
                dx = newNode.x - gx
                dy = newNode.y - gy
                d = math.sqrt(dx * dx + dy * dy)
            #                print (gx,gy)
            #                print d
                if d < minDis:
                    #print "MIN"
                    #print d
                    minDis = d
                    self.end = Node(gx,gy)
                    #print self.end.x,self.end.y

# check goal
                gx = self.end.x
                gy = self.end.y
                dx = newNode.x - gx
                dy = newNode.y - gy
                d = math.sqrt(dx * dx + dy * dy)
                if d <= self.expandDis:
                    if not self.__CollisionCheck(newNode, obstacleList,Node(gx,gy)): #if intersects
                        continue
                    else:
                        #print "ok"
                        #print (gx,gy)
                        path += [[gx,gy]]
                        foundNode.append((gx,gy))
                        check = True
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



class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0,item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)
    def printItems(self):
        for items in self.items:
            print items


def awakeRobots():
    global global_robots_queue
    while global_robots_queue.isEmpty() == False: ##while queue is not empty
        global global_unawakenRobots
        #print global_robots_queue.printItems()
        print ("robots left:",len(global_unawakenRobots))
        nextRobot = global_robots_queue.dequeue()
        currentNode = nextRobot['startCoord']
        previousPath = nextRobot['previousPath']
        numRobotsAtNode = nextRobot['numRobotsAtNode']
        rand = nextRobot['rand']
        rrtshortestpath(currentNode,obstacleList,previousPath,rand,numRobotsAtNode)

    #print "complete awakening all robots"
    return




#=====end of RRT class

def rrtpath(obstacles,startcoord,goalcoord,randAreas):
    rrt = RRT(start=startcoord, goal=goalcoord,randArea = randAreas, obstacleList=obstacles)
    answer = rrt.Planning(animation=False)
    path= answer['path']
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-r')
    smoothiePath.reverse()
    #print smoothiePath
    return {'path': smoothiePath, 'node': answer['node']}


def rrtshortestpath(currentNode,obstacleList,previousPath,rand,numAct):
    global global_unawakenRobots
    global global_path
    unawakenRobots = global_unawakenRobots
    #print ("left",unawakenRobots)
    #print len(unawakenRobots)
    if len(unawakenRobots) == 0:
        #print "0 left"
        #print previousPath
        global_path.append(previousPath)
        return
    if ((numAct == 2) & (len(unawakenRobots) == 1)):
        #print "final"
        a= rrtpath(obstacleList,currentNode[0],unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        newPath = previousPath + a['path']
        #print newPath
        global_path.append(newPath)
        return
    if numAct == 1:
        #print "num = 1"
        a = rrtpath(obstacleList,currentNode,unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        #print a['path']
        newPath = previousPath + a['path']
        #print newPath
        #print "finding next node"
        nextLoop = {'startCoord': foundNode[0], 'obstacleList': obstacleList, 'previousPath': newPath, 'rand': rand, 'numRobotsAtNode': 2}
        global_robots_queue.enqueue(nextLoop)
#        rrtshortestpath(foundNode[0],obstacleList,newPath,rand,2)
        return
    elif numAct == 2:
        #print "num = 2"
        if(type(currentNode)== tuple):
            fixedTypeCurrentNode = currentNode
            #print "tuple"
        else:
            #print "list"
            fixedTypeCurrentNode = currentNode[0]
        #print (currentNode,unawakenRobots[-1])
        a = rrtpath(obstacleList,fixedTypeCurrentNode,unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        newPath = previousPath + a['path']
        nextLoop = {'startCoord': foundNode, 'obstacleList': obstacleList, 'previousPath': newPath, 'rand': rand, 'numRobotsAtNode': 2}
        #print "robot 1"
        #print newPath
        #print nextLoop
        global_robots_queue.enqueue(nextLoop)
        unawakenRobots = global_unawakenRobots
        if len(unawakenRobots) == 0:
            return
#        rrtshortestpath(foundNode,obstacleList,newPath,rand,2)
        b = rrtpath(obstacleList,fixedTypeCurrentNode,unawakenRobots[-1],rand)
        foundNode = b['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        newPath = b['path']
        nextLoop = {'startCoord': foundNode, 'obstacleList': obstacleList, 'previousPath': newPath, 'rand': rand, 'numRobotsAtNode': 2}
        #print "robot 2"
        #print newPath
        #print nextLoop

        global_robots_queue.enqueue(nextLoop)
#        rrtshortestpath(foundNode,obstacleList,newPath,rand,2)
        return

    return




robots = [(2.2073865752517436,-1.889047496129887),(7.418773235738816,-0.44990765706479596),(4.973369588410079,7.5255867963970475),(-6.343762246568933,6.482770565702903),(7.638112697692372,7.546541304735052),(-6.616019419392999,0.6818317478052967),(7.566930030403939,8.35536654882807),(1.962511182428952,3.827257744484897),(0.26424382924462364,4.408587560180833),(-1.6363879702350923,4.2742016914298455),(0.7514081303472473,3.0733529751116455),(0.5267146642115579,-0.4931108621138689),(4.659784708236221,3.526482298351895),(7.758024410082515,-3.9166527369821686),(-3.768419738963783,5.061463126225703),(-1.238311811529301,0.7476069382605006),(4.667157304771778,-2.534219407619858),(-3.276919255050708,-2.3398760145109065),(4.027309841872726,2.225932179379771),(-3.134941746562891,4.392880537243911),(-1.7919488323765025,-0.9304890692153593),(-2.1055915905313096,7.17999624927331),(3.996028251723197,-2.2093291846560725),(-3.5164396704835954,6.373383887540029),(0.2854132295569878,5.243555165301994),(-3.8098482367621416,0.7147790107512577),(4.154214433161562,1.2618722266885047),(1.7474003547927728,-1.2477990406749173),(0.8212165138124101,7.303175605155082),(4.302652030414358,4.117179861754971),(-5.4248143794161745,8.136563438916923),(-3.3165405488513895,6.68797610264568),(-3.030985979887064,-5.606359093806035),(1.7286467154371232,-3.3592566534232917),(-6.453060574822895,-5.554029587082576),(7.548033057492154,-3.6395894800134774),(1.3847553205953984,-4.943156251673194),(7.54428351186684,-5.122805239294733),(6.069523460454115,-4.654963174682476),(-3.7547046958434,-5.922862243521676),(8.07226580148582,-3.7043831068229327),(3.268322380456488,6.87301494816695),(-0.23603546301427958,-1.6753606362682891),(5.7410221165283835,-5.033349148339862),(-3.572734367362853,7.552213061823806),(4.81906153781483,1.9800634058196565),(4.715317354682997,-3.8923377625585642),(4.748705754540919,-3.4749419274934694),(5.5460592904260055,6.6983154240782),(-5.1248967023390675,4.5660839994153575),(-5.392545551804345,4.952462262791909),(-2.3217308730612407,-4.312281956042135),(7.1765781232461565,8.292057771681467),(3.777882182182938,1.5017283946377313),(0.5183545338567797,6.284145566423535),(6.092954859853264,-2.775568375975115),(0.5311369877823493,8.12312296540589),(-4.984831097840179,-1.316710012155605),(-4.044553473757543,5.295550268280642),(-1.8287418950606478,-2.0036206511860257),(4.689812573707262,1.118966583506718),(-6.30809112233693,-3.423770818145659),(-4.716591528644418,-0.535560816076492),(4.482578581463751,1.5283944740509536),(0.4455311690117645,6.18488677664192),(6.4849370898978025,-2.1915511517535906),(-4.913418124436547,8.081318894398798),(5.022656397893212,-2.7608289720228574),(1.8798535943447847,-0.6383019674365489),(-4.479245268156541,5.078256011534479),(6.344455275249538,1.329287354089785),(6.57261450653339,-5.301205210090737),(7.63100896637573,1.738124725253063),(6.382889837024428,4.80635547189856),(1.7730736494644317,-2.741939970082505),(2.5744559952202186,0.30488052494965334),(6.427969280461144,-3.3622836044372106),(-6.057352547804057,-4.579248928098282),(-1.3002157590205492,6.368914071163556),(-1.287303589354793,1.1882800807818956),(-1.788658351617726,-0.12416042706065422),(4.181030805569011,3.8092654021697934),(0.5987523399188897,5.260960668076128),(3.6562263543941365,-0.5658475093721069),(5.693423870458452,0.4164082744067814),(-4.321501660402658,-4.456941382383687),(-4.612247153395806,3.829994051433551),(0.30136669473326805,-0.2156774475360601),(7.538395029430419,6.086625229536261),(2.521044193397004,2.7833014013462796),(-3.205820119702892,8.036067317096311),(4.6964176150134245,-5.078331659520719),(-1.6280542910310398,7.990489676732845),(-2.6106312086221592,-4.490064377416797),(6.170146848608327,2.6317971255677906),(7.264489879066921,-1.7977843984976092),(3.6808085503323236,-1.4245323836814894),(-4.918965105549663,8.29552691059055),(-4.963389653872008,-4.542198610532715),(-1.8471622418386797,-2.620020574934525),(-6.158494054910672,4.832306410269326),(3.583623786998344,8.228503826577253),(5.856255988579135,-3.3541159123469337),(3.4250893144721433,7.447867490632302),(0.6423588364572801,-2.472384061943537),(-4.1577442406987295,6.6662877272485455),(-3.852268794082571,4.5634314155888385),(2.0452830203720405,7.186721143020939),(3.250059279540161,6.753195379630367),(4.88257377272856,0.3825841113610746),(-5.457943264046682,-1.733974330494143),(-0.23350260020815483,6.811155081017756),(-6.70295522422186,-4.945460151408957),(-4.783391721935423,3.857389962835348),(-5.157610615246372,-2.277416451686563),(0.5990437760730734,4.196517832316757),(-5.271233321976013,7.258607493577954),(-3.720812228177616,-4.008457892597985),(4.666101135392579,-3.8678865237426545),(3.4244559446025162,1.6371673455419966),(7.6506160594950545,-4.779703964356228),(1.4229928081412178,4.2364709505478215),(3.181575868501068,4.040065465836787),(1.9249637997589817,5.686935126665827),(-1.9749657545824375,0.326640873787011),(5.199848659454887,7.798093200242936),(4.834143842795577,2.718471268911519),(2.0081887795594477,-0.6073054412326799),(2.1862617965660016,-5.0823412265754335),(-1.2114554132257362,5.656851204119694),(-1.1193292419264615,-0.8911270167957932),(-5.785627369121695,5.7760457258746625),(5.75854238068434,-1.8348631409702518),(-6.237067067904015,7.272881247133454),(-4.466869370955683,5.402656452545206),(6.500795366272993,1.7985609780485925),(-0.013224632648134893,5.624906533035998),(5.713519862200783,0.0371583235365609),(1.4393245345179375,-3.8847578742816524),(7.816150056355288,-3.222837298213615),(4.639322038735824,7.180157818416753),(5.8158669552286275,0.740779373737146),(7.359462888550909,6.147223823325191),(4.120537903753147,7.732517123456904),(-1.7807428254991935,7.519039005570991),(0.7465081534747418,5.6101876625716285),(0.9838156241238671,-2.770041751461609),(-6.0653733562613965,6.769937731630743),(-1.4349215484028237,3.8455290371136064),(-2.5786651848935787,5.189917289530594),(-3.6402659875882186,1.5738600985019806),(6.202416057614024,8.467895789094996),(-3.2483339919722307,5.8841156676234645),(5.802779991995073,-5.932824013422136),(-4.133657146733643,-1.8549606621594004),(-5.444545296305635,7.569995420942168),(2.5448715227696512,-1.0369343760909198),(3.6810534482179387,-3.8394939980312888),(8.102502422790518,-0.6675134448085274),(4.373665577804144,8.402818394485003),(-0.7094573941126807,6.887803345371094),(0.4651044887588478,7.298428603670792),(5.560719660822557,-4.982407345591296),(5.394099386231296,-1.6388855875544195),(7.5298057430505105,-1.9892292663300681),(6.892855272674088,-0.5937947418630616),(-6.154590237386031,-2.6310167838522003),(2.691340569100814,-5.818584948025541),(7.117457396849843,-0.22083786570504493),(7.457977218246763,-1.0298937969363626),(7.84815943007754,-0.565157557641359),(-1.8667196752569009,0.0939600881616478),(1.0590787635810806,3.0536227066617885),(3.6587627404941427,-5.768358805694836),(-2.10145664290307,4.634746937350806),(5.583603367661769,2.521712903685027),(1.4072099810963001,7.520163010483652),(-2.971869498676368,7.729122941168884),(-6.0069317705424945,-4.863131398177553),(-6.351229972392486,7.9830178399070055),(-5.563016904846581,-5.81299682621115),(0.827385242329937,-3.154107603214514),(-6.454074575246995,-1.412478324306166),(3.3014635550083833,-5.215244328698136),(3.7960877902297367,-2.15815013489891),(7.011374681645717,-1.1012226161701335),(-4.738958255779945,-2.546487163278671),(-5.486809608364741,4.169259889991571),(-5.443956747210483,-1.8446484911258532),(-1.8165506695596862,6.00695261255007),(-6.022637515411948,-3.3688760748678646),(-4.379908959188498,6.783101228645848),(-2.5401817105894935,4.9443233919488305),(5.887459685678761,-3.8274345436115307),(2.4899979525665943,3.962096580823869),(-1.104062437366955,1.0565026766515393),(8.102932964645166,6.715089326547218),(5.387731315433247,0.3844175532061831),(6.3095116881784845,-3.8757009250238177),(-0.1360746446263681,0.39868626045554567),(-6.423973590069131,4.671708613156295),(-0.5186710851469867,-2.5614658664604586),(5.294959936854654,0.7546653550183651),(2.0669825304151823,-2.3607914760751645),(3.517479038134561,-4.621464338760124),(-5.0298384099569375,3.445242533020502),(6.46284337202088,3.761893483367337),(-1.6951278115382689,5.293052000422565),(-5.799402804024482,6.212834927246061),(1.5557585479348477,-3.623292031988746),(3.3070237365924813,-2.7485282008738645),(4.395076511229729,-3.669401081286677),(-0.48958338397042755,0.060508866467598565),(2.419845595634068,6.804867516083051),(-5.22287120953545,-0.23130559774757842),(-6.4246591482969135,0.5663932732174883),(3.6424833348488166,-2.52891567034766),(-0.7111359405177549,6.103498593429385),(-2.2266208956569526,-4.502957790915962),(-5.876941749936402,-0.6009145815962436),(-4.84790355350174,5.626889726334723),(-6.653788481621504,8.164765214471114),(-1.3447074305066922,-0.19506457753300754),(-0.1656078456355523,2.818503665538999),(1.6431483367636668,-3.177392859717861),(-3.0693554983542364,2.2671675897705965),(2.7101872843437382,-0.502026206187649),(2.7268087606292672,-1.195518154433092),(0.46189235793172845,-0.13062547525178392),(4.233569236084011,-0.1378887119870349),(-1.7478757011512593,-5.705510155138268),(-1.8665782663581032,-0.07015785448803413),(-1.3524631269945777,3.5009702267499145),(-2.2659052202810814,6.089162417087251),(-2.2742993837516465,0.37707441225235083),(0.2110360615927771,-5.8205791832063785),(4.583925243522908,0.7956846800927186),(1.925971563899341,7.6632231145256755),(-0.15198638131848963,-5.302377845270425),(7.682093746471817,2.8165182708423613),(6.9261338601069555,-1.623362689011926),(-1.3224051718684144,1.366424721704818),(2.791061376537047,-5.345422136580185),(-6.5817539335544515,8.12093865213157),(-2.1603219296384557,7.254514065259621),(-4.701926507454818,5.395784134933753),(3.023663721224806,-5.882755948386988),(-0.6676162604919709,-4.648821506198292),(5.916902808895823,-2.9973195793729577),(3.894963083883991,4.419685206209317)]

obstacleList = [[(-0.0246174585342621,0.16891705339831045),(1.9301998869031127,-0.25379933613156713),(2.1415580816680513,0.7236093365871203),(1.1641494089493643,0.9349675313520591),(1.5868657984792418,2.8897848767894345),(0.6094571257605544,3.1011430715543726),(0.39809893099561555,2.123734398835685),(-0.5793097417230719,2.335092593600624),(-0.15659335219319442,4.289909939037998),(-1.1340020249118816,4.501268133802937),(-1.7680766092066984,1.5690421156468752),(0.18674073623067672,1.146325726116998)],[(-0.05901157522490006,-0.09731020795587667),(-1.492534431882603,-1.4919471569285974),(-0.7952159573962381,-2.2087085852574506),(-0.0784545290673882,-1.5113901107710888),(1.3161824199053331,-2.9449129674287895),(2.0329438482341837,-2.247594492942429),(1.335625373747822,-1.5308330646135777),(2.052386802076673,-0.8335145901272166),(0.638306899261461,-0.8140716362847271)],[(-1.8985138332922995,0.07593614482383877),(-3.888733837631598,0.27348192262654597),(-3.987506726532953,-0.7216280795431058),(-2.992396724363303,-0.8204009684444589),(-3.189942502166012,-2.8106209727837586),(-2.194832499996357,-2.9093938616851127),(-2.0960596110950043,-1.9142838595154614),(-1.100949608925354,-2.013056748416814),(-1.9972867221936528,-0.9191738573458109)],[(3.558366313174796,-0.7469747877882957),(4.200548350375849,-2.641071467236391),(5.147596690099898,-2.3199804486358633),(4.826505671499372,-1.3729321089118178),(6.720602350947466,-0.7307500717107629),(6.39951133234694,0.21629826801328667),(5.452462992622889,-0.10479275058724113),(5.1313719740223664,0.8422555891368066),(4.5054146528988435,-0.4258837691877702)],[(-0.1292924782198224,-2.838239291135715),(-1.8695413496807514,-3.8239034658823434),(-1.3767092623074424,-4.6940279016128095),(-0.5065848265769743,-4.201195814239495),(0.47907934816965103,-5.941444685700426),(1.3492037839001179,-5.448612598327115),(0.8563716965268057,-4.5784881625966465),(1.72649613225727,-4.0856560752233335),(0.36353960915349137,-3.70836372686618)],[(0.25492505294213214,6.481748034941264),(0.21323463991774919,8.481313465093988),(-1.786330790234977,8.439623052069605)],[(1.802666880196373,2.3290618407232597),(2.727656395595662,0.5558178088987102),(3.6142784115079385,1.0183125665983537),(3.151783653808293,1.904934582510628),(4.925027685632841,2.8299240979099256),(4.462532927933195,3.7165461138222016),(3.5759109120209187,3.2540513561225515),(3.1134161543212775,4.140673372034827),(2.6892888961086463,2.791556598422905)],[(3.9747552446326915,6.354629051101332),(2.451543302495492,5.0585482745559),(3.0995836907682035,4.296942303487298),(3.861189661836809,4.944982691760014),(5.15727043838224,3.42177074962281),(5.918876409450841,4.0698111378955275),(5.2708360211781216,4.83141710896413),(6.032441992246724,5.47945749723684),(7.328522768792157,3.956245555099641),(8.090128739860756,4.604285943372348),(7.442088351588037,5.365891914440944),(8.203694322656643,6.01393230271367),(6.794047963315328,6.12749788550956),(6.1460075750426135,6.889103856578155),(4.622795632905406,5.593023080032731)],[(-3.775070566373749,2.0380637954647742),(-3.5175923907117252,4.021420795366142),(-4.509270890662411,4.150159883197155),(-4.638009978493424,3.158481383246468),(-6.621366978394792,3.4159595589084892),(-6.7501060662258014,2.4242810589578054),(-5.758427566275118,2.295541971126795),(-5.88716665410613,1.3038634711761112),(-4.766749066324433,2.1668028832957846)],[(-2.8928868598181494,1.3522478460536007),(-4.84903978333255,1.7687400306454113),(-5.265531967924365,-0.18741289286898621)],[(-3.8010223726087475,-4.555270651862383),(-4.295555878780673,-2.6173757601633265),(-6.233450770479729,-3.1119092663352483)]]

start = robots[0]
goal = robots[1]
rand = (-40,50)
global_unawakenRobots = [item for item in robots if item != start]

global_path = []
global_robots_queue = Queue()
startLoop = {'startCoord': start, 'obstacleList': obstacleList, 'previousPath': [], 'rand': rand, 'numRobotsAtNode': 1}
global_robots_queue.enqueue(startLoop)
awakeRobots()

print "global path"
print global_path

# samLines(global_path)



drawRobots(robots)
drawPolygons(obstacleList)


plt.axis('scaled')
plt.grid(True)
plt.pause(0.01)  # Need for Mac
plt.show()
