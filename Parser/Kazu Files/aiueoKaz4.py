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
        print lines
        prex = "f"
        prey = "f"
        randColor = numpy.random.rand(3,1)
        for (x,y) in lines:
            print (x,y)
            if prex == "f":
                prex = x
                prey = y
            else:
                line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None,edgecolor = randColor)
                plt.gca().add_patch(line)
                prex = x
                prey = y
                print("connecting",(x,y),"to",(prex,prey))

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

class RRT():
    u"""
    Class for RRT Planning
    """
    
    def __init__(self, start, goal, obstacleList, randArea,expandDis=1.0,goalSampleRate=10,maxIter=500):
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
    
    def Planning(self,animation=True):
        u"""
        Pathplanning 

        animation: flag for animation on or off
        """
        robots = self.robots
        
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
            for (gx,gy) in robots:
                # check goal
                dx = newNode.x - gx
                dy = newNode.y - gy
                d = math.sqrt(dx * dx + dy * dy)
                if d <= self.expandDis:
                    if not self.__CollisionCheck(newNode, obstacleList,Node(gx,gy)): #if intersects
                        continue
                    else:
#                        print "ok"
#                        print (gx,gy)
                        path += [[gx,gy]]
                        foundNode.append((gx,gy))
                        check = True
                        #print("Goal!!")
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
#        
#        print global_robots_queue.printItems()
        nextRobot = global_robots_queue.dequeue()
        currentNode = nextRobot['startCoord']
        previousPath = nextRobot['previousPath']
        numRobotsAtNode = nextRobot['numRobotsAtNode']
        rand = nextRobot['rand']
        rrtshortestpath(currentNode,obstacleList,previousPath,rand,numRobotsAtNode)
    
    print "complete awakening all robots"
    return




#=====end of RRT class

def rrtpath(obstacles,startcoord,goalcoord,randAreas):
    rrt = RRT(start=startcoord, goal=goalcoord,randArea = randAreas, obstacleList=obstacles)
    answer = rrt.Planning(animation=True)
    path= answer['path']
    smoothiePath = supersmoothie(path,obstacles)
    plt.plot([x for (x,y) in smoothiePath], [y for (x,y) in smoothiePath],'-r')
    smoothiePath.reverse()
#    print smoothiePath
    return {'path': smoothiePath, 'node': answer['node']}


def rrtshortestpath(currentNode,obstacleList,previousPath,rand,numAct):
    global global_unawakenRobots
    global global_path
    unawakenRobots = global_unawakenRobots
#    print ("left",unawakenRobots)
#    print len(unawakenRobots)
    if len(unawakenRobots) == 0:
#        print "0 left"
#        print previousPath
        global_path.append(previousPath)
        return
    if ((numAct == 2) & (len(unawakenRobots) == 1)):
#        print "final"
        a= rrtpath(obstacleList,currentNode[0],unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        newPath = previousPath + a['path']
#        print newPath
        global_path.append(newPath)
        return
    if numAct == 1:
#        print "num = 1"
        a = rrtpath(obstacleList,currentNode,unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        print a['path']
        newPath = previousPath + a['path']
        print newPath
#        print "finding next node"
        nextLoop = {'startCoord': foundNode[0], 'obstacleList': obstacleList, 'previousPath': newPath, 'rand': rand, 'numRobotsAtNode': 2}
        global_robots_queue.enqueue(nextLoop)
#        rrtshortestpath(foundNode[0],obstacleList,newPath,rand,2)
        return
    elif numAct == 2:
#        print "num = 2"
        if(type(currentNode)== tuple):
            fixedTypeCurrentNode = currentNode
#            print "tuple"
        else:
#            print "list"
            fixedTypeCurrentNode = currentNode[0]
#        print (currentNode,unawakenRobots[-1])
        a = rrtpath(obstacleList,fixedTypeCurrentNode,unawakenRobots[-1],rand)
        foundNode = a['node']
        global_unawakenRobots = [item for item in unawakenRobots if item != foundNode[0]]
        newPath = previousPath + a['path']
        nextLoop = {'startCoord': foundNode, 'obstacleList': obstacleList, 'previousPath': newPath, 'rand': rand, 'numRobotsAtNode': 2}
#        print "robot 1"
#        print newPath
#        print nextLoop
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
#        print "robot 2"
#        print newPath
#        print nextLoop

        global_robots_queue.enqueue(nextLoop)
#        rrtshortestpath(foundNode,obstacleList,newPath,rand,2)
        return

    return




robots = [(-2.509278431748773,-3.922474465367287),(-7.107424404849778,-2.812756223068684),(3.5205959751531815,-2.931531824191513),(-7.165446396882602,-2.0996148548443516),(6.02935731440502,-5.261094048485732),(3.904262207651888,3.8089575795269255),(3.8286975127967278,-0.7133363957594243),(-0.7240838304088939,-4.431190351496525),(-4.258944187533284,-1.801253934472979),(4.619657165890612,4.738368197549777),(5.672547844111952,-4.923136860246734),(-6.510714164511064,6.014885179901174),(6.581752005102803,0.18410284239318475),(5.317897497716265,-5.372175391088058),(-4.212500820662962,-2.8633375380140915),(3.5122586806621463,5.91313092720547),(2.6460362697667765,-0.4982818680142582),(5.410959780808819,-3.6325355845202614),(-4.133467016668308,4.295869773947251),(4.03600557632266,4.107086378184382),(-5.661023087090978,6.764854060140824),(1.4402600866838302,6.001557518067644),(-5.829483106173848,1.8651027848129633),(-1.3766027532160994,4.320463330260083),(-4.469108649942299,7.097174444202885),(0.5628677886049331,2.515468303902562),(-4.991570759314196,-5.235328587210487),(1.4869652860007783,2.7185729833074337),(-5.3608768725523435,0.9033171866452108),(3.115313432380285,-6.6080911392585335),(-3.2170497394270394,-6.227638705361352),(-2.380909363734613,4.562645764248117),(-7.0343060308033225,-1.6028584756751103),(-4.661304978315947,0.3553126357573486),(-6.867426333014263,-5.170612605726073),(3.2373722049522513,6.162574651654387),(-3.282106514274324,-5.3984475412014135),(-1.426665057069048,2.4882676171902816),(5.033131713589478,-6.830275315499104),(-6.492910942748091,1.8647967575065074),(2.585098378745954,7.206666825595523),(-5.401014718898611,-3.1678586262311166),(0.9536954973571419,-6.5182201296921125),(-4.104290314438201,0.5092942289412257),(-3.3217521540662687,2.1762684719253453),(0.5464145883053995,-3.504437781952001),(5.308481515220439,-2.5365574967522955),(4.912030554154432,-0.10028864381661151),(5.600728839868731,-1.9561724825998574),(6.200811637319177,-5.448107199870678),(2.64730920939067,1.8263613146733402),(5.64104932154363,-5.031912652145638),(-2.3663494648784793,0.13930114273762673),(6.417562847170576,3.1844357956815674),(4.731673017035201,4.386389589111374),(0.9953228128291833,3.3276036064694896),(-5.701110125272803,-5.290649043703827),(3.43682143756493,1.0385493422266148),(-3.2840427333495765,0.9907302636224458),(-4.499080404523966,0.9031554062374205),(2.352516534046387,2.58805513042096),(-3.604202540338137,-2.1702445844538927),(2.179391449640285,-1.3421347720612467),(1.810578061538699,-6.532862788345355),(0.9070906635724008,-4.2408869038883),(2.6969355415368277,-5.942484955498676),(5.256278118345433,6.056418419547386),(2.648074708173528,3.16016447083709),(-4.46904208420311,0.8168799509356717),(-0.3234163025549259,2.993918643672721),(-5.087565619438815,3.2832414183556136),(3.934212456166625,2.850836678530495),(-3.653322991474971,0.7056556745486544),(-6.062328998357615,-3.8864146373377912),(2.0539624422369744,5.767517065705032),(-6.809719062396742,-2.6533272998985273),(-3.410759460024167,-6.240018706598062),(-0.15516844561726817,4.2556799399768686),(5.634222136161025,-0.7107465595646207),(-5.147134926128004,6.627319313279637),(-4.604102817639647,-2.5373985626171223),(3.6432370143703547,-7.058744109387414),(-3.0118962135626557,-6.923736189742284),(0.16049196565801083,3.891677367038831),(2.2284547844821265,6.379594219844078),(-6.703781870167221,-6.3826895050622685),(-6.403787296661032,-6.1575150500005),(1.6955006203081062,-4.518822461749798),(-4.224391571115705,-6.519620871270878),(5.3577103166299835,5.493536772140185),(2.2257112856617844,6.28437656246752),(0.1376899102716278,0.3661161641895223),(2.8013222890430836,-0.7154465187036321),(-3.121978510814876,-5.597410563455873),(0.2123851441377651,3.7303777314029425),(3.9203827100840076,-2.613033839280704),(3.080714203736182,2.243546068654556),(0.41375873100907334,-0.8813116265247913),(-1.9839214852818925,2.0017003002229146),(3.3944386338193464,-5.063952933625005),(-2.3897660926973447,1.6089836053703461),(6.043016516335708,1.3820531921493746),(-4.369209108254726,2.7722463573958454),(6.618350172682893,-3.839448869766444),(4.342114962672925,-3.721106726677296),(4.266605672860658,-2.421616943601528),(5.49281093953012,-5.408431476878537),(-5.611570199982603,-4.362853735062409),(1.7547113674155739,3.28661450184239),(-7.012954812694112,-1.2188171679171989),(1.7928274018585064,-5.7821045076340924),(-2.4751282296955495,-6.274138254189922),(1.9948673412127924,6.712639723435952),(-5.055026907578888,6.482673894926847),(6.23011950365585,2.7587766225447012),(-5.608616394764281,3.8568901689680573),(-2.964486039549638,-0.03926154790003178),(-6.755464521423876,-6.942436338977626),(4.300765499369705,-0.4090056159053894),(-4.302540987272197,-7.05116707145715),(-3.288341724301954,-0.028074885550654294),(6.480930689351405,7.06165943117187),(6.658658662130925,-0.9691408792231417),(-3.756594103041084,0.27779176318686627),(0.3822110158850389,2.745098463841094),(4.246744316470217,-5.192874619270508),(-3.8106622757871733,-2.9598451653197575),(-1.0420292261464636,-0.2783687581562271),(1.3828653114467873,-3.1565083929238),(-6.493609883788841,4.459239892921888),(-7.092295759777315,-3.58953969114361),(0.9856006088542273,5.862206717937992),(-1.5742172971790387,-6.70345590697619),(-7.048581827547602,-6.354974838673925),(-2.400820966124291,-4.783669781108644),(-4.088275918409511,-6.846508961850756),(6.475556236630339,-4.691664618431153),(3.4789304169617647,3.241757981457102),(-0.3671026140359679,-0.5311950060133803),(-2.9814206233224576,1.4335880697590895),(4.444685956037189,-0.7104808082351877),(-0.6565263214933692,6.977925922162685),(-6.19088469017654,3.076640390913985),(5.094418087985527,5.300340545809133),(-2.2714647667830734,-5.9506964011952235),(5.7766526040149735,-1.499794668205296),(-2.3230739780952394,0.5523372251686292),(-1.8235780997281017,-0.6618280383352211),(3.5155462017355035,0.8326083567784481),(-3.9875006454793804,-4.274264087004985),(4.308835988632566,0.4360136309600069),(1.5026691129641776,1.664349649294838),(5.524945969139425,0.4493819004835311),(-1.5424545196668964,-3.607158158816202),(2.2099186029916993,6.509921209483021),(-6.563770759812223,1.9653625745365666),(2.8559959120481686,2.628991491590657),(-4.825203615628791,4.599068845515459),(-4.611754111367001,-3.9767103378887603),(6.0926564782806985,-1.6225181771760608),(-1.5488093659145967,4.200031266641098),(5.201559144156537,-4.78274738628399),(-5.923195015247982,4.255765967900116),(-2.2679635938264795,-5.640929150333175),(4.299869300802598,6.433342081092993),(4.820558672144051,5.109102686508308),(-2.5634159911052627,-3.181599660537377),(4.087614059307491,2.425515343017313),(-6.880683057403824,-1.5205729526753187),(4.695113361268309,-5.758275293155353),(-3.561051946081937,4.816117511821119),(-6.657684554223266,5.5945949600314755),(0.8062360312432384,3.821994328546918),(-1.0936750718960218,2.842491621638022),(-6.864450541867754,5.681601075351271),(0.8721616228480613,-6.253330510341845),(-1.6433589126154446,-3.128188194163302),(-1.2293444364672474,-5.3975448680785085),(5.829207147154699,5.011189717994146),(6.07532881237608,2.6742558073132674),(-1.8268009471382518,7.3936016499164765),(-0.4163471146471478,2.975702424299028),(2.4389008038819817,1.338988270372509),(-3.755390967400505,4.3223591780902115),(4.0131141805082695,-0.9713686348163293),(6.4837291274332305,-1.9533090162724536),(-3.3706902149400704,4.08019451990738),(-1.5855466650490788,-3.6271817511342577),(-3.7071886148893185,1.9119826544498615),(-2.848196044010109,2.298500296645228),(-4.657346587377319,-3.3993820336985596),(2.3135391447245883,6.667568960186527),(2.205645924825836,-5.769573710690249),(-3.599663475239638,0.16243179731282797),(-6.43068118555877,6.694405308128934),(-6.784255813756791,7.135826720212467),(1.8963785156395598,-2.13684885623311),(-3.8820775708747366,0.07304985731408031),(-4.901984836660368,-4.510404808405855),(1.7692026905781102,-0.6725776949655788),(-1.620786627677595,-5.534494139194846),(4.958047398555081,-6.302913920791891),(6.007360514485725,5.765044467090126),(-6.469340840991886,3.4437393608613487),(-4.510362576755558,6.402103653248181),(-1.7136847788117553,-0.13967681835706447),(-3.2896484538380615,1.4797310252353117),(2.3486322710099365,7.1244911959120145),(-5.041041649153464,-5.457792618915386),(2.5188599177806523,-0.7458185020932762),(2.4506010099590103,-6.805872837311955),(-4.8692750335596875,-5.283636863835779),(-4.909515784849731,4.73886190822294),(6.008611513224201,-4.7234747516005715),(1.0786318441604816,-6.202726976191007),(-2.350277657058114,-3.498677894980262),(-3.694394637307031,0.6043661930396693),(6.332473434618551,0.5104354921577228),(-1.5930589133839117,-5.21666868387874),(-4.477845561348885,2.653473861921551),(-3.6324133217979466,1.580447552062532),(5.216839601868527,0.9767570281829343),(-5.498340257061826,-6.028496925528255),(0.28868456543602683,2.9230056084608247),(4.833083721177658,-7.09729518441579),(-6.212715869753461,2.3832392316070976),(-3.550117992270491,-4.623978452508352),(-6.673693715702141,-0.926466763936018),(5.878593678642627,-1.777239130505178),(4.253699116986924,4.814194116978344),(3.0522811355226,2.490735310605979),(-3.741474102186928,0.4787908419896407),(-6.812540666537432,2.3293814807438995),(-4.925575609571624,-3.0615924062788027),(-7.1011633263227525,-7.1094695373898515),(5.358325624225062,-1.7165362925464462),(5.096131702103804,-6.847523048899361),(-4.805470128794431,0.3753209965691173),(0.5798533022611041,6.44988178192459),(5.189110729840484,5.655510666891937),(6.567673212084294,-0.021162344812560185),(-6.329366079256283,0.7195458807462511),(-2.8065172884750647,0.874511641191484),(5.060924200891,-5.456383546383301),(5.238977151366544,-0.6208396304951345),(4.981341782062115,5.536028586975251),(3.895213552779783,4.984609859266788),(2.119264164904214,-4.233660283690835),(0.5209874250802216,2.7951042573439597),(-4.6814013665772025,-1.1280486137700922)
]

obstacleList = [[(-0.07794686776757936,0.08148296437456723),(0.9672212808083951,1.7866584806198216),(-0.7379542354368591,2.8318266291957963)],[(0.3513768944551585,0.13278465197453843),(1.0499920632236377,-1.7412315822045012),(1.987000180313159,-1.3919239978202602),(1.6376925959289181,-0.4549158807307419),(3.5117088301079575,0.24369928803773955),(3.1624012457237174,1.180707405127259),(2.225393128634197,0.8313998207430187),(1.8760855442499584,1.768407937832538),(1.288385011544678,0.4820922363587786)],[(1.8186732162791026,-3.068229883882896),(3.121068849458158,-4.586045951354881),(3.879976883194153,-3.934848134765352),(3.2287790666046243,-3.1759401010293593),(4.746595134076608,-1.873544467850294),(4.095397317487074,-1.1146364341143031),(3.3364892837510824,-1.7658342507038343),(2.685291467161554,-1.0069262169678428),(2.577581250015093,-2.4170320672933654)],[(-3.0445209165241534,-0.031338559624686985),(-4.621218983389218,-1.2617950626933503),(-4.005990731854883,-2.050144096125886),(-3.217641698422353,-1.4349158445915524),(-1.987185195353684,-3.0116139114566156),(-1.1988361619211567,-2.3963856599222852),(-1.81406441345549,-1.6080366264897512),(-1.0257153800229557,-0.9928083749554207),(-2.4292926649898217,-0.8196875930572193)],[(0.9651616484208912,-2.9070900858036173),(-0.3049399143253325,-1.362149303666776),(-1.0774103053937543,-1.9972000850398874),(-0.4423595240206425,-2.7696704761083084),(-1.9873003061574832,-4.039772038854536),(-1.3522495247843742,-4.812242429922952),(-0.5797791337159502,-4.177191648549838),(0.055271647657157974,-4.949662039618261),(-1.4896691344796862,-6.219763602364482),(-0.8546183531065769,-6.992233993432908),(-0.08214796203814967,-6.357183212059795),(0.5529028193349608,-7.129653603128215),(0.6903224290302707,-5.722132430686684),(1.4627928200986966,-5.087081649313573),(0.19269125735247017,-3.5421408671767285)],[(3.5530839034361423,3.7551199964726276),(3.903137990290535,5.724247248906185),(2.918574364073753,5.8992742923333825),(2.743547320646558,4.914710666116603),(0.7744200682130007,5.264764752970996),(0.5993930247858037,4.280201126754213),(1.583956651002584,4.105174083327018),(1.4089296075753883,3.120610457110237),(2.5685202772193634,3.9301470398998224)],[(-2.041437858901194,1.9405408119544008),(-1.0891607705626307,3.6992813690080624),(-1.9685310490894594,4.175419913177345),(-2.444669593258742,3.2960496346505144),(-4.203410150312399,4.248326722989085),(-4.6795486944816815,3.3689564444622597),(-3.8001784159548526,2.89281790029297),(-4.276316960124138,2.0134476217661432),(-2.9208081374280224,2.416679356123687)],[(-0.3991266655847069,0.15185964346437272),(-1.3166992340276564,1.9289529391697517),(-3.093792529733037,1.0113803707267988)],[(-4.941351899891012,0.13003663020657785),(-6.810714203730782,0.8410110158090838),(-7.166201396532038,-0.09367013611079972),(-6.231520244612153,-0.4491573289120514),(-6.942494630214652,-2.3185196327518316),(-6.00781347829477,-2.6740068255530804),(-5.652326285493513,-1.7393256736331937),(-4.717645133573629,-2.094812866434448),(-5.296839092692264,-0.8046445217133078)],[(-0.8627555769969231,4.361421573380015),(0.8536984615321284,5.387962139367205),(0.3404281785385379,6.246189158631732),(-0.5177988407259909,5.732918875638137),(-1.5443394067131804,7.44937291416719),(-2.4025664259777075,6.936102631173597),(-1.8892961429841115,6.077875611909067),(-2.747523162248637,5.564605328915478),(-3.774063728235819,7.281059367444536),(-4.63229074750036,6.767789084450936),(-4.1190204645067645,5.9095620651864),(-4.9772474837712855,5.396291782192819),(-3.605750181513166,5.051335045921885),(-3.0924798985195734,4.193108026657351),(-1.376025859990516,5.219648592644543)],[(4.671527194080773,1.6786716154520969),(6.667109521925088,1.5458138990956323),(6.733538380103324,2.5436050630177918),(5.735747216181165,2.6100339211960226),(5.868604932537622,4.605616249040343),(4.870813768615465,4.672045107218574),(4.804384910437233,3.6742539432964127),(3.8065937465150714,3.740682801474645),(4.737956052259003,2.6764627793742553)]]



start = robots[0]
goal = (7,5)
rand = (-2,10)
global_unawakenRobots = [item for item in robots if item != start]

global_path = []
global_robots_queue = Queue()
startLoop = {'startCoord': start, 'obstacleList': obstacleList, 'previousPath': [], 'rand': rand, 'numRobotsAtNode': 1}
global_robots_queue.enqueue(startLoop)
awakeRobots()

print "global path"
print global_path

samLines(global_path)



drawRobots(robots)
drawPolygons(obstacleList)


plt.axis('scaled')
plt.grid(True)
plt.pause(0.01)  # Need for Mac
plt.show()
