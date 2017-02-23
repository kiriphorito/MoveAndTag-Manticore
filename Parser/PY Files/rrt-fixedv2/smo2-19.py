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

obstacleList = [[(0.18975287913547792,-0.1650233858614687),(1.9561477353155101,0.7729793924183843),(1.4871463461755836,1.6561768205084004),(0.6039489180855675,1.1871754313684741),(-0.3340538601942856,2.9535702875485064),(-1.217251288284301,2.48456889840858),(-0.7482498991443747,1.6013714703185638),(-1.631447327234391,1.1323700811786377),(-0.2792485100044484,0.7181740422285475)],[(0.7939926466247189,-1.6536555312509535),(1.4198292398197592,-3.5532156273305757),(2.3696092878595723,-3.2402973307330543),(2.0566909912620512,-2.290517282693244),(3.956251087341673,-1.6646806894982014),(3.643332790744153,-0.714900641458391),(2.693552742704341,-1.027818938055912),(2.3806344461068223,-0.07803889001610309),(1.7437726946645298,-1.3407372346534334)],[(-2.661166902604846,-3.7047890678650806),(-1.936117819758969,-5.568737518725337),(-1.0041435943288395,-5.206212977302397),(-1.3666681357517778,-4.27423875187227),(0.4972803151084779,-3.549189669026384),(0.13475577368553715,-2.617215443596258),(-0.7972184517445922,-2.979739985019199),(-1.1597429931675325,-2.04776575958907),(-1.7291926771747193,-3.342264526442139)],[(-0.12659524820354592,0.040822222342230076),(-1.8690669595418767,1.0225514799207485),(-2.8507962171203958,-0.7199202314175798)],[(2.9509499042264613,2.527978956704507),(3.23486244687614,0.5482330972250158),(4.224735376615889,0.6901893685498541),(4.082779105291049,1.680062298289603),(6.062524964770542,1.963974840939287),(5.920568693445698,2.9538477706790336),(4.930695763705951,2.811891499354193),(4.78873949238111,3.8017644290939394),(3.9408228339662053,2.669935228029349)],[(0.662932371505907,3.620671422361223),(1.691482396519182,1.905420756701772),(2.54910772934891,2.4196957692084107),(2.0348327168422697,3.277321102038135),(3.7500833825017215,4.305871127051407),(3.2358083699950826,5.163496459881138),(2.378183037165357,4.649221447374499),(1.8639080246587192,5.506846780204226),(1.5205577043356318,4.134946434867861)],[(-1.6741954965936179,2.1017776302143205),(-3.6383035157083077,2.4789774303857497),(-4.015503315879737,0.5148694112710572)],[(2.33562710228351,6.38773334259324),(4.249176183782569,5.806072004103132),(4.5400068530276245,6.7628465448526605),(3.583232312278093,7.053677214097715),(4.164893650768206,8.967226295596774),(3.2081191100186706,9.25805696484183),(2.917288440773624,8.301282424092296),(1.960513900024089,8.592113093337353),(2.6264577715285666,7.344507883342768)],[(4.030553162323309,-1.1192467813456652),(4.528138688344358,-3.0563602639001473),(6.465252170898841,-2.558774737879099)],[(-3.01774450696405,-4.373711148540161),(-4.859847123416839,-3.594828647479966),(-5.249288373946939,-4.515879955706359),(-4.328237065720542,-4.905321206236461),(-5.107119566780739,-6.747423822689248),(-4.186068258554341,-7.1368650732193455),(-3.796627008024247,-6.21581376499295),(-2.8755756997978508,-6.605255015523048),(-3.654458200858046,-8.447357631975837),(-2.7334068926316517,-8.836798882505935),(-2.3439656421015496,-7.91574757427954),(-1.4229143338751598,-8.305188824809639),(-1.9545243915714583,-6.994696266053148),(-1.5650831410413606,-6.073644957826755),(-3.4071857574941466,-5.294762456766556)],[(1.1798493732931583,-7.288553571680055),(2.026678171857574,-9.100425803908902),(2.9326142879720014,-8.677011404626692),(2.509199888689791,-7.771075288512268),(4.321072120918637,-6.924246489947851),(3.8976577216364268,-6.018310373833426),(2.9917216055220033,-6.441724773115636),(2.5683072062397985,-5.535788657001214),(2.0857854894075807,-6.865139172397845)],[(-4.924127815666981,-3.5546241619589463),(-3.70570119519226,-1.9686113718092113),(-4.498707590267129,-1.359398061571849),(-5.10792090050449,-2.1524044566467184),(-6.693933690654222,-0.9339778361719917),(-7.303147000891583,-1.7269842312468606),(-6.510140605816712,-2.336197541484223),(-7.119353916054079,-3.1292039365590925),(-8.705366706203813,-1.9107773160843733),(-9.314580016441177,-2.7037837111592427),(-8.521573621366308,-3.312997021396602),(-9.130786931603666,-4.1060034164714745),(-7.728567226291437,-3.9222103316339654),(-6.935560831216571,-4.5314236418713225),(-5.7171342107418495,-2.945410851721586)],[(-1.6688257203527526,5.896073644055235),(-2.5763946228964514,4.11385068181521),(-1.6852831417764405,3.660066230543357),(-1.2314986905045822,4.551177711663372),(0.5507242717354539,3.6436088091196805),(1.0045087230072944,4.534720290239692),(0.1133972418872784,4.988504741511548),(0.5671816931591289,5.879616222631561),(-0.7777142392327379,5.442289192783388)],[(6.3772200246821065,0.8863048457838263),(6.513438438878939,-1.1090508978452873),(7.511116310693495,-1.0409416907468743),(7.443007103595081,-0.04326381893231346),(9.438362847224198,0.09295459526452599),(9.370253640125778,1.090632467079082),(8.37257576831122,1.0225232599806637),(8.304466561212802,2.0202011317952215),(10.299822304841918,2.156419545992054),(10.231713097743494,3.1540974178066232),(9.234035225928935,3.0859882107082033),(9.16592601883052,4.083666082522754),(11.161281762459645,4.219884496719585),(11.093172555361223,5.2175623685341455),(10.095494683546658,5.149453161435737),(10.027385476448236,6.147131033250294),(9.0978168117321,5.081343954337317),(8.100138939917539,5.0132347472389),(8.236357354114382,3.0178790036097816),(7.238679482299823,2.9497697965113625),(7.3748978964966625,0.9544140528822446)],[(5.785473652621135,-3.078521424219244),(3.8344379305352314,-3.5183618624974082),(4.054358149674311,-4.49387972354036),(5.029876010717262,-4.273959504401279),(5.469716448995422,-6.224995226487184),(6.4452343100383755,-6.005075007348104),(6.225314090899295,-5.029557146305149),(7.200831951942248,-4.809636927166073),(6.005393871760215,-4.054039285262196)],[(6.579585622719702,3.969075715945165),(7.153396498044602,5.8849935323869005),(6.195437589823731,6.1718989700493525),(5.908532152161279,5.213940061828481),(3.992614335719541,5.787750937153369),(3.705708898057094,4.8297920289325),(4.6636678062779655,4.5428865912700545),(4.376762368615518,3.5849276830491883),(5.621626714498833,4.25598115360761)],[(-0.46882404686579754,-1.0205006376199117),(-2.448343208645851,-0.7350117794387336),(-2.733832066827031,-2.714530941218787)],[(-5.5664843460437385,1.7380089465954538),(-7.526736320763721,1.3412568884956844),(-7.328360291713842,0.36113090113569735),(-6.348234304353845,0.5595069301855766),(-5.951482246254074,-1.4007450445344065),(-4.971356258894083,-1.2023690154845217),(-5.169732287943965,-0.2222430281245258),(-4.189606300583974,-0.02386699907464429),(-5.368108316993853,0.7578829592354619)]]
rand = (-12, 15)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (1.3048920678710019,4.656854955326798)
goal = (1.3844452734400168,5.2984113754190965)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.3844452734400168,5.2984113754190965)
goal = (1.0457818460211428,3.961593535096485)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.0457818460211428,3.961593535096485)
goal = (1.661478332658053,6.202982955248258)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.661478332658053,6.202982955248258)
goal = (2.1692447333842075,3.3243861514208835)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.1692447333842075,3.3243861514208835)
goal = (-2.532504936580061,4.3062892466625105)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.532504936580061,4.3062892466625105)
goal = (5.199732525106224,1.775288303047443)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.199732525106224,1.775288303047443)
goal = (-4.316641633773274,-0.730859772989298)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (1.3844452734400168,5.2984113754190965)
goal = (1.83329468942574,5.488633954916434)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.83329468942574,5.488633954916434)
goal = (1.1571547700246825,5.842453317936647)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1571547700246825,5.842453317936647)
goal = (0.994257171123154,6.474992222578125)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.994257171123154,6.474992222578125)
goal = (-0.2570246585689002,5.722983304184501)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.2570246585689002,5.722983304184501)
goal = (-2.579639242712564,5.492507685501115)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.579639242712564,5.492507685501115)
goal = (0.2912395472122835,-0.36648588068440446)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.2912395472122835,-0.36648588068440446)
goal = (8.812580422165553,1.4199224734606144)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (1.83329468942574,5.488633954916434)
goal = (2.220142178368338,5.054138591165577)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.220142178368338,5.054138591165577)
goal = (2.2803625552017266,6.217114147923281)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.2803625552017266,6.217114147923281)
goal = (3.5304791475372994,5.191733078507923)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.5304791475372994,5.191733078507923)
goal = (5.282797536573733,7.303181952246437)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.282797536573733,7.303181952246437)
goal = (6.999414112169948,7.489477215517962)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.999414112169948,7.489477215517962)
goal = (9.649170336765756,1.749850764625867)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (1.0457818460211428,3.961593535096485)
goal = (0.9131857309443845,3.0547956790749886)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.9131857309443845,3.0547956790749886)
goal = (0.8257048173703598,3.044696698101408)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.8257048173703598,3.044696698101408)
goal = (0.007476350642660279,3.6327111467516193)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.007476350642660279,3.6327111467516193)
goal = (-1.530069412321252,1.907992725744732)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.530069412321252,1.907992725744732)
goal = (-0.7741052468500413,-0.342464911127875)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.7741052468500413,-0.342464911127875)
goal = (-3.4608418453239533,-1.7074204625784875)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (1.1571547700246825,5.842453317936647)
goal = (0.013082903891710629,6.0932990005277965)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.013082903891710629,6.0932990005277965)
goal = (3.644393289286727,5.597051681170639)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.644393289286727,5.597051681170639)
goal = (-1.3772632831244795,8.674454690352253)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.3772632831244795,8.674454690352253)
goal = (-4.466663850501701,5.953620950270334)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.466663850501701,5.953620950270334)
goal = (-6.952320580405702,2.6897855885159565)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (2.220142178368338,5.054138591165577)
goal = (3.3752992761268086,5.195133587306016)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.3752992761268086,5.195133587306016)
goal = (4.661347983941077,7.268192090575489)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.661347983941077,7.268192090575489)
goal = (5.929188408961524,5.3043905665079265)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.929188408961524,5.3043905665079265)
goal = (4.958861261307957,0.5991426298331373)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.958861261307957,0.5991426298331373)
goal = (7.226480740054669,-1.7374694499632106)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (0.9131857309443845,3.0547956790749886)
goal = (0.18537301673612028,2.761044957467771)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.18537301673612028,2.761044957467771)
goal = (2.1230096121288575,1.5283545142678658)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.1230096121288575,1.5283545142678658)
goal = (-1.9718834136741208,2.3754779316819423)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.9718834136741208,2.3754779316819423)
goal = (-2.582742591674032,0.3787627136532059)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.582742591674032,0.3787627136532059)
goal = (2.058427036394841,-3.721210604502362)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (1.661478332658053,6.202982955248258)
goal = (0.007546616373096171,6.7439626049645565)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.007546616373096171,6.7439626049645565)
goal = (-0.5203606468045017,8.27314485512063)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.5203606468045017,8.27314485512063)
goal = (4.236702777916532,8.747896925576356)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.236702777916532,8.747896925576356)
goal = (6.654538286257495,8.256941867478705)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.654538286257495,8.256941867478705)
goal = (9.97016764866385,1.532505675715944)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (0.994257171123154,6.474992222578125)
goal = (1.4677896740730425,9.219803341669643)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.4677896740730425,9.219803341669643)
goal = (-2.4196180975041406,8.53753475039337)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.4196180975041406,8.53753475039337)
goal = (-4.657037719422262,4.9714020985151794)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.657037719422262,4.9714020985151794)
goal = (-7.8620272914248455,2.3919843951951876)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (2.2803625552017266,6.217114147923281)
goal = (5.079984436432216,7.551412484357623)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.079984436432216,7.551412484357623)
goal = (5.464998611586095,7.494343778958942)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.464998611586095,7.494343778958942)
goal = (6.969065879835853,7.66961079823443)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.969065879835853,7.66961079823443)
goal = (9.469199842685388,0.7309258637603051)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (0.8257048173703598,3.044696698101408)
goal = (1.9098861198632573,0.9738183918878889)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.9098861198632573,0.9738183918878889)
goal = (-1.91390513260182,1.7802439638961651)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.91390513260182,1.7802439638961651)
goal = (0.7473979790495768,-1.3576469059804186)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.7473979790495768,-1.3576469059804186)
goal = (4.857628924242007,-3.0570204069073252)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (0.013082903891710629,6.0932990005277965)
goal = (-2.408462950602024,6.648440348469425)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.408462950602024,6.648440348469425)
goal = (-3.3008663616936884,5.884584663276014)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.3008663616936884,5.884584663276014)
goal = (-4.010473517254802,2.474972694037822)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.010473517254802,2.474972694037822)
goal = (-6.5222116332259645,0.0298704635865068)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (3.3752992761268086,5.195133587306016)
goal = (5.517765580424518,4.099978412125667)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.517765580424518,4.099978412125667)
goal = (6.24914296102857,6.182840074547956)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.24914296102857,6.182840074547956)
goal = (7.177240050365981,3.0535831846922417)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.177240050365981,3.0535831846922417)
goal = (9.88634329601988,1.2148329163751956)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (0.18537301673612028,2.761044957467771)
goal = (-1.334479466443332,2.02141118268573)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.334479466443332,2.02141118268573)
goal = (-2.2531746975394302,1.4617227551243595)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.2531746975394302,1.4617227551243595)
goal = (-3.7705173654261825,2.1656999163412287)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.7705173654261825,2.1656999163412287)
goal = (-3.2524539896392923,-3.0714835810103276)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (0.007546616373096171,6.7439626049645565)
goal = (-0.6694300431115465,8.883611324280842)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.6694300431115465,8.883611324280842)
goal = (-3.1506717803256032,8.193071244745742)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.1506717803256032,8.193071244745742)
goal = (-5.277127130201424,4.222058068064696)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.277127130201424,4.222058068064696)
goal = (-6.526335271014026,-0.10731167081292448)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (2.1692447333842075,3.3243861514208835)
goal = (2.491842720581136,1.30151636486233)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.491842720581136,1.30151636486233)
goal = (4.134942054385965,0.6585743577200542)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.134942054385965,0.6585743577200542)
goal = (5.542630131237841,0.9551168684445397)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.542630131237841,0.9551168684445397)
goal = (6.70584994205765,-2.4289891678078925)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (-0.2570246585689002,5.722983304184501)
goal = (-2.826404193197545,4.044855013380902)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.826404193197545,4.044855013380902)
goal = (-5.182278803829787,3.9625188471933015)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.182278803829787,3.9625188471933015)
goal = (-5.251666880419252,-1.4318164028071454)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (3.5304791475372994,5.191733078507923)
goal = (6.6036659475737025,6.099263313257575)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.6036659475737025,6.099263313257575)
goal = (7.87795003429348,4.701796065384713)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.87795003429348,4.701796065384713)
goal = (10.02281481273668,1.4196861687299975)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (0.007476350642660279,3.6327111467516193)
goal = (-2.586489584580514,3.1362203136195905)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.586489584580514,3.1362203136195905)
goal = (-4.313136602768431,2.363224205505622)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.313136602768431,2.363224205505622)
goal = (1.0682347265070344,-4.276791871100912)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (3.644393289286727,5.597051681170639)
goal = (6.14010363938481,7.7910107450449395)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.14010363938481,7.7910107450449395)
goal = (7.933592268403963,6.003941734878323)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.933592268403963,6.003941734878323)
goal = (9.567680018756494,0.5853537159853666)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (4.661347983941077,7.268192090575489)
goal = (4.665785936327893,8.809354313219487)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.665785936327893,8.809354313219487)
goal = (7.594973361285607,8.786316379812497)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.594973361285607,8.786316379812497)
goal = (10.67102570587982,1.5377034218799288)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (2.1230096121288575,1.5283545142678658)
goal = (2.7382344445437905,-0.013191495911026863)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.7382344445437905,-0.013191495911026863)
goal = (5.112218112518956,-0.7654803602954541)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.112218112518956,-0.7654803602954541)
goal = (1.2496555373964426,-4.453140585641847)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (-0.5203606468045017,8.27314485512063)
goal = (-4.550670849395055,8.370912496486977)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.550670849395055,8.370912496486977)
goal = (-5.825647895307556,8.561355849338632)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.825647895307556,8.561355849338632)
goal = (-9.167325226546291,4.018389644006154)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (1.4677896740730425,9.219803341669643)
goal = (5.4217815531225035,8.703606228596975)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.4217815531225035,8.703606228596975)
goal = (8.368357408020596,6.736141529610633)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.368357408020596,6.736141529610633)
goal = (9.752062232536485,0.20038768186249456)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (5.079984436432216,7.551412484357623)
goal = (6.6526267671554695,8.251241264020653)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.6526267671554695,8.251241264020653)
goal = (8.845027637240587,7.612131846421285)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.845027637240587,7.612131846421285)
goal = (11.061717948903699,1.0540660224241183)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (1.9098861198632573,0.9738183918878889)
goal = (0.9068867248955765,-0.35347463107708776)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.9068867248955765,-0.35347463107708776)
goal = (3.7595429186834206,-1.882916559834963)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.7595429186834206,-1.882916559834963)
goal = (0.6753240452168008,-4.903745038989502)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (-2.408462950602024,6.648440348469425)
goal = (-3.4302015186321597,5.403154605094205)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.4302015186321597,5.403154605094205)
goal = (-6.2873749481111805,7.299302419349356)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.2873749481111805,7.299302419349356)
goal = (-8.415387783106672,1.8308572355016093)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (5.517765580424518,4.099978412125667)
goal = (6.596582402757683,2.77230517672802)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.596582402757683,2.77230517672802)
goal = (6.156245677338921,0.47890598531465756)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.156245677338921,0.47890598531465756)
goal = (10.249324959369222,0.22164753366257983)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (-1.334479466443332,2.02141118268573)
goal = (-2.583432554504312,0.9695443500531944)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.583432554504312,0.9695443500531944)
goal = (-4.3592781331752075,2.104717419153456)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.3592781331752075,2.104717419153456)
goal = (-4.3643505775053555,-3.05395943085329)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (-0.6694300431115465,8.883611324280842)
goal = (-5.277626698904678,7.676976925445693)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.277626698904678,7.676976925445693)
goal = (-6.632582548660405,6.962700231557992)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.632582548660405,6.962700231557992)
goal = (-9.274526480673728,2.979271770893936)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (2.491842720581136,1.30151636486233)
goal = (3.5540204297506097,0.10687359147809161)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.5540204297506097,0.10687359147809161)
goal = (5.286040494067514,-1.1007076624881584)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.286040494067514,-1.1007076624881584)
goal = (6.985770433387032,-2.3650028425713554)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (-2.532504936580061,4.3062892466625105)
goal = (-3.624868170194346,3.096194452951895)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.624868170194346,3.096194452951895)
goal = (-4.7164082911362675,2.223689548015539)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.7164082911362675,2.223689548015539)
goal = (-7.572052654755071,0.5600112426299582)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (-2.579639242712564,5.492507685501115)
goal = (-5.654563782651015,3.6370786757199447)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.654563782651015,3.6370786757199447)
goal = (-8.352717175726372,1.296401556574244)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (5.282797536573733,7.303181952246437)
goal = (9.016750961812349,6.455750590617974)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.016750961812349,6.455750590617974)
goal = (10.674545389391849,-0.3589922203585232)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (-1.530069412321252,1.907992725744732)
goal = (-4.0737929519253155,0.6022463270830585)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.0737929519253155,0.6022463270830585)
goal = (-4.4016028066266095,-3.200669351822304)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (-1.3772632831244795,8.674454690352253)
goal = (-6.191986927025283,5.517308118513714)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.191986927025283,5.517308118513714)
goal = (-7.983331428927983,0.33970401805550665)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (5.929188408961524,5.3043905665079265)
goal = (10.090072248173023,4.115015366723194)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.090072248173023,4.115015366723194)
goal = (10.187397540051176,-1.5298520588766316)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (-1.9718834136741208,2.3754779316819423)
goal = (-3.995425178413649,-0.2471938721812137)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.995425178413649,-0.2471938721812137)
goal = (-7.161376769171793,-0.4651767325416092)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (4.236702777916532,8.747896925576356)
goal = (9.873334606692335,8.561860904401659)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.873334606692335,8.561860904401659)
goal = (9.504463901426817,-1.9935348716735577)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (-2.4196180975041406,8.53753475039337)
goal = (-7.533017329404123,8.082818048736438)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.533017329404123,8.082818048736438)
goal = (-8.06721285003644,-0.1912110249321426)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (5.464998611586095,7.494343778958942)
goal = (10.086422323730677,8.301970842366718)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.086422323730677,8.301970842366718)
goal = (8.869107732866926,-2.3033057133989967)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (-1.91390513260182,1.7802439638961651)
goal = (-2.9088293886617373,-0.8875699977106759)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.9088293886617373,-0.8875699977106759)
goal = (-3.1638351154945434,-3.923058706644394)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (-3.3008663616936884,5.884584663276014)
goal = (-5.920940433429427,4.105354724017703)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.920940433429427,4.105354724017703)
goal = (-8.325193867686577,-0.2060991208216656)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (6.24914296102857,6.182840074547956)
goal = (10.26736080609318,5.234652224486906)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.26736080609318,5.234652224486906)
goal = (9.678412738037238,-2.0481682935015657)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (-2.2531746975394302,1.4617227551243595)
goal = (-4.111224530756848,-0.5065583024992169)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.111224530756848,-0.5065583024992169)
goal = (-3.4836712027341203,-3.88370238226208)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (-3.1506717803256032,8.193071244745742)
goal = (-8.113736885124805,7.689669002256249)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.113736885124805,7.689669002256249)
goal = (-8.769376479990779,-0.8794260510618859)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (4.134942054385965,0.6585743577200542)
goal = (5.611620158629792,-0.7895851642841585)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.611620158629792,-0.7895851642841585)
goal = (5.024329204424271,-4.437014439722303)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (-2.826404193197545,4.044855013380902)
goal = (-5.487887229031301,2.7947411931338237)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.487887229031301,2.7947411931338237)
goal = (-7.2549894640130255,-1.9307766652946832)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (6.6036659475737025,6.099263313257575)
goal = (10.576481381807447,6.142141429168024)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.576481381807447,6.142141429168024)
goal = (10.220393760565031,-2.174089977164302)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (-2.586489584580514,3.1362203136195905)
goal = (-5.40783139312207,2.064220982830159)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.40783139312207,2.064220982830159)
goal = (-7.9095750994412635,-1.996695044287721)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (6.14010363938481,7.7910107450449395)
goal = (10.598173705711787,6.408720269225002)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.598173705711787,6.408720269225002)
goal = (10.774593860810871,-2.8463996942568945)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (4.665785936327893,8.809354313219487)
goal = (10.319167392099743,4.050782986905594)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.319167392099743,4.050782986905594)
goal = (8.091364655913662,-3.8742151672550857)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (2.7382344445437905,-0.013191495911026863)
goal = (3.935397369300107,-2.3252304182312606)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.935397369300107,-2.3252304182312606)
goal = (4.5674907456867135,-4.541992804401899)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (-4.550670849395055,8.370912496486977)
goal = (-8.22448174035722,9.23822179014283)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.22448174035722,9.23822179014283)
goal = (-9.064885680672534,-1.4389967088244102)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (5.4217815531225035,8.703606228596975)
goal = (10.400163364075159,3.9629060924934834)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.400163364075159,3.9629060924934834)
goal = (8.839784920258992,-3.705965890020316)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (6.6526267671554695,8.251241264020653)
goal = (10.084516808479261,3.6517358913488973)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.084516808479261,3.6517358913488973)
goal = (10.8328884918995,-3.1493426799136497)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (0.9068867248955765,-0.35347463107708776)
goal = (1.5262634238974186,-3.82590918966251)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.5262634238974186,-3.82590918966251)
goal = (2.5510168045224937,-5.234074638864492)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (-3.4302015186321597,5.403154605094205)
goal = (-7.07126840501728,3.55118726447391)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.07126840501728,3.55118726447391)
goal = (-5.763531482269775,-3.6579676496403515)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (6.596582402757683,2.77230517672802)
goal = (8.80992360174372,1.4511069938342018)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.80992360174372,1.4511069938342018)
goal = (7.848410011340242,-4.1824394377490295)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (-2.583432554504312,0.9695443500531944)
goal = (-2.634528945821958,-1.3270199589307232)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.634528945821958,-1.3270199589307232)
goal = (-3.0427298926070563,-4.575838145140198)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (-5.277626698904678,7.676976925445693)
goal = (-8.968123198292346,7.226900952550899)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.968123198292346,7.226900952550899)
goal = (-9.126755680176409,-4.440870460052176)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (3.5540204297506097,0.10687359147809161)
goal = (4.127015194884546,-2.4347572821565047)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.127015194884546,-2.4347572821565047)
goal = (3.732005181959071,-5.221153363772472)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (-3.624868170194346,3.096194452951895)
goal = (-6.766205631992312,2.5342080397775995)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.766205631992312,2.5342080397775995)
goal = (-2.761951783655708,-4.989048781847965)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (5.199732525106224,1.775288303047443)
goal = (5.902110615253791,-1.5075262050089648)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.902110615253791,-1.5075262050089648)
goal = (10.256448190542322,-3.516166907602706)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (0.2912395472122835,-0.36648588068440446)
goal = (-0.011278403682419835,-5.818184892678305)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (6.999414112169948,7.489477215517962)
goal = (10.494920166138144,-3.975928077994088)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (-0.7741052468500413,-0.342464911127875)
goal = (-0.27599453734956647,-6.019843732012843)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (-4.466663850501701,5.953620950270334)
goal = (-7.724253415722748,-4.87135112035662)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (4.958861261307957,0.5991426298331373)
goal = (7.045075951706718,-5.505249159875621)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (-2.582742591674032,0.3787627136532059)
goal = (-5.346730165169319,-5.64176133209086)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (6.654538286257495,8.256941867478705)
goal = (4.988324575939586,-5.860852842698328)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (-4.657037719422262,4.9714020985151794)
goal = (-7.2557537266124195,-5.586273693324597)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (6.969065879835853,7.66961079823443)
goal = (8.414159892951538,-5.899084405885922)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (0.7473979790495768,-1.3576469059804186)
goal = (1.4310009985910561,-5.734575975128864)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (-4.010473517254802,2.474972694037822)
goal = (-6.104456157910031,-5.938512850371888)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (7.177240050365981,3.0535831846922417)
goal = (8.379961279518419,-5.932389667722934)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (-3.7705173654261825,2.1656999163412287)
goal = (-5.8636077685877535,-6.044043511995458)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (-5.277127130201424,4.222058068064696)
goal = (-5.634956717137314,-6.493529724751421)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (5.542630131237841,0.9551168684445397)
goal = (3.843890937980966,-5.971300140224439)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (-5.182278803829787,3.9625188471933015)
goal = (-1.2872161147015948,-6.475998122463432)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (7.87795003429348,4.701796065384713)
goal = (10.564814865824172,-5.531277800590129)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (-4.313136602768431,2.363224205505622)
goal = (-3.655385353605249,-7.179839245442936)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (7.933592268403963,6.003941734878323)
goal = (10.867931652556173,-5.757262400233644)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (7.594973361285607,8.786316379812497)
goal = (9.679315397429992,-6.022923259062932)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (5.112218112518956,-0.7654803602954541)
goal = (5.867431231735598,-6.517097573041513)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (-5.825647895307556,8.561355849338632)
goal = (-6.792489858853112,-7.119389897099504)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (8.368357408020596,6.736141529610633)
goal = (8.193046836741797,-6.372117183402242)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (8.845027637240587,7.612131846421285)
goal = (7.761156057805529,-6.36977601390097)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (3.7595429186834206,-1.882916559834963)
goal = (0.7805341957896363,-5.891343202729827)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (-6.2873749481111805,7.299302419349356)
goal = (-3.7566069962781388,-7.250172607579919)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (6.156245677338921,0.47890598531465756)
goal = (6.178740472734347,-6.475298261397517)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (-4.3592781331752075,2.104717419153456)
goal = (-1.4606898792279885,-7.222807573684749)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (-6.632582548660405,6.962700231557992)
goal = (-8.45255485736556,-7.349072418091281)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (5.286040494067514,-1.1007076624881584)
goal = (7.278549469596641,-7.019982117000328)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (-4.7164082911362675,2.223689548015539)
goal = (-3.582034381092905,-7.701485092852547)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (-5.654563782651015,3.6370786757199447)
goal = (-6.478215937054184,-7.899934870354332)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (9.016750961812349,6.455750590617974)
goal = (9.903934098005678,-6.693057421432028)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (-4.0737929519253155,0.6022463270830585)
goal = (-3.524432883239469,-7.8460664870235295)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (-6.191986927025283,5.517308118513714)
goal = (-5.5597013152633155,-8.007932661861712)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (10.090072248173023,4.115015366723194)
goal = (7.807062347801303,-7.1136633690072735)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (-3.995425178413649,-0.2471938721812137)
goal = (-1.1474916164419398,-7.487225153053457)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (9.873334606692335,8.561860904401659)
goal = (7.255774490777579,-7.328680961180229)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (-7.533017329404123,8.082818048736438)
goal = (-9.129786105552347,-7.922393794323429)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (10.086422323730677,8.301970842366718)
goal = (6.582788313323576,-7.792193751319104)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (-2.9088293886617373,-0.8875699977106759)
goal = (0.15804230077833914,-7.279493117611356)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-5.920940433429427,4.105354724017703)
goal = (-6.175949054042048,-8.740413967234485)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (10.26736080609318,5.234652224486906)
goal = (8.239769937977709,-8.244353683342872)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-4.111224530756848,-0.5065583024992169)
goal = (-0.46129741181112927,-7.5840290329055495)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (-8.113736885124805,7.689669002256249)
goal = (-5.0323268815925735,-8.742108831355162)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (5.611620158629792,-0.7895851642841585)
goal = (4.603162583383833,-7.556174749560892)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (-5.487887229031301,2.7947411931338237)
goal = (-2.4149223630412076,-8.387315470683012)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (10.576481381807447,6.142141429168024)
goal = (7.090459294469998,-8.065975030708596)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (-5.40783139312207,2.064220982830159)
goal = (0.45611211929429274,-8.14768359440375)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (10.598173705711787,6.408720269225002)
goal = (6.824342076944177,-8.056696376654136)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (10.319167392099743,4.050782986905594)
goal = (6.513967115098781,-7.985479633218405)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (3.935397369300107,-2.3252304182312606)
goal = (4.938380938012386,-8.091736968406899)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (-8.22448174035722,9.23822179014283)
goal = (0.8635655842362553,-7.995440676644564)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (10.400163364075159,3.9629060924934834)
goal = (4.882460742413722,-8.345562554669536)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (10.084516808479261,3.6517358913488973)
goal = (6.431014742764788,-8.961054750892774)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (1.5262634238974186,-3.82590918966251)
goal = (1.4025464422519143,-7.945857450302803)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-7.07126840501728,3.55118726447391)
goal = (1.0866958513307416,-8.622081610676572)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (8.80992360174372,1.4511069938342018)
goal = (6.0493397388846155,-8.94828408573702)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (-2.634528945821958,-1.3270199589307232)
goal = (3.513266813623945,-8.63020983098394)
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
f = open('smo2sol-19.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
