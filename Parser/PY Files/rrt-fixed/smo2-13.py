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

obstacleList = [[(0.004232150203032415,0.04459721786188164),(1.8196938096302873,-0.7945088321399274),(2.6587998596320954,1.020952827287329)],[(0.3385812525719565,0.6904217331200425),(2.263009405197085,1.2350099127418868),(1.9907153153861625,2.197223989054453),(1.0285012390735981,1.92492989924353),(0.4839130594517493,3.8493580518686583),(-0.47830101686081306,3.5770639620577342),(-0.20600692704988988,2.6148498857451705),(-1.1682210033624538,2.342555795934248),(0.0662871627610333,1.6526358094326064)],[(0.03391759929418715,0.127005175208054),(-1.7185670466322547,1.090746616566717),(-2.2004377673115862,0.2145042936034961),(-1.3241954443483654,-0.26736642707583536),(-2.2879368857070284,-2.0198510730022776),(-1.411694562743809,-2.5017217936816083),(-0.9298238420644769,-1.6254794707183873),(-0.05358151910125411,-2.1073501913977197),(-0.44795312138514487,-0.7492371477551667)],[(-1.4016229600372647,1.0320949432621238),(-1.4164915973788268,3.0320396734043356),(-3.416436327521036,3.017171036062776)],[(-2.5376103772497127,0.3249662740523913),(-2.7198047849349516,2.3166502845842385),(-3.7156467902008776,2.225553080741618),(-3.624549586358256,1.2297110754756955),(-5.616233596890105,1.047516667790453),(-5.525136393047484,0.05167466252453229),(-4.52929438778156,0.14277186636715022),(-4.438197183938939,-0.8530701388987736),(-3.533452382515636,0.23386907020976988)],[(2.5516383294373863,-0.34306688577535194),(1.7609749766398695,-2.1801437656426543),(2.679513416573519,-2.5754754420414154),(3.074845092972284,-1.6569370021077607),(4.911921972839593,-2.4476003549052705),(5.307253649238348,-1.5290619149716163),(4.388715209304692,-1.1337302385728611),(4.784046885703449,-0.2151917986392092),(3.4701767693710392,-0.7383985621741083)],[(-1.651740921800295,5.131228706082034),(0.12424951594335676,4.21152333747581),(0.5841022002464684,5.0995185563476335),(-0.3038930186253559,5.559371240650746),(0.6158123499808696,7.335361678394399),(-0.2721828688909529,7.795214362697509),(-0.7320355531940658,6.907219143825683),(-1.6200307720658942,7.367071828128799),(-1.1918882374971826,6.019223924953859)],[(-2.0967495973249966,-3.7146026091236743),(-2.6747741876641635,-1.799951838627073),(-4.589424958160768,-2.3779764289662335)],[(2.472181909450933,-4.038070421494398),(1.6338836765316493,-2.222235607232813),(0.7259662694008573,-2.641384723692453),(1.1451153858604988,-3.5493021308232486),(-0.6707194284010876,-4.387600363742531),(-0.2515703119414452,-5.295517770873324),(0.6563470951893489,-4.876368654413681),(1.0754962116489908,-5.784286061544476),(1.5642645023201411,-4.457219537954041)],[(6.680004448873314,3.09342019387293),(4.74692671457299,3.606460614456264),(4.490406504281321,2.639921747306105),(5.456945371431482,2.38340153701444),(4.943904950848148,0.4503238027141121),(5.910443817998312,0.1938035924224435),(6.166964028289979,1.160342459572607),(7.13350289544014,0.9038222492809371),(6.423484238581647,2.126881326722769)],[(0.7784907609618162,3.0385546255978078),(2.4256968663121947,1.9042213207458194),(2.992863518738189,2.7278243734210066),(2.1692604660630006,3.294991025847004),(3.30359377091499,4.942197131197382),(2.479990718239807,5.509363783623373),(1.9128240658138065,4.685760730948186),(1.0892210131386157,5.252927383374182),(1.345657413387813,3.8621576782729954)],[(-0.2584650074085925,-9.319982950033557),(1.7324083485268291,-9.129134107683335),(1.6369839273517224,-8.133697429715623),(0.6415472493840104,-8.229121850890735),(0.4506984070337859,-6.238248494955306),(-0.5447382709339257,-6.3336729161304195),(-0.44931384975881433,-7.329109594098135),(-1.4447505277265267,-7.424534015273242),(-0.3538894285837021,-8.324546272065847)],[(7.942711223859192,3.35703101478331),(9.736878690214418,4.240752182727395),(9.295018106242381,5.137835915905009),(8.397934373064764,4.695975331932967),(7.51421320512068,6.490142798288196),(6.617129471943074,6.048282214316157),(7.0589900559151095,5.15119848113854),(6.161906322737494,4.709337897166499),(7.500850639887151,4.254114747960924)],[(-6.162056742706499,1.6711416366260623),(-5.329064263738479,3.489416514080708),(-6.238201702465804,3.905912753564719),(-6.654697941949813,2.9967753148373957),(-8.472972819404454,3.8297677938054226),(-8.889469058888468,2.920630355078096),(-7.980331620161143,2.504134115594084),(-8.396827859645157,1.5949966768667567),(-7.071194181433822,2.08763787611007)],[(1.0836644294535656,-6.474417530991257),(2.1622137996135633,-8.158677385089627),(3.004343726662756,-7.619402700009625),(2.4650690415827516,-6.777272772960438),(4.149328895681121,-5.69872340280044),(3.610054210601123,-4.856593475751257),(2.7679242835519355,-5.395868160831259),(2.2286495984719323,-4.553738233782067),(1.9257943565027504,-5.935142845911257)]]
rand = (-10,11)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (-0.8580438984399503,-3.357740512529885)
goal = (-1.1635846952135704,-3.5898275659207988)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.1635846952135704,-3.5898275659207988)
goal = (-1.151708430245363,-4.068838032266729)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.151708430245363,-4.068838032266729)
goal = (-1.1825508798995337,-2.5005323490505926)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.1825508798995337,-2.5005323490505926)
goal = (0.1671594651323609,-1.688730280478878)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.1671594651323609,-1.688730280478878)
goal = (1.7685783306119323,-2.008242291999678)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.7685783306119323,-2.008242291999678)
goal = (-5.511980412914461,-2.306917974540763)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.511980412914461,-2.306917974540763)
goal = (4.425348059845485,-7.352518194742693)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (-1.1635846952135704,-3.5898275659207988)
goal = (-1.3273884337995066,-3.5129484396629485)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.3273884337995066,-3.5129484396629485)
goal = (-2.067291367416086,-3.824830108173834)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.067291367416086,-3.824830108173834)
goal = (-0.6851138333838183,-2.3854347593014102)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.6851138333838183,-2.3854347593014102)
goal = (0.4195386169528934,-1.8470724182157312)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.4195386169528934,-1.8470724182157312)
goal = (1.9522558819236053,-4.796278761457063)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.9522558819236053,-4.796278761457063)
goal = (-5.5114926690697,-1.9591068610653881)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.5114926690697,-1.9591068610653881)
goal = (-7.652607449008947,-5.184195900755515)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (-1.3273884337995066,-3.5129484396629485)
goal = (-2.192753691840162,-3.3398371780200327)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.192753691840162,-3.3398371780200327)
goal = (-2.055064928443266,-2.3260138089150875)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.055064928443266,-2.3260138089150875)
goal = (1.1702978376572553,-2.30540542804788)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1702978376572553,-2.30540542804788)
goal = (-4.489073754399347,-2.1023821787415864)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.489073754399347,-2.1023821787415864)
goal = (-5.936049140919231,-2.1795728272208104)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.936049140919231,-2.1795728272208104)
goal = (-8.003329773281935,-4.10269750059067)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (-1.151708430245363,-4.068838032266729)
goal = (-0.9379645510362025,-4.538877013337544)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.9379645510362025,-4.538877013337544)
goal = (-1.3111718903978797,-5.332337011970395)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.3111718903978797,-5.332337011970395)
goal = (-2.204667965989315,-6.465208196711574)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.204667965989315,-6.465208196711574)
goal = (1.4765086552027231,-5.984604991397574)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.4765086552027231,-5.984604991397574)
goal = (3.922005227590045,-4.498230559583743)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.922005227590045,-4.498230559583743)
goal = (4.6159126739018514,-7.325702891718207)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (-2.067291367416086,-3.824830108173834)
goal = (-2.4602246148480607,-5.226567269213852)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.4602246148480607,-5.226567269213852)
goal = (-2.800651518673975,-6.207230313705585)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.800651518673975,-6.207230313705585)
goal = (-5.006303688986741,-5.602865844555183)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.006303688986741,-5.602865844555183)
goal = (-6.428539190490298,-4.0706098821726835)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.428539190490298,-4.0706098821726835)
goal = (-8.051268902880258,-3.4594660321648636)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (-2.192753691840162,-3.3398371780200327)
goal = (-2.460901980983367,-1.5171049939677008)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.460901980983367,-1.5171049939677008)
goal = (-3.748982840181128,-1.385743743887578)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.748982840181128,-1.385743743887578)
goal = (-5.181986463634818,-1.7848282018600115)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.181986463634818,-1.7848282018600115)
goal = (-6.167392603289603,-1.084149666157824)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.167392603289603,-1.084149666157824)
goal = (-8.689801723779986,-3.7130284916067113)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (-0.9379645510362025,-4.538877013337544)
goal = (-1.2355160029775725,-5.75289767028363)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.2355160029775725,-5.75289767028363)
goal = (-2.211789613445114,-6.487129356251088)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.211789613445114,-6.487129356251088)
goal = (2.443595925586523,-4.291199350442926)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.443595925586523,-4.291199350442926)
goal = (3.6265583332023983,-6.397162099502733)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.6265583332023983,-6.397162099502733)
goal = (4.070379655567223,-9.106123765410308)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (-1.1825508798995337,-2.5005323490505926)
goal = (-0.8380140453137006,-1.8401429104525526)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8380140453137006,-1.8401429104525526)
goal = (1.1773937816961357,-2.08176920508855)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1773937816961357,-2.08176920508855)
goal = (-1.0471573025374195,0.8374383949995376)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.0471573025374195,0.8374383949995376)
goal = (-0.9094700315640809,1.686890180054661)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.9094700315640809,1.686890180054661)
goal = (-6.781067693238834,1.7490298966111766)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (-0.6851138333838183,-2.3854347593014102)
goal = (1.024065925375984,-1.1715711492244925)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.024065925375984,-1.1715711492244925)
goal = (2.340109650602768,-2.8335176651165774)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.340109650602768,-2.8335176651165774)
goal = (1.2851684282700546,2.3717195881660516)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.2851684282700546,2.3717195881660516)
goal = (4.7665745674758195,2.4532985459054917)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (-2.055064928443266,-2.3260138089150875)
goal = (-3.7382024432068457,-1.0519726889354963)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.7382024432068457,-1.0519726889354963)
goal = (-3.2801340244194135,0.19380119592651823)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.2801340244194135,0.19380119592651823)
goal = (-6.1471375695340384,0.22412145050908805)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.1471375695340384,0.22412145050908805)
goal = (-7.816226744610139,0.10307365912692745)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (-1.3111718903978797,-5.332337011970395)
goal = (-1.4510639557749565,-7.710546847223878)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.4510639557749565,-7.710546847223878)
goal = (0.87426838397867,-7.531862512539048)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.87426838397867,-7.531862512539048)
goal = (-3.608350186705634,-9.112263235315524)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.608350186705634,-9.112263235315524)
goal = (5.306453063363113,-7.706208764615878)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (-2.4602246148480607,-5.226567269213852)
goal = (-4.276129897494847,-5.96809380215531)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.276129897494847,-5.96809380215531)
goal = (-4.52112970435418,-6.3855955689248844)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.52112970435418,-6.3855955689248844)
goal = (-5.887260969385885,-7.487856982789789)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.887260969385885,-7.487856982789789)
goal = (-8.72545850929759,-7.4106041265525135)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (-2.460901980983367,-1.5171049939677008)
goal = (-3.490586693561421,-0.5285356444851921)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.490586693561421,-0.5285356444851921)
goal = (-4.8672987807955295,-0.41100719685886666)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.8672987807955295,-0.41100719685886666)
goal = (-3.6433039463440453,2.389270532974761)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.6433039463440453,2.389270532974761)
goal = (-7.497750029431231,1.2557335724412173)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (-1.2355160029775725,-5.75289767028363)
goal = (1.2174114114263705,-6.067494800919842)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.2174114114263705,-6.067494800919842)
goal = (-1.0998699302264212,-8.737661709501296)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.0998699302264212,-8.737661709501296)
goal = (-3.587243773768325,-9.18579274246832)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.587243773768325,-9.18579274246832)
goal = (5.151152089047875,-8.97664722448345)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (-0.8380140453137006,-1.8401429104525526)
goal = (0.5724307126485826,-0.4855944457024943)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.5724307126485826,-0.4855944457024943)
goal = (-0.7132551906609574,0.8137982478184558)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.7132551906609574,0.8137982478184558)
goal = (-1.1947275926569327,3.3601316951024565)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.1947275926569327,3.3601316951024565)
goal = (4.217608873386167,3.017633161790423)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (0.1671594651323609,-1.688730280478878)
goal = (1.4036842411447452,-0.7937293499601328)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.4036842411447452,-0.7937293499601328)
goal = (2.7693332814197174,-2.8160999045815984)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.7693332814197174,-2.8160999045815984)
goal = (5.201906849942015,-0.7541677286593931)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.201906849942015,-0.7541677286593931)
goal = (3.1511956417363827,4.039393037840696)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (0.4195386169528934,-1.8470724182157312)
goal = (2.8503759567072837,0.17028856038297135)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.8503759567072837,0.17028856038297135)
goal = (4.9823111130900415,-3.6058529133367108)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.9823111130900415,-3.6058529133367108)
goal = (3.8283058505379692,3.7275736103032795)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (1.1702978376572553,-2.30540542804788)
goal = (3.1500789226681913,-3.7753075947318298)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.1500789226681913,-3.7753075947318298)
goal = (5.333599449811196,-2.035497584438251)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.333599449811196,-2.035497584438251)
goal = (6.998071807206982,-3.9170074323033246)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (-2.204667965989315,-6.465208196711574)
goal = (-2.997439552637392,-8.18437308162134)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.997439552637392,-8.18437308162134)
goal = (-5.630305569908709,-7.866038116859197)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.630305569908709,-7.866038116859197)
goal = (-8.606192018199248,-8.778956264907599)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (-2.800651518673975,-6.207230313705585)
goal = (-3.7356844080094245,-7.9888623019170115)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.7356844080094245,-7.9888623019170115)
goal = (-4.978186738664491,-8.952607305812705)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.978186738664491,-8.952607305812705)
goal = (-8.888535011937009,-9.16210229124063)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (-3.748982840181128,-1.385743743887578)
goal = (-5.4235247591902525,-0.6286909858304899)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.4235247591902525,-0.6286909858304899)
goal = (-6.624143029569923,-0.5550636699639817)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.624143029569923,-0.5550636699639817)
goal = (-8.324931927970885,-0.7182773122275989)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (-2.211789613445114,-6.487129356251088)
goal = (-2.0029122072925833,-9.232489653116303)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.0029122072925833,-9.232489653116303)
goal = (-4.825811159841928,-9.171907318167944)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.825811159841928,-9.171907318167944)
goal = (5.9818727752386085,-6.04939050241653)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (1.1773937816961357,-2.08176920508855)
goal = (2.622434703962881,-4.490716734729734)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.622434703962881,-4.490716734729734)
goal = (4.4671120137447,-4.993675185135187)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.4671120137447,-4.993675185135187)
goal = (7.1162218046308165,-0.2492674220058202)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (1.024065925375984,-1.1715711492244925)
goal = (3.6271015323987363,0.007229474884438147)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.6271015323987363,0.007229474884438147)
goal = (4.075328448391327,1.8448516354802909)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.075328448391327,1.8448516354802909)
goal = (1.0690164524351324,4.936696617347238)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (-3.7382024432068457,-1.0519726889354963)
goal = (-5.60713801693433,-1.501993028457254)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.60713801693433,-1.501993028457254)
goal = (-5.5101157599487705,1.744849739415633)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.5101157599487705,1.744849739415633)
goal = (-8.598446512938992,-1.1063539901435444)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (-1.4510639557749565,-7.710546847223878)
goal = (-0.9811401436397196,-9.280350799590511)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.9811401436397196,-9.280350799590511)
goal = (1.9587767983409723,-8.772215882076527)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.9587767983409723,-8.772215882076527)
goal = (6.1247948738718385,-8.32115176114197)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (-4.276129897494847,-5.96809380215531)
goal = (-6.185868925112539,-6.312346849958316)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.185868925112539,-6.312346849958316)
goal = (-6.785846741723738,-6.113099998493205)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.785846741723738,-6.113099998493205)
goal = (-8.556181248100419,0.11274894622156673)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (-3.490586693561421,-0.5285356444851921)
goal = (-4.953815092285504,1.4486205836061021)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.953815092285504,1.4486205836061021)
goal = (-4.111366863857088,2.668382604213491)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.111366863857088,2.668382604213491)
goal = (-5.419057447737639,4.247524131328433)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (1.2174114114263705,-6.067494800919842)
goal = (3.1594362898832316,-7.324915735285904)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.1594362898832316,-7.324915735285904)
goal = (2.759167094100807,-8.563843848065302)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.759167094100807,-8.563843848065302)
goal = (6.734223513534149,-5.210989831033049)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (0.5724307126485826,-0.4855944457024943)
goal = (-1.3980882627215765,1.0938839307333996)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.3980882627215765,1.0938839307333996)
goal = (2.9171821398486326,3.1169002216830943)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.9171821398486326,3.1169002216830943)
goal = (2.0290682621777734,5.164118668281825)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (1.4036842411447452,-0.7937293499601328)
goal = (2.6794120971320936,1.4238230777947152)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.6794120971320936,1.4238230777947152)
goal = (5.32637348441877,-0.5178409530233843)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.32637348441877,-0.5178409530233843)
goal = (4.209320368143862,4.292171144144266)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (1.7685783306119323,-2.008242291999678)
goal = (3.9252870237280124,0.2074038215184668)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.9252870237280124,0.2074038215184668)
goal = (4.723745471606126,-4.976167142584407)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.723745471606126,-4.976167142584407)
goal = (7.306704315623593,-0.99027778110821)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (1.9522558819236053,-4.796278761457063)
goal = (4.242613772039517,-6.086575332246076)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.242613772039517,-6.086575332246076)
goal = (6.822097471590395,-6.091180910822921)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (-4.489073754399347,-2.1023821787415864)
goal = (-6.9558403539646525,-3.7177128595450197)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.9558403539646525,-3.7177128595450197)
goal = (-8.555123652457542,0.9792800381389384)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (1.4765086552027231,-5.984604991397574)
goal = (4.139711989616675,-6.401813468464461)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.139711989616675,-6.401813468464461)
goal = (6.907684604921165,-6.967475602647443)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (-5.006303688986741,-5.602865844555183)
goal = (-6.652123724715143,-7.284117050258393)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.652123724715143,-7.284117050258393)
goal = (-7.075140701993142,3.9312016768867366)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (-5.181986463634818,-1.7848282018600115)
goal = (-7.474788254483228,-2.3414449179111294)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.474788254483228,-2.3414449179111294)
goal = (-6.1603667505731945,4.172890473189874)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (2.443595925586523,-4.291199350442926)
goal = (4.874732848267307,-4.864548107367358)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.874732848267307,-4.864548107367358)
goal = (7.138642573667072,-5.1273279121614435)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (-1.0471573025374195,0.8374383949995376)
goal = (-3.367970514653736,2.958119972295721)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.367970514653736,2.958119972295721)
goal = (-2.1749979216362867,5.487070014529147)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (2.340109650602768,-2.8335176651165774)
goal = (4.818080953281738,-5.903411519417958)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.818080953281738,-5.903411519417958)
goal = (7.486232393704052,-4.565058961324085)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (-3.2801340244194135,0.19380119592651823)
goal = (-4.570446838210691,3.3695806277095297)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.570446838210691,3.3695806277095297)
goal = (-4.874183782318513,4.54002178040991)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (0.87426838397867,-7.531862512539048)
goal = (3.2200516808648576,-8.43320072745552)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.2200516808648576,-8.43320072745552)
goal = (7.01460204488254,-7.27387679388387)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (-4.52112970435418,-6.3855955689248844)
goal = (-7.278277937285532,-6.654879773632245)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.278277937285532,-6.654879773632245)
goal = (-6.218422327317186,4.257687100027821)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (-4.8672987807955295,-0.41100719685886666)
goal = (-7.357185891105733,-0.5118521794238617)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.357185891105733,-0.5118521794238617)
goal = (-5.484524215326099,4.456795213352532)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (-1.0998699302264212,-8.737661709501296)
goal = (-5.164520608695299,-9.160733758565708)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.164520608695299,-9.160733758565708)
goal = (6.9665982807913736,-9.280098144412365)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (-0.7132551906609574,0.8137982478184558)
goal = (-1.120305089659559,4.447448009455803)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.120305089659559,4.447448009455803)
goal = (-1.693129528447595,6.0772782678573005)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (2.7693332814197174,-2.8160999045815984)
goal = (6.464567192606475,-2.6340314190295553)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.464567192606475,-2.6340314190295553)
goal = (7.8069456228195815,-3.0171110058720805)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (2.8503759567072837,0.17028856038297135)
goal = (4.910830522302117,1.5634245318412)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.910830522302117,1.5634245318412)
goal = (4.521284290851581,4.12881300000563)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (3.1500789226681913,-3.7753075947318298)
goal = (6.436895159972927,-3.799503283762628)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.436895159972927,-3.799503283762628)
goal = (7.789557012563831,-3.8542345376579785)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (-2.997439552637392,-8.18437308162134)
goal = (-7.841263290397095,-8.805799893487357)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.841263290397095,-8.805799893487357)
goal = (7.964413729631483,-7.227559794175167)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (-3.7356844080094245,-7.9888623019170115)
goal = (-8.044593585099182,-8.604072903116311)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.044593585099182,-8.604072903116311)
goal = (8.26476599340281,-7.688137188508067)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (-5.4235247591902525,-0.6286909858304899)
goal = (-7.536467253933738,-0.006252377526255515)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.536467253933738,-0.006252377526255515)
goal = (-6.774868147156573,4.19712300700931)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (-2.0029122072925833,-9.232489653116303)
goal = (3.171757581795587,-8.82175601713547)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.171757581795587,-8.82175601713547)
goal = (8.58836643469474,-7.41352747001754)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (2.622434703962881,-4.490716734729734)
goal = (5.565921276667273,-6.043374952163418)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.565921276667273,-6.043374952163418)
goal = (7.939774706914248,-3.1825867844313054)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (3.6271015323987363,0.007229474884438147)
goal = (5.1451317325826,1.2303384344565327)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.1451317325826,1.2303384344565327)
goal = (7.157250296215393,0.16383424489621845)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (-5.60713801693433,-1.501993028457254)
goal = (-7.366759396929873,-3.3472472519242347)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.366759396929873,-3.3472472519242347)
goal = (-7.472971263025624,4.10289867017992)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (-0.9811401436397196,-9.280350799590511)
goal = (3.3028923451831194,-8.51335167726649)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.3028923451831194,-8.51335167726649)
goal = (8.572838635549543,-5.8141491843526705)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (-6.185868925112539,-6.312346849958316)
goal = (-7.592957294905236,-5.198951796562409)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.592957294905236,-5.198951796562409)
goal = (-5.689798332494927,4.427416792250755)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (-4.953815092285504,1.4486205836061021)
goal = (-5.14900763016008,3.2600021390056906)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.14900763016008,3.2600021390056906)
goal = (-5.421146604544675,4.475357452336455)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (3.1594362898832316,-7.324915735285904)
goal = (4.4104035634835235,-7.3873825066688035)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.4104035634835235,-7.3873825066688035)
goal = (8.847544906006787,-5.8105809445287555)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (-1.3980882627215765,1.0938839307333996)
goal = (0.8688612338358048,4.799518155392368)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.8688612338358048,4.799518155392368)
goal = (-3.8559429871255864,5.6223895367744205)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (2.6794120971320936,1.4238230777947152)
goal = (4.31569653607027,2.564683141093832)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.31569653607027,2.564683141093832)
goal = (3.281584640790486,4.908081701410731)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (3.9252870237280124,0.2074038215184668)
goal = (5.787296237619735,-0.06023711970799006)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.787296237619735,-0.06023711970799006)
goal = (7.489240484369928,0.16602647993173747)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (-5.511980412914461,-2.306917974540763)
goal = (-8.246764956525967,-2.7566943333647966)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.246764956525967,-2.7566943333647966)
goal = (-5.297611041754197,5.1781825704625355)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (-5.5114926690697,-1.9591068610653881)
goal = (-6.920714225239046,5.8706672703783465)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (-5.936049140919231,-2.1795728272208104)
goal = (-8.480667593634553,5.448565092009771)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (3.922005227590045,-4.498230559583743)
goal = (8.800494268729592,-4.569172785633069)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (-6.428539190490298,-4.0706098821726835)
goal = (-8.090367262652146,5.792859145927734)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (-6.167392603289603,-1.084149666157824)
goal = (-5.572240124087333,6.035513428122453)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (3.6265583332023983,-6.397162099502733)
goal = (9.507778034589846,-5.894879170946911)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (-0.9094700315640809,1.686890180054661)
goal = (1.6755970304460757,5.830179381371877)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (1.2851684282700546,2.3717195881660516)
goal = (2.4143419255208283,5.4919793092463856)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (-6.1471375695340384,0.22412145050908805)
goal = (-6.366059858736719,6.213849439416718)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (-3.608350186705634,-9.112263235315524)
goal = (9.692554337295244,-8.974283949940567)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (-5.887260969385885,-7.487856982789789)
goal = (-7.844831869763199,6.0009915565502325)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (-3.6433039463440453,2.389270532974761)
goal = (-2.2647560902319332,6.523411733910791)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (-3.587243773768325,-9.18579274246832)
goal = (8.555372080673544,-2.275887716381847)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (-1.1947275926569327,3.3601316951024565)
goal = (-0.9012351601173547,7.222508471830361)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (5.201906849942015,-0.7541677286593931)
goal = (7.473370935370015,1.3830093829789476)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (4.9823111130900415,-3.6058529133367108)
goal = (8.933782203621535,-2.2229023944370265)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (5.333599449811196,-2.035497584438251)
goal = (8.504367817687298,-0.521527853572124)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (-5.630305569908709,-7.866038116859197)
goal = (-6.290576712754742,6.7908942517958995)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (-4.978186738664491,-8.952607305812705)
goal = (9.584657183872851,-4.450300992396889)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (-6.624143029569923,-0.5550636699639817)
goal = (-4.42484037442812,6.867500659142836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (-4.825811159841928,-9.171907318167944)
goal = (9.103990653394115,-1.4694080946213584)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (4.4671120137447,-4.993675185135187)
goal = (9.163663536960511,-1.341437727764685)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (4.075328448391327,1.8448516354802909)
goal = (5.562013268365694,3.624913656579068)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (-5.5101157599487705,1.744849739415633)
goal = (-8.167724422777436,7.08205972382599)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (1.9587767983409723,-8.772215882076527)
goal = (8.258134547037917,-0.08275708181715302)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (-6.785846741723738,-6.113099998493205)
goal = (-7.460873130135322,7.438462664097974)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (-4.111366863857088,2.668382604213491)
goal = (-3.7313541967233483,7.453232050917618)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (2.759167094100807,-8.563843848065302)
goal = (7.775699184800883,1.1907454125295498)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (2.9171821398486326,3.1169002216830943)
goal = (3.956416911886718,4.727938429859538)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (5.32637348441877,-0.5178409530233843)
goal = (7.422930558131037,1.905060916682487)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (4.723745471606126,-4.976167142584407)
goal = (8.192447775841702,1.7405699722723913)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (4.242613772039517,-6.086575332246076)
goal = (8.627601997717823,2.0118080728950893)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (-6.9558403539646525,-3.7177128595450197)
goal = (-0.23304728509546813,7.77587896597519)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (4.139711989616675,-6.401813468464461)
goal = (9.078466144598279,2.185947655603174)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (-6.652123724715143,-7.284117050258393)
goal = (4.580610561431682,4.751658683802036)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (-7.474788254483228,-2.3414449179111294)
goal = (1.7058295852968346,6.985679880945421)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (4.874732848267307,-4.864548107367358)
goal = (9.040880845002347,2.3463866596998653)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (-3.367970514653736,2.958119972295721)
goal = (1.9441406583323086,7.009069156278068)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (4.818080953281738,-5.903411519417958)
goal = (5.868315001519681,3.427557367327344)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (-4.570446838210691,3.3695806277095297)
goal = (1.8402032501626682,7.675603396809612)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (3.2200516808648576,-8.43320072745552)
goal = (6.308491884704425,3.783572446141328)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-7.278277937285532,-6.654879773632245)
goal = (2.953879259990371,6.254094231300012)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (-7.357185891105733,-0.5118521794238617)
goal = (4.24754821805392,5.6667035935789585)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-5.164520608695299,-9.160733758565708)
goal = (5.504745453989132,3.974810127515987)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (-1.120305089659559,4.447448009455803)
goal = (3.5848311397171866,7.150922118334561)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (6.464567192606475,-2.6340314190295553)
goal = (9.088795825550571,2.9593070983315304)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (4.910830522302117,1.5634245318412)
goal = (5.716782612952507,4.318713603637887)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (6.436895159972927,-3.799503283762628)
goal = (5.698083622003505,4.328055551523432)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (-7.841263290397095,-8.805799893487357)
goal = (4.415513651988137,5.47577810851819)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (-8.044593585099182,-8.604072903116311)
goal = (4.667511046281323,5.650312839476031)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (-7.536467253933738,-0.006252377526255515)
goal = (3.588203861074067,7.255809176965739)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (3.171757581795587,-8.82175601713547)
goal = (5.477113564085371,5.3493875284948995)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (5.565921276667273,-6.043374952163418)
goal = (5.923572383713731,5.438050219210286)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (5.1451317325826,1.2303384344565327)
goal = (5.006144650729473,5.659510740995161)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (-7.366759396929873,-3.3472472519242347)
goal = (5.756796345898616,6.388396033697068)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (3.3028923451831194,-8.51335167726649)
goal = (6.0211812226846515,6.127749104701664)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-7.592957294905236,-5.198951796562409)
goal = (4.9324038975118505,7.434057437879707)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (-5.14900763016008,3.2600021390056906)
goal = (7.034275070452434,6.886177585412584)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (4.4104035634835235,-7.3873825066688035)
goal = (8.517014358680738,6.036228383213018)
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
f = open('smo2sol-13.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
