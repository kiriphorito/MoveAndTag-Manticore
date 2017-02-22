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

obstacleList = [[(0.008639777777150031,-0.003523847943305239),(0.405744656439321,-1.9637043792573734),(1.3858349220963553,-1.7651519399262878),(1.1872824827652697,-0.785061674269254),(3.147463014079338,-0.3879567956070829),(2.9489105747482522,0.592133470049951),(1.9688203090912182,0.39358103071886563),(1.7702678697601326,1.3736712963759),(0.9887300434341841,0.1950285913877802)],[(-2.141242624317434,0.8621636918111755),(-1.003424746408352,-0.7826373529083592),(-0.18102422404858354,-0.21372841395381892),(-0.7499331630031252,0.6086721084059501),(0.8948678817164093,1.7464899863150327),(0.32595894276186677,2.5688905086748006),(-0.49644157959790025,1.9999815697202583),(-1.0653505185524426,2.8223820920800256),(-1.3188421019576673,1.4310726307657173)],[(1.5142204930906271,-3.4891461428595165),(-0.2615735449444232,-2.5690616171342073),(-0.72161580780708,-3.4569586361517364),(0.1662812112104468,-3.91700089901439),(-0.7538033145148553,-5.692794937049446),(0.13409370450267155,-6.152837199912097),(0.5941359673653227,-5.264940180894568),(1.4820329863828507,-5.724982443757224),(1.0541782302279747,-4.377043161877043)],[(-0.3350604642280523,-5.977774048648444),(-2.0491720204591823,-7.008221316971714),(-1.5339483862975511,-7.865277095087279),(-0.6768926081819845,-7.350053460925644),(0.35355466014128706,-9.064165017156778),(1.2106104382568572,-8.548941382995139),(0.6953868040952165,-7.691885604879574),(1.5524425822107841,-7.176661970717941),(0.1801631699335821,-6.834829826764008)],[(2.8286791566458733,4.311358012438656),(2.771161440133697,6.310530769427712),(1.7715750616391637,6.281771911171621),(1.800333919895257,5.282185532677096),(-0.19883883709380168,5.2246678161649145),(-0.17007997883771164,4.225081437670384),(0.8295063996568188,4.253840295926473),(0.8582652579129078,3.254253917431944),(1.8290927781513457,4.282599154182564)],[(4.149378100570911,3.0509925924272556),(6.1493000979384975,3.033328944863941),(6.158131921720158,4.033289943547732),(5.158170923036362,4.042121767329392),(5.175834570599678,6.042043764696977),(4.1758735719158855,6.050875588478636),(4.167041748134229,5.050914589794842),(3.167080749450433,5.059746413576502),(4.15820992435257,4.050953591111049)],[(-1.8565384333362631,0.12376841002754693),(-3.8009727845069965,0.5919297646770384),(-4.035053461831744,-0.38028741090832063),(-3.0628362862463767,-0.6143680882330731),(-3.5309976408958703,-2.5588024394038067),(-2.5587804653105057,-2.7928831167285524),(-2.3246997879857547,-1.8206659411431856),(-1.35248261240039,-2.054746618467932),(-2.0906191106610086,-0.8484487655578193)],[(4.395653420290096,-2.570497712367599),(4.62677061651562,-4.557099041675734),(5.620071281169682,-4.441540443562976),(5.504512683056924,-3.448239778908908),(7.4911140123650615,-3.217122582683389),(7.3755554142523,-2.223821918029316),(6.382254749598231,-2.3393805161420773),(6.266696151485474,-1.346079851488011),(5.388954084944163,-2.4549391142548385)],[(-6.63419980530169,0.49866303967861425),(-7.722759416639691,-1.1791444500423156),(-6.883855671779224,-1.7234242557113202),(-6.339575866110226,-0.8845205108508529),(-4.6617683763892925,-1.9730801221888563),(-4.117488570720291,-1.1341763773283902),(-4.956392315580757,-0.5898965716593845),(-4.412112509911754,0.24900717320108162),(-5.795296060441225,-0.04561676599038633)],[(-1.3978903937006055,6.481302544560039),(-2.5739999654116086,8.0989447408846),(-3.3828210635738936,7.5108899550291),(-2.794766277718389,6.7020688568668225),(-4.412408474042954,5.525959285155815),(-3.8243536881874514,4.717138186993536),(-3.015532590025168,5.305192972849039),(-2.427477804169669,4.496371874686753),(-4.045120000494238,3.3202623029757565),(-3.4570652146387264,2.5114412048134636),(-1.0306019201518817,4.275605562379974),(-2.206711491862887,5.893247758704539)],[(-1.2222980145261584,10.914213371930243),(0.22088383034438697,9.529574009989366),(0.9132035113148267,10.251164932424631),(0.19161258887955457,10.943484613395068),(1.576251950820443,12.386666458265607),(0.8546610283851708,13.078986139236052),(0.16234134741472572,12.35739521680078),(-0.5592495750205463,13.049714897771228),(-0.5299783335557156,11.635804294365512)],[(-6.547937903781419,-7.306434226738093),(-8.16867948800115,-6.1345994614316055),(-8.754596870654396,-6.944970253541466),(-7.94422607854453,-7.53088763619471),(-9.11606084385102,-9.151629220414442),(-8.305690051741163,-9.737546603067685),(-7.719772669087914,-8.927175810957815),(-6.909401876978045,-9.51309319361107),(-7.133855286434668,-8.116805018847955)],[(-1.7268719610755825,-4.917984095624801),(-1.7936502720258365,-2.9190992422161246),(-3.792535125434516,-2.9858775531663775)],[(3.205426119788286,0.0025122670893310637),(3.1259119630802834,-1.9959064825432409),(5.124330712712854,-2.075420639251246)],[(9.38142463136234,3.531530896282441),(9.271070485339575,5.5284840661028),(8.27259390042939,5.473306993091416),(8.327770973440774,4.474830408181238),(6.3308178036204135,4.364476262158463),(6.3859948766317975,3.36599967724829),(7.38447146154198,3.4211767502596753),(7.439648534553361,2.4227001653494944),(8.382948046452162,3.476353823271057)],[(-5.846126259145565,3.5008361519510487),(-3.9204634602987363,4.041042395100266),(-4.190566581873344,5.003873794523682),(-5.153397981296754,4.7337706729490705),(-5.693604224445976,6.659433471795904),(-6.656435623869397,6.389330350221293),(-6.386332502294784,5.4264989507978765),(-7.3491639017181996,5.15639582922327),(-6.116229380720174,4.4636675513744635)],[(3.2984074233374403,0.6993209415920973),(4.356841369180706,-0.9976521230988693),(5.20532790152619,-0.4684351501772348),(4.676110928604558,0.38005138216825024),(6.373083993295525,1.4384853280115137),(5.843867020373894,2.286971860356993),(4.99538048802841,1.7577548874353608),(4.466163515106776,2.6062414197808463),(4.146893955682924,1.2285379145137285)]]
rand = (-8, 10)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (-0.5806552537935943,12.336935156911455)
goal = (-1.1825477634651227,12.44489696010881)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.1825477634651227,12.44489696010881)
goal = (-1.8891286728652341,12.757355355192892)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.8891286728652341,12.757355355192892)
goal = (1.1463192622316232,11.500665462336958)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1463192622316232,11.500665462336958)
goal = (1.2584857607675985,10.50153240161917)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.2584857607675985,10.50153240161917)
goal = (2.4670786208743714,10.918981285726584)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.4670786208743714,10.918981285726584)
goal = (1.384903320995166,6.516708692596115)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.384903320995166,6.516708692596115)
goal = (8.054113813919791,8.413283452919892)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (-1.1825477634651227,12.44489696010881)
goal = (-1.6863038036431837,12.521576970154364)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.6863038036431837,12.521576970154364)
goal = (-1.0763895410407756,10.042356585675119)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.0763895410407756,10.042356585675119)
goal = (1.5970313060106847,12.385395337768049)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.5970313060106847,12.385395337768049)
goal = (-3.825821599153298,10.552313231403755)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.825821599153298,10.552313231403755)
goal = (3.0963052829915796,11.333027035588316)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.0963052829915796,11.333027035588316)
goal = (-0.3783847551088506,5.845156795440342)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.3783847551088506,5.845156795440342)
goal = (7.53124029660427,6.857240380002985)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (-1.6863038036431837,12.521576970154364)
goal = (-3.655752847488829,12.422313458438193)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.655752847488829,12.422313458438193)
goal = (-4.532635494561281,12.67513361608789)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.532635494561281,12.67513361608789)
goal = (-0.29989238969842447,9.225461326080325)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.29989238969842447,9.225461326080325)
goal = (-1.4948456297226365,7.90303705532369)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.4948456297226365,7.90303705532369)
goal = (0.055695765795878316,5.838138209149239)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.055695765795878316,5.838138209149239)
goal = (1.1942752500600697,2.2963461392252977)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (-1.8891286728652341,12.757355355192892)
goal = (-4.33644656428548,12.42667774749259)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.33644656428548,12.42667774749259)
goal = (-4.668638529231142,12.410983504548977)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.668638529231142,12.410983504548977)
goal = (-4.831606239988123,10.6871617212297)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.831606239988123,10.6871617212297)
goal = (1.511316776227046,8.910998145973029)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.511316776227046,8.910998145973029)
goal = (-1.7639568445993659,5.454859769932851)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.7639568445993659,5.454859769932851)
goal = (7.881099291099574,7.317487456727463)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (-1.0763895410407756,10.042356585675119)
goal = (-1.4353986473296887,9.082082351748397)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.4353986473296887,9.082082351748397)
goal = (0.07868126985876778,9.000035505795214)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.07868126985876778,9.000035505795214)
goal = (0.20322177678720088,7.26137092876769)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.20322177678720088,7.26137092876769)
goal = (-0.3213876211951572,5.475954095837793)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.3213876211951572,5.475954095837793)
goal = (3.2044101138131307,2.3965574989246505)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (-3.655752847488829,12.422313458438193)
goal = (-4.765213097590191,11.839913119373875)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.765213097590191,11.839913119373875)
goal = (-5.22151909289413,10.646178400982432)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.22151909289413,10.646178400982432)
goal = (-7.583762759664778,11.173757004225838)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.583762759664778,11.173757004225838)
goal = (-1.4872256782016446,5.276765582632336)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.4872256782016446,5.276765582632336)
goal = (-0.10349179179024759,0.9018016950123009)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (-4.33644656428548,12.42667774749259)
goal = (-4.589693968833323,11.519410932634536)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.589693968833323,11.519410932634536)
goal = (-6.702785678163285,10.556336153312056)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.702785678163285,10.556336153312056)
goal = (-7.743456678085393,11.048904242186623)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.743456678085393,11.048904242186623)
goal = (-4.0989156858723685,4.815612438753936)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.0989156858723685,4.815612438753936)
goal = (2.2354746999113946,1.63892018901136)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (1.1463192622316232,11.500665462336958)
goal = (1.6570708495112747,11.784962204937958)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.6570708495112747,11.784962204937958)
goal = (2.2744762132822256,11.24713456017611)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.2744762132822256,11.24713456017611)
goal = (2.7600530806229635,10.08067256098893)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.7600530806229635,10.08067256098893)
goal = (5.454022650239709,10.803421352662369)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.454022650239709,10.803421352662369)
goal = (8.849630699393973,9.17805722367098)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (1.5970313060106847,12.385395337768049)
goal = (2.713618701311157,11.645542101009568)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.713618701311157,11.645542101009568)
goal = (3.6213579305953854,12.588203308260283)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.6213579305953854,12.588203308260283)
goal = (5.6142202857528485,11.578479382657498)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.6142202857528485,11.578479382657498)
goal = (8.612515393480457,7.86509835522758)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (-4.532635494561281,12.67513361608789)
goal = (-7.352039546046061,12.144840323963983)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.352039546046061,12.144840323963983)
goal = (-7.737522143975806,10.620021199635811)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.737522143975806,10.620021199635811)
goal = (-0.8252821024966668,4.938937386304806)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.8252821024966668,4.938937386304806)
goal = (1.8454860196263976,1.3777522362140981)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (-4.668638529231142,12.410983504548977)
goal = (-6.606449654152318,10.259982602430284)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.606449654152318,10.259982602430284)
goal = (-6.351429052432561,8.73463635015072)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.351429052432561,8.73463635015072)
goal = (-8.348785778399423,4.914473303842168)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-8.348785778399423,4.914473303842168)
goal = (1.1170653460341953,0.6961541701070626)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (-1.4353986473296887,9.082082351748397)
goal = (-1.5132573355902093,8.291621809645601)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.5132573355902093,8.291621809645601)
goal = (-0.7148471444343141,6.414863740573498)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.7148471444343141,6.414863740573498)
goal = (0.6833693422917655,5.545173444381273)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.6833693422917655,5.545173444381273)
goal = (7.0792595528403055,4.749165395039661)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (-4.765213097590191,11.839913119373875)
goal = (-5.645849863620892,9.343309681870537)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.645849863620892,9.343309681870537)
goal = (-7.29696713757362,9.050852946793935)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.29696713757362,9.050852946793935)
goal = (-8.434205937873234,4.897901396946127)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-8.434205937873234,4.897901396946127)
goal = (-0.19535036691559782,-0.7358912815999616)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (-4.589693968833323,11.519410932634536)
goal = (-6.678226594459433,9.684399229067957)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.678226594459433,9.684399229067957)
goal = (-5.038709446239843,7.925991934001814)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.038709446239843,7.925991934001814)
goal = (-6.591504167663261,3.97489528493932)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.591504167663261,3.97489528493932)
goal = (-5.656806446440317,-1.6435458364829163)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (1.6570708495112747,11.784962204937958)
goal = (2.7981186551062684,11.630245351665057)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.7981186551062684,11.630245351665057)
goal = (3.4490428068259114,10.184102796363625)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.4490428068259114,10.184102796363625)
goal = (3.7351947093595914,7.8012064709557265)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.7351947093595914,7.8012064709557265)
goal = (7.815949021226366,4.877425864207854)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (1.2584857607675985,10.50153240161917)
goal = (2.3824870476340703,10.24872485971423)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.3824870476340703,10.24872485971423)
goal = (2.38148615850492,9.179685431541717)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.38148615850492,9.179685431541717)
goal = (2.9169502717520235,6.9503035835858284)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.9169502717520235,6.9503035835858284)
goal = (9.013135472340206,5.895779763987003)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (-3.825821599153298,10.552313231403755)
goal = (-4.096280701756387,7.337913816648456)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.096280701756387,7.337913816648456)
goal = (-3.6809488694679624,3.6406337533476645)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.6809488694679624,3.6406337533476645)
goal = (-1.4672141804627126,-1.3372544849907992)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (-0.29989238969842447,9.225461326080325)
goal = (0.8527260482032322,7.417501085661378)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.8527260482032322,7.417501085661378)
goal = (-0.6614837126319184,4.066955607338292)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.6614837126319184,4.066955607338292)
goal = (5.094088058589941,1.977137950159781)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (-4.831606239988123,10.6871617212297)
goal = (-6.793697204511064,7.937415853727245)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.793697204511064,7.937415853727245)
goal = (-6.202021716548124,3.6910373501770977)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.202021716548124,3.6910373501770977)
goal = (-6.805937685330882,-1.6097135209141502)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (0.07868126985876778,9.000035505795214)
goal = (0.738482951785306,6.767825959595001)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.738482951785306,6.767825959595001)
goal = (3.170775194985568,5.371827714502853)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.170775194985568,5.371827714502853)
goal = (6.007228867690353,2.3896385645510776)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (-5.22151909289413,10.646178400982432)
goal = (-8.0148799382518,9.45702746648935)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.0148799382518,9.45702746648935)
goal = (-5.768003574006125,3.3867732114178786)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.768003574006125,3.3867732114178786)
goal = (-5.202228602065961,-1.8829047773855905)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (-6.702785678163285,10.556336153312056)
goal = (-8.89115662269531,10.082963824884)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.89115662269531,10.082963824884)
goal = (-9.007064629650442,4.396567375698318)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-9.007064629650442,4.396567375698318)
goal = (-8.557024201476326,-1.764005856086869)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (2.2744762132822256,11.24713456017611)
goal = (4.189901076223546,10.326843473640078)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.189901076223546,10.326843473640078)
goal = (6.592423934325694,11.397220869029983)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.592423934325694,11.397220869029983)
goal = (9.085426551171718,3.3869734685089625)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (2.713618701311157,11.645542101009568)
goal = (4.192696340162648,12.620901473648818)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.192696340162648,12.620901473648818)
goal = (6.506398413936672,9.918968361401639)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.506398413936672,9.918968361401639)
goal = (8.771902154861717,2.470606849628938)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (-7.352039546046061,12.144840323963983)
goal = (-8.401810133665268,7.638851887753841)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.401810133665268,7.638851887753841)
goal = (-6.018941481236346,3.1259922874809654)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.018941481236346,3.1259922874809654)
goal = (-4.309062711500579,-1.8937019500781025)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (-6.606449654152318,10.259982602430284)
goal = (-8.02076600716394,7.04195143906421)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.02076600716394,7.04195143906421)
goal = (-8.286402985020034,2.948736247388796)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-8.286402985020034,2.948736247388796)
goal = (-7.2554330367735345,-2.6444572461782734)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (-1.5132573355902093,8.291621809645601)
goal = (-0.19172301716931273,6.065517998432977)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.19172301716931273,6.065517998432977)
goal = (2.2523257559443675,4.1727767544332615)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.2523257559443675,4.1727767544332615)
goal = (1.9648235831923522,-1.2678572925579008)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (-5.645849863620892,9.343309681870537)
goal = (-7.653177742069651,6.039325531128666)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.653177742069651,6.039325531128666)
goal = (-4.985583717271919,2.0874651969897204)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.985583717271919,2.0874651969897204)
goal = (-5.1314796816270345,-2.619629001519307)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (-6.678226594459433,9.684399229067957)
goal = (-6.866054652093922,4.742856966516337)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.866054652093922,4.742856966516337)
goal = (-7.208896488335263,2.128548417905858)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.208896488335263,2.128548417905858)
goal = (-8.903894170643513,-2.524102865648813)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (2.7981186551062684,11.630245351665057)
goal = (4.289990372430681,13.027771063876408)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.289990372430681,13.027771063876408)
goal = (6.589943122904225,9.628164576316447)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.589943122904225,9.628164576316447)
goal = (8.146432133341417,1.774225587724672)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (2.3824870476340703,10.24872485971423)
goal = (4.571931028855007,10.049910540907753)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.571931028855007,10.049910540907753)
goal = (5.368854241365533,7.507487380027182)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.368854241365533,7.507487380027182)
goal = (6.465081875427222,0.8822203870708076)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (2.4670786208743714,10.918981285726584)
goal = (4.272997468351326,8.999490580695337)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.272997468351326,8.999490580695337)
goal = (6.954570826835376,9.607995886068192)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.954570826835376,9.607995886068192)
goal = (6.991288549908138,0.9304081519188525)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (3.0963052829915796,11.333027035588316)
goal = (7.079218100149237,12.813849289532211)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.079218100149237,12.813849289532211)
goal = (6.382395939618087,0.3568749596376257)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (-1.4948456297226365,7.90303705532369)
goal = (-0.9257324503851372,2.74078382970721)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.9257324503851372,2.74078382970721)
goal = (1.1660900997793835,-1.9181699730793582)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (1.511316776227046,8.910998145973029)
goal = (4.86965729344295,6.567862174132724)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.86965729344295,6.567862174132724)
goal = (5.288461198283942,-1.330071118966556)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (0.20322177678720088,7.26137092876769)
goal = (2.8345529929290514,4.6682524869060025)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.8345529929290514,4.6682524869060025)
goal = (1.6690929127108642,-2.0548649093797007)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (-7.583762759664778,11.173757004225838)
goal = (-7.188135322029185,1.9526000413504825)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.188135322029185,1.9526000413504825)
goal = (-8.96325240182101,-2.6265007438644723)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (-7.743456678085393,11.048904242186623)
goal = (-4.2653782123253965,2.153998757999368)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.2653782123253965,2.153998757999368)
goal = (-7.83402724499556,-3.231824577677327)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (2.7600530806229635,10.08067256098893)
goal = (6.76745377262109,8.830501190090938)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.76745377262109,8.830501190090938)
goal = (7.1467077382434105,-0.6514286380715664)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (3.6213579305953854,12.588203308260283)
goal = (7.816167579226001,11.329313341111803)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.816167579226001,11.329313341111803)
goal = (7.1271024016617694,-1.877047995693526)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (-7.737522143975806,10.620021199635811)
goal = (-3.967975683179356,2.1974467340711055)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.967975683179356,2.1974467340711055)
goal = (-6.375684892162343,-3.2811953719140075)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (-6.351429052432561,8.73463635015072)
goal = (-3.7993424412428567,1.9350300700562464)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.7993424412428567,1.9350300700562464)
goal = (-4.530750273042681,-3.5223534403095176)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (-0.7148471444343141,6.414863740573498)
goal = (-0.004735106710217707,2.887082810835004)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.004735106710217707,2.887082810835004)
goal = (1.479832177145612,-2.1609094704143095)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (-7.29696713757362,9.050852946793935)
goal = (-5.438010712576013,1.3530273517351485)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.438010712576013,1.3530273517351485)
goal = (-7.206050287734774,-3.8439407515175263)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (-5.038709446239843,7.925991934001814)
goal = (-2.7435169325271938,2.2829806614452934)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.7435169325271938,2.2829806614452934)
goal = (-4.876397621463908,-4.152235818491346)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (3.4490428068259114,10.184102796363625)
goal = (7.432655501598177,10.043115667213499)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.432655501598177,10.043115667213499)
goal = (4.363799945097147,-2.303483125127536)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (2.38148615850492,9.179685431541717)
goal = (3.7031626434563627,5.416931425129718)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.7031626434563627,5.416931425129718)
goal = (1.2517943477859,-2.7279175444526054)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (-4.096280701756387,7.337913816648456)
goal = (-2.43152553907011,1.9855996400322855)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.43152553907011,1.9855996400322855)
goal = (0.9532459017973132,-3.045150365669887)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (0.8527260482032322,7.417501085661378)
goal = (3.6535909560061217,3.888878442369112)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.6535909560061217,3.888878442369112)
goal = (2.9931223381151018,-3.444967752928796)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (-6.793697204511064,7.937415853727245)
goal = (-6.274680667611687,1.1131511630755195)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.274680667611687,1.1131511630755195)
goal = (-4.092108837561084,-4.417470272898653)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (0.738482951785306,6.767825959595001)
goal = (1.7905728293449155,2.821571936497838)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.7905728293449155,2.821571936497838)
goal = (4.224215380854419,-3.1679333180532625)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (-8.0148799382518,9.45702746648935)
goal = (-6.200355375135557,1.082229662646819)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.200355375135557,1.082229662646819)
goal = (-4.994048050460903,-4.6475582527481)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (-8.89115662269531,10.082963824884)
goal = (-7.543484524203146,0.1953522793834086)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.543484524203146,0.1953522793834086)
goal = (-6.15538187270163,-4.99007421480103)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (4.189901076223546,10.326843473640078)
goal = (5.580002896812786,6.7100622894452435)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.580002896812786,6.7100622894452435)
goal = (7.771363106610071,-2.161820623899546)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (4.192696340162648,12.620901473648818)
goal = (8.863020383956854,10.420772595339084)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.863020383956854,10.420772595339084)
goal = (8.325734083074472,-2.2519473498513474)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (-8.401810133665268,7.638851887753841)
goal = (-2.890081289421042,0.7997960617718789)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.890081289421042,0.7997960617718789)
goal = (-7.553792205734238,-5.456053528952174)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (-8.02076600716394,7.04195143906421)
goal = (-8.47401093120758,-1.3480237357673737)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.47401093120758,-1.3480237357673737)
goal = (-6.431283839120669,-5.363186727660666)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (-0.19172301716931273,6.065517998432977)
goal = (-0.1449822098259066,2.502483162195606)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.1449822098259066,2.502483162195606)
goal = (3.3401531190353406,-4.020859562189584)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (-7.653177742069651,6.039325531128666)
goal = (-7.829919585822873,-1.4101310325756788)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.829919585822873,-1.4101310325756788)
goal = (-7.961141499602332,-5.624611290679264)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (-6.866054652093922,4.742856966516337)
goal = (-5.6382140937678145,-1.3496693289425075)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.6382140937678145,-1.3496693289425075)
goal = (-8.269222207090325,-5.864435823891984)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (4.289990372430681,13.027771063876408)
goal = (8.113140758475478,8.580739329443507)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.113140758475478,8.580739329443507)
goal = (7.7589125433409585,-2.48257351891257)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (4.571931028855007,10.049910540907753)
goal = (6.624588010339217,6.868886357374173)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.624588010339217,6.868886357374173)
goal = (8.160844595978233,-3.0584584373800654)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (4.272997468351326,8.999490580695337)
goal = (7.478438632223803,7.132725778610446)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.478438632223803,7.132725778610446)
goal = (8.330829862171434,-3.1799020836014407)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (1.384903320995166,6.516708692596115)
goal = (3.7324863041999965,3.5357738603617523)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.7324863041999965,3.5357738603617523)
goal = (3.7742171309765205,-3.9940282868634753)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (-0.3783847551088506,5.845156795440342)
goal = (1.58686923776248,-4.632033115115656)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (0.055695765795878316,5.838138209149239)
goal = (-2.0917305860479365,-5.1649981722994465)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (-1.7639568445993659,5.454859769932851)
goal = (-0.11548781447689116,-6.135044080912724)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (-0.3213876211951572,5.475954095837793)
goal = (3.9512709227766543,-4.7925139186067005)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (-1.4872256782016446,5.276765582632336)
goal = (-1.4963952947398784,-6.310425681888145)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (-4.0989156858723685,4.815612438753936)
goal = (-3.625975960353119,-6.337135093784339)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (5.454022650239709,10.803421352662369)
goal = (7.546364777467733,-3.494061258412704)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (5.6142202857528485,11.578479382657498)
goal = (8.842334861464433,-4.108449881872213)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (-0.8252821024966668,4.938937386304806)
goal = (1.7150775260273132,-5.991037132348438)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (-8.348785778399423,4.914473303842168)
goal = (-8.710114769818807,-7.302292318848369)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (0.6833693422917655,5.545173444381273)
goal = (5.501398957264451,-4.76995863341361)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (-8.434205937873234,4.897901396946127)
goal = (-3.6898921021206963,-6.656235537926269)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (-6.591504167663261,3.97489528493932)
goal = (-3.1115504732176342,-6.696357009434575)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (3.7351947093595914,7.8012064709557265)
goal = (6.646080055769687,-4.478469619132093)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (2.9169502717520235,6.9503035835858284)
goal = (7.1996013991743695,-4.545565926267621)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (-3.6809488694679624,3.6406337533476645)
goal = (2.2275579322642827,-5.996057811331559)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (-0.6614837126319184,4.066955607338292)
goal = (3.4581372719675034,-5.742649159758892)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (-6.202021716548124,3.6910373501770977)
goal = (-5.700620777833464,-7.597238946271253)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (3.170775194985568,5.371827714502853)
goal = (8.648985421885321,-4.568301561156455)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (-5.768003574006125,3.3867732114178786)
goal = (-1.76104425228601,-7.513310455277959)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (-9.007064629650442,4.396567375698318)
goal = (-8.695360962454618,-8.513819178054426)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (6.592423934325694,11.397220869029983)
goal = (7.114718822630763,-5.5502045450788495)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (6.506398413936672,9.918968361401639)
goal = (6.905458848515707,-5.753418536634692)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (-6.018941481236346,3.1259922874809654)
goal = (-4.785436537903169,-8.203172061138597)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (-8.286402985020034,2.948736247388796)
goal = (-5.561890422527986,-8.414258096825128)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (2.2523257559443675,4.1727767544332615)
goal = (3.6646570670629917,-6.419007035675981)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (-4.985583717271919,2.0874651969897204)
goal = (-5.1409080065123725,-8.357422909666589)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (-7.208896488335263,2.128548417905858)
goal = (-4.940911795496136,-8.50801381492836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (6.589943122904225,9.628164576316447)
goal = (8.410014848138598,-5.813620968913087)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (5.368854241365533,7.507487380027182)
goal = (9.30882699873061,-6.06675956269998)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (6.954570826835376,9.607995886068192)
goal = (8.816025208420221,-6.343110621375811)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (7.079218100149237,12.813849289532211)
goal = (7.42926187962566,-6.552271708619946)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (-0.9257324503851372,2.74078382970721)
goal = (1.5516311785279058,-7.708043282447644)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (4.86965729344295,6.567862174132724)
goal = (6.733878255820626,-6.944766574358582)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (2.8345529929290514,4.6682524869060025)
goal = (4.86326759664543,-7.246547965876346)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (-7.188135322029185,1.9526000413504825)
goal = (-3.224548536466921,-8.706611635159067)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (-4.2653782123253965,2.153998757999368)
goal = (-1.6305464798394391,-7.984947737301049)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (6.76745377262109,8.830501190090938)
goal = (8.979618945517153,-6.838896770202937)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (7.816167579226001,11.329313341111803)
goal = (9.276125823408119,-7.424228151249831)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (-3.967975683179356,2.1974467340711055)
goal = (-1.9950273020386255,-8.787786008470345)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (-3.7993424412428567,1.9350300700562464)
goal = (0.17501158632247815,-8.890112385471589)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-0.004735106710217707,2.887082810835004)
goal = (2.292076704340408,-7.913185491879064)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (-5.438010712576013,1.3530273517351485)
goal = (-0.015243697349868413,-9.483543191513736)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-2.7435169325271938,2.2829806614452934)
goal = (1.9985139577929818,-9.055725172754192)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (7.432655501598177,10.043115667213499)
goal = (4.954488320365108,-7.3337380973449875)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (3.7031626434563627,5.416931425129718)
goal = (6.458387016066332,-7.681612841726243)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (-2.43152553907011,1.9855996400322855)
goal = (0.7259682992539549,-9.685493338596222)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (3.6535909560061217,3.888878442369112)
goal = (5.251611180769258,-8.309461481807759)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (-6.274680667611687,1.1131511630755195)
goal = (4.642428983334055,-9.275436137545617)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (1.7905728293449155,2.821571936497838)
goal = (5.414728820105568,-8.427117607873232)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (-6.200355375135557,1.082229662646819)
goal = (5.313964793304471,-8.990997077830748)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (-7.543484524203146,0.1953522793834086)
goal = (5.646718449309811,-8.62556071245241)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (5.580002896812786,6.7100622894452435)
goal = (9.102882584856438,-7.439539181495505)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (8.863020383956854,10.420772595339084)
goal = (7.557165231138626,-8.187849915644737)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (-2.890081289421042,0.7997960617718789)
goal = (6.393997333790322,-8.730459912526559)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (-8.47401093120758,-1.3480237357673737)
goal = (7.276172207203302,-8.41244425010317)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-0.1449822098259066,2.502483162195606)
goal = (7.2603240610318664,-8.847345024289233)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (-7.829919585822873,-1.4101310325756788)
goal = (8.238705098379196,-8.786311372692374)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (-5.6382140937678145,-1.3496693289425075)
goal = (8.47183362435145,-9.161076571508602)
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
f = open('smo2sol-20.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
