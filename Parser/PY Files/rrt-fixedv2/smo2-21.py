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

obstacleList = [[(0.15358558045176507,0.11478201847925684),(1.1151294958478548,-0.1598692126571739),(0.2911758024385627,-3.044500958845444),(1.2527197178346539,-3.319152189981874),(2.0766734112439447,-0.43452044379360466),(3.0382173266400345,-0.7091716749300354),(3.312868557776466,0.2523722404660549),(4.274412473172555,-0.022278990670375948),(3.1758075486268327,-3.8684546522547354),(5.098895379419012,-4.417757114527598),(6.197500303964735,-0.5715814529432373),(7.159044219360825,-0.8462326840796679),(4.96183437026938,-8.538584007248387),(3.038746539477199,-7.989281544975526),(2.7640953083407647,-8.950825460371616),(4.687183139132945,-9.500127922644479),(4.412531907996516,-10.461671838040568),(1.5279001618082493,-9.637718144631274),(1.253248930671818,-10.599262060027364),(2.2147928460679083,-10.873913291163795),(1.9401416149314774,-11.835457206559886),(-0.9444901312567945,-11.011503513150593),(0.4287660244253604,-6.203783936170145),(-0.5327778909707286,-5.929132705033712),(-1.9060340466528802,-10.736852282014164),(-4.790665792841151,-9.912898588604872),(-5.065317023977584,-10.87444250400096),(1.665490383795038,-12.797001121955978),(1.3908391526586144,-13.758545037352066),(2.3523830680547078,-14.033196268488497),(3.176336761463997,-11.148564522300227),(4.137880676860091,-11.423215753436656),(3.8632294457236576,-12.384759668832746),(5.786317276515838,-12.934062131105607),(6.335619738788698,-11.010974300313428),(11.143339315769147,-12.384230455995583),(11.41799054690558,-11.422686540599486),(6.61027096992513,-10.049430384917338),(6.884922201061562,-9.087886469521246),(11.692641778042008,-10.461142625203404),(11.967293009178439,-9.499598709807312),(7.1595734321979885,-8.126342554125161),(8.532829587880142,-3.318622977144712),(13.340549164860592,-4.691879132826862),(13.615200395997022,-3.7303352174307736),(8.807480819016574,-2.35707906174862),(9.082132050153005,-1.395535146352528),(10.043675965549093,-1.670186377488962),(10.318327196685523,-0.708642462092874),(5.510607619705076,0.6646136935892837),(6.88386377538723,5.472333270569733),(5.922319859991139,5.746984501706164),(4.5490637043089865,0.9392649247257142),(3.587519788912896,1.2139161558621447),(3.8621710200493267,2.1754600712582346),(2.9006271046532364,2.450111302394665),(3.4499295669260976,4.373199133186845),(2.4883856515300082,4.647850364323276),(1.939083189257147,2.724762533531096),(0.9775392738610568,2.9994137646675267),(0.7028880427246262,2.037869849271437),(-1.220199788067554,2.5871723115442977),(0.9770100610238895,10.279523634713017),(0.015466145627799688,10.554174865849449),(-1.0831387789179217,6.7079992042650884),(-7.8139461866905515,8.630557822220103),(-8.088597417826982,7.669013906824014),(-1.3577900100543516,5.746455288868999),(-1.632441241190782,4.784911373472909),(-4.517072987379053,5.6088650668822),(-4.7917242185154825,4.6473211514861115),(-1.907092472327213,3.823367458076818),(-2.181743703463644,2.861823542680728),(-3.1432876188597336,3.1364747738171594),(-3.417938849996164,2.1749308584210696),(-1.4948510192039843,1.6256283961482083),(-2.0441534814768456,-0.29745943464397195),(-4.928785227665115,0.5264942587653216),(-5.203436458801545,-0.43504965663077),(-4.241892543405456,-0.7097008877672007),(-4.516543774541886,-1.6712448031632898),(-18.939702505483236,2.4485236638831713),(-19.76365619889253,-0.4361080823051009),(-5.340497467951179,-4.555876549351558),(-5.88979993022404,-6.47896438014374),(-4.928256014827949,-6.7536156112801695),(-3.280348628009366,-0.9843521189036305),(-2.3188047126132765,-1.2590033500400617),(-2.8681071748861378,-3.182091180832242),(-1.906563259490048,-3.456742411968672),(-0.5333071038078944,1.3509771650117774),(0.42823681158819565,1.0763259338753468)],[(-20.743868713116836,-4.503775777361863),(-23.557639574088874,-3.4632501688616895),(-23.90448144358894,-4.401173789185689),(-24.84240506391295,-4.054331919685635),(-23.801879455412774,-1.2405610587135993),(-24.739803075736774,-0.8937191892135337),(-25.780328684236956,-3.707490050185578),(-26.71825230456097,-3.360648180685528),(-21.16878239255995,11.646129744498694),(-23.044629633208036,12.339813483498812),(-28.594099545208977,-2.6669644416854084),(-29.532023165533033,-2.3201225721853334),(-27.104130079032554,4.245342770082781),(-28.042053699356565,4.59218463958285),(-29.4294211773569,0.8404901582867224),(-35.99488651962492,3.2683832447872128),(-36.34172838912501,2.330459624463159),(-29.776263046856933,-0.09743346203725967),(-30.469946785857047,-1.97328070268532),(-31.40787040618104,-1.6264388331852038),(-31.754712275681115,-2.56436245350924),(-24.251323313088996,-5.339097409509716),(-24.598165182589046,-6.27702102983374),(-23.660241562265032,-6.62386289933378),(-24.353925301265154,-8.499710139981817),(-23.416001680941154,-8.846552009481869),(-22.72231794194103,-6.970704768833846),(-21.784394321617036,-7.317546638333895),(-21.43755245211696,-6.37962301800988),(-19.56170521146894,-7.073306757010016),(-20.949072689469162,-10.825001238306067),(-22.82491993011719,-10.131317499305943),(-23.171761799617247,-11.069241119629954),(-21.295914558969212,-11.762924858630068),(-21.64275642846924,-12.700848478954091),(-25.39445090976534,-11.313481000953848),(-26.088134648765482,-13.189328241601872),(-24.212287408117444,-13.883011980602),(-24.559129277617462,-14.820935600926012),(-28.310823758913546,-13.433568122925763),(-26.22977254191319,-7.806026400981693),(-27.167696162237192,-7.459184531481635),(-29.24874737923756,-13.086726253425713),(-30.186670999561546,-12.739884383925661),(-27.758777913061152,-6.1744190416575595),(-28.69670153338516,-5.827577172157509),(-29.04354340288523,-6.765500792481505),(-32.795237884181276,-5.378133314481339),(-33.488921623181405,-7.25398055512929),(-29.737227141885334,-8.641348033129564),(-30.430910880885467,-10.517195273777558),(-34.18260536218151,-9.1298277957773),(-34.52944723168157,-10.06775141610139),(-30.77775275038551,-11.455118894101592),(-31.124594619885578,-12.393042514425574),(-32.06251824020962,-12.046200644925488),(-32.40936010970965,-12.984124265249552),(-31.471436489385674,-13.330966134749573),(-31.818278358885706,-14.26888975507363),(-34.63204921985775,-13.228364146573437),(-34.97889108935779,-14.16628776689748),(-34.04096746903381,-14.513129636397453),(-34.73465120803394,-16.388976877045522),(-33.796727587709896,-16.735818746545593),(-33.10304384870979,-14.859971505897551),(-32.1651202283858,-15.20681337539761),(-32.85880396738586,-17.082660616045676),(-31.920880347061903,-17.429502485545687),(-30.533512869061664,-13.677808004249629),(-28.657665628413657,-14.371491743249743),(-32.1260843234142,-23.750727946489917),(-33.06400794373825,-23.40388607698984),(-31.329798596237936,-18.71426797536976),(-32.26772221656192,-18.367426105869754),(-32.614564086061975,-19.30534972619376),(-39.18002942833007,-16.8774566396933),(-39.52687129783011,-17.81538026001737),(-32.961405955562086,-20.243273346517718),(-33.30824782506214,-21.181196966841732),(-37.059942306358224,-19.793829488841453),(-37.406784175858235,-20.731753109165552),(-33.65508969456217,-22.119120587165785),(-34.00193156406226,-23.05704420748976),(-35.87777880471026,-22.363360468489667),(-36.57146254371039,-24.23920770913769),(-32.81976806241434,-25.626575187137895),(-33.16660993191443,-26.564498807461973),(-44.42169337580249,-22.40239637346128),(-45.11537711480274,-24.278243614109154),(-44.17745349447865,-24.625085483609336),(-48.339555928479356,-35.88016892749742),(-44.587861447183414,-37.26753640549768),(-40.425759013182514,-26.012452961609632),(-36.67406453188652,-27.39982043960977),(-37.714590140386896,-30.21359130058175),(-36.776666520062726,-30.560433170081904),(-35.7361409115626,-27.746662309109844),(-33.86029367091455,-28.440346048109966),(-34.20713554041461,-29.37826966843398),(-32.331288299766435,-30.071953407434155),(-26.781818387765572,-15.065175482249892),(-24.90597114711754,-15.758859221250022),(-25.599654886117655,-17.63470646189805),(-24.66173126579362,-17.98154833139811),(-23.274363787793433,-14.229853850102034),(-22.33644016746939,-14.57669571960212),(-22.683282036969477,-15.514619339926123),(-21.745358416645466,-15.861461209426173),(-23.132725894645613,-19.613155690722245),(-22.19480227432164,-19.959997560222313),(-21.84796040482159,-19.022073939898277),(-16.220418682877543,-21.10312515689863),(-15.873576813377458,-20.165201536574628),(-21.501118535321474,-18.084150319574274),(-20.8074347963214,-16.208303078926242),(-19.869511175997413,-16.55514494842631),(-19.522669306497384,-15.617221328102293),(2.9874975812789444,-23.941426196103716),(4.028023189779113,-21.127655335131724),(-18.482143697997202,-12.80345046713026),(-18.135301828497152,-11.865526846806237),(-16.259454587849085,-12.559210585806344),(-15.912612718349052,-11.621286965482359),(-17.788459958997052,-10.927603226482232),(-17.441618089497,-9.989679606158216),(-13.689923608200974,-11.37704708415847),(-13.343081738700901,-10.439123463834456),(-14.281005359024899,-10.092281594334366),(-13.587321620024778,-8.21643435368633),(-14.525245240348799,-7.869592484186297),(-15.218928979348926,-9.745439724834325),(-17.09477621999694,-9.051755985834191),(-16.747934350496898,-8.113832365510202),(-15.810010730172882,-8.460674235010256),(-15.463168860672823,-7.522750614686238),(-16.40109248099683,-7.175908745186183),(-15.70740874199671,-5.300061504538155),(-16.645332362320723,-4.953219635038088),(-17.33901610132085,-6.829066875686131),(-18.276939721644855,-6.482225006186056),(-16.889572243644622,-2.7305305248900034),(-18.76541948429264,-2.0368467858898796),(-20.152786962292886,-5.788541267185939),(-21.090710582616897,-5.4416993976858725)],[(7.5904411255968345,4.452887978392367),(10.587028544120347,4.309836088692877),(10.634712507353509,5.308698561534042),(17.62674981724172,4.974910818901887),(17.674433780474864,5.973773291743054),(10.682396470586678,6.307561034375212),(10.730080433819843,7.306423507216381),(9.731217960978675,7.354107470449542),(9.874269850678177,10.350694888973056),(16.866307160566357,10.016907146340898),(16.913991123799523,11.015769619182084),(15.91512865095836,11.063453582415217),(16.058180540657844,14.060041000938755),(15.059318067816722,14.107724964171881),(14.916266178117208,11.111137545648381),(13.917403705276005,11.158821508881571),(14.29887541114137,19.149721291610916),(19.293187775347178,18.91130147544515),(19.34087173858041,19.91016394828626),(14.346559374374552,20.14858376445208),(14.63266315377357,26.1417586014991),(11.636075735250051,26.284810491198606),(10.920816286752531,11.301873398581066),(9.921953813911355,11.349557361814217),(9.969637777144523,12.348419834655392),(8.970775304303348,12.39610379788856),(8.732355488137499,7.401791433682711),(7.733493015296338,7.449475396915877)]]
rand = (-55,30)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (-7.222730733099127,-21.711907814471918)
goal = (-5.025856610923064,-22.231038122693327)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.025856610923064,-22.231038122693327)
goal = (-11.38315696695642,-21.768492822781027)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-11.38315696695642,-21.768492822781027)
goal = (-5.5453692498406255,-17.475928314163145)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.5453692498406255,-17.475928314163145)
goal = (-7.217560223475239,-13.218070181846883)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.217560223475239,-13.218070181846883)
goal = (-16.609002655984447,-18.029239411703823)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-16.609002655984447,-18.029239411703823)
goal = (-1.701409820635746,-8.596747099020849)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.701409820635746,-8.596747099020849)
goal = (-23.276004253569912,-0.5137702547609209)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (-5.025856610923064,-22.231038122693327)
goal = (-2.521952853235888,-22.718784272741495)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.521952853235888,-22.718784272741495)
goal = (-2.593494150838886,-26.126477842112788)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.593494150838886,-26.126477842112788)
goal = (-6.708476764021555,-27.524335613639956)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.708476764021555,-27.524335613639956)
goal = (3.214224725654148,-20.18212506308201)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.214224725654148,-20.18212506308201)
goal = (3.671969185864924,-14.279952903291356)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.671969185864924,-14.279952903291356)
goal = (0.3799315797536238,-7.5899564222326426)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.3799315797536238,-7.5899564222326426)
goal = (-20.992781233860207,2.118035095187892)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (-2.521952853235888,-22.718784272741495)
goal = (-0.40876710787297554,-27.421574636200994)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.40876710787297554,-27.421574636200994)
goal = (2.560147141927068,-24.005886905267182)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.560147141927068,-24.005886905267182)
goal = (2.912447531939371,-16.48087929238604)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.912447531939371,-16.48087929238604)
goal = (6.034847932055037,-17.182850946536742)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.034847932055037,-17.182850946536742)
goal = (8.16254142382067,-13.014125449555124)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.16254142382067,-13.014125449555124)
goal = (-30.840331846328763,-5.7458719207796385)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (-11.38315696695642,-21.768492822781027)
goal = (-11.861313809534273,-26.88178027305959)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.861313809534273,-26.88178027305959)
goal = (-15.366960940814558,-25.877438997468744)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.366960940814558,-25.877438997468744)
goal = (-17.23529211936643,-22.188330598969372)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-17.23529211936643,-22.188330598969372)
goal = (-16.191964988317174,-29.939383234970144)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-16.191964988317174,-29.939383234970144)
goal = (-16.092028135569052,-4.9800910834902155)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-16.092028135569052,-4.9800910834902155)
goal = (-31.904102397733006,-3.06240977468439)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (-2.593494150838886,-26.126477842112788)
goal = (-4.070428157911593,-28.6856509326784)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.070428157911593,-28.6856509326784)
goal = (-4.731775241254255,-31.825015960878037)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.731775241254255,-31.825015960878037)
goal = (-6.9450011624005725,-35.31832056027669)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.9450011624005725,-35.31832056027669)
goal = (14.467139342975592,-28.33674328591657)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (14.467139342975592,-28.33674328591657)
goal = (-15.79972588943081,7.715847948673435)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (-0.40876710787297554,-27.421574636200994)
goal = (3.00022749767745,-26.467088742673017)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.00022749767745,-26.467088742673017)
goal = (0.9782021334092406,-31.88750622176768)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.9782021334092406,-31.88750622176768)
goal = (-0.08334084981683532,-36.38780959427723)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.08334084981683532,-36.38780959427723)
goal = (14.611253269223582,-25.740182006586096)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (14.611253269223582,-25.740182006586096)
goal = (-0.3290945225067432,11.079422694811782)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (-11.861313809534273,-26.88178027305959)
goal = (-9.343821807029435,-28.566226263154768)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.343821807029435,-28.566226263154768)
goal = (-15.956589230209445,-27.01036961805475)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.956589230209445,-27.01036961805475)
goal = (-18.40126971646812,-28.58722959911698)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-18.40126971646812,-28.58722959911698)
goal = (-28.557360775392603,-34.249003232517104)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-28.557360775392603,-34.249003232517104)
goal = (-39.34663696938362,-20.50162270229622)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (-5.5453692498406255,-17.475928314163145)
goal = (-6.897908291628639,-16.671504151603322)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.897908291628639,-16.671504151603322)
goal = (-8.385685185173337,-11.794539327906286)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-8.385685185173337,-11.794539327906286)
goal = (-8.260539512811484,-9.030925699147105)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-8.260539512811484,-9.030925699147105)
goal = (-2.1421889284752993,-5.161242254095747)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.1421889284752993,-5.161242254095747)
goal = (-16.22570182096578,8.259184581734353)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (-6.708476764021555,-27.524335613639956)
goal = (-4.285658993587553,-33.88678690536843)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.285658993587553,-33.88678690536843)
goal = (-6.552737674703899,-36.73644038928999)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.552737674703899,-36.73644038928999)
goal = (15.86049742012969,-32.01276260238255)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (15.86049742012969,-32.01276260238255)
goal = (-39.8665628546982,-23.33938786294692)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (2.560147141927068,-24.005886905267182)
goal = (6.11972697701232,-18.38902883673698)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.11972697701232,-18.38902883673698)
goal = (10.06776057724754,-21.405963443280974)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (10.06776057724754,-21.405963443280974)
goal = (14.7942502551739,-19.311024036802454)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (14.7942502551739,-19.311024036802454)
goal = (-5.714733979683189,10.796426689937327)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (-15.366960940814558,-25.877438997468744)
goal = (-18.994955085563085,-27.208191277750494)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-18.994955085563085,-27.208191277750494)
goal = (-22.51070868777458,-24.69681023591666)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-22.51070868777458,-24.69681023591666)
goal = (-31.72033545618931,-29.468754308326545)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-31.72033545618931,-29.468754308326545)
goal = (-39.64572220731615,-18.052503193643137)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (-4.070428157911593,-28.6856509326784)
goal = (-2.06043727441628,-32.914166109975326)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.06043727441628,-32.914166109975326)
goal = (3.7604263915748035,-35.67511717515708)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.7604263915748035,-35.67511717515708)
goal = (18.792292444539868,-21.571787441659005)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (18.792292444539868,-21.571787441659005)
goal = (-36.48890358775609,-9.68131426784041)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (3.00022749767745,-26.467088742673017)
goal = (9.061817811768371,-22.599311234905894)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.061817811768371,-22.599311234905894)
goal = (6.65539541600144,-34.62352420916406)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.65539541600144,-34.62352420916406)
goal = (17.26804968968426,-17.105238437789613)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (17.26804968968426,-17.105238437789613)
goal = (5.023660817804057,11.73161406801104)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (-9.343821807029435,-28.566226263154768)
goal = (-14.743918744872204,-32.06330566953211)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-14.743918744872204,-32.06330566953211)
goal = (-20.989097303741445,-33.111737994591245)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-20.989097303741445,-33.111737994591245)
goal = (-30.432966086934208,-33.71817884849117)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-30.432966086934208,-33.71817884849117)
goal = (-38.94926651592276,-11.089122014425968)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (-6.897908291628639,-16.671504151603322)
goal = (-12.229861974664665,-14.192322346495597)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-12.229861974664665,-14.192322346495597)
goal = (-5.328543514404046,-7.77536375234526)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.328543514404046,-7.77536375234526)
goal = (-0.16079352282253723,-4.475047609104955)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.16079352282253723,-4.475047609104955)
goal = (-14.061087795759349,10.235960328055114)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (-7.217560223475239,-13.218070181846883)
goal = (-9.043142603153164,-11.143882514107624)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.043142603153164,-11.143882514107624)
goal = (-6.164478421610816,-6.998020779727138)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.164478421610816,-6.998020779727138)
goal = (-15.76449932309989,-3.655794251718909)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.76449932309989,-3.655794251718909)
goal = (-7.74285781526892,11.799329579390111)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (3.214224725654148,-20.18212506308201)
goal = (8.734858036320318,-16.006728876159162)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.734858036320318,-16.006728876159162)
goal = (9.050151202948854,-8.521271276095117)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.050151202948854,-8.521271276095117)
goal = (5.444446505853911,12.489787449063854)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (2.912447531939371,-16.48087929238604)
goal = (6.213104872582271,-13.535163434204161)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.213104872582271,-13.535163434204161)
goal = (8.603357287799604,-7.695148720758098)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.603357287799604,-7.695148720758098)
goal = (6.061562054166757,12.774663825172269)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (-17.23529211936643,-22.188330598969372)
goal = (-22.44893431703696,-20.626265235977222)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.44893431703696,-20.626265235977222)
goal = (-29.711833191413444,-16.140416316549736)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-29.711833191413444,-16.140416316549736)
goal = (-34.70913536602931,-2.611696957297042)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (-4.731775241254255,-31.825015960878037)
goal = (5.681197179815015,-36.702755619445234)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.681197179815015,-36.702755619445234)
goal = (19.262913394127395,-20.41690688410802)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (19.262913394127395,-20.41690688410802)
goal = (-43.35392690982719,-36.619697576921375)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (0.9782021334092406,-31.88750622176768)
goal = (11.447933469040017,-34.73687062812006)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.447933469040017,-34.73687062812006)
goal = (17.108764591592347,-16.438894497444814)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (17.108764591592347,-16.438894497444814)
goal = (5.990130817927842,13.460719213951208)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (-15.956589230209445,-27.01036961805475)
goal = (-25.410740312750946,-31.722201850638896)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-25.410740312750946,-31.722201850638896)
goal = (-33.24502988238605,-29.851812493366907)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-33.24502988238605,-29.851812493366907)
goal = (-45.42873460914726,-26.824929493854846)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (-8.385685185173337,-11.794539327906286)
goal = (-10.77645959398874,-5.950587576529411)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-10.77645959398874,-5.950587576529411)
goal = (-15.26877766553062,-3.1635816020605105)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.26877766553062,-3.1635816020605105)
goal = (-27.194871006896484,4.487776084093682)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (-4.285658993587553,-33.88678690536843)
goal = (11.86921240375569,-34.20479471890004)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.86921240375569,-34.20479471890004)
goal = (-32.34272950095917,-35.65754062206715)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-32.34272950095917,-35.65754062206715)
goal = (-46.23405554665081,-27.042724761837768)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (6.11972697701232,-18.38902883673698)
goal = (10.85328269901818,-17.926385844591213)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (10.85328269901818,-17.926385844591213)
goal = (14.336587164733196,-11.416582390410195)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (14.336587164733196,-11.416582390410195)
goal = (4.976220538220957,14.072576012464303)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (-18.994955085563085,-27.208191277750494)
goal = (-26.75992300552529,-28.381360253886612)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-26.75992300552529,-28.381360253886612)
goal = (-31.307848449351358,-15.914988177362392)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-31.307848449351358,-15.914988177362392)
goal = (-46.78870056745271,-24.374353131462026)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (-2.06043727441628,-32.914166109975326)
goal = (13.668851771758114,-35.78548096383506)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (13.668851771758114,-35.78548096383506)
goal = (17.843476127836993,-15.036325942359326)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (17.843476127836993,-15.036325942359326)
goal = (-47.05111495784807,-30.029344682161238)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (9.061817811768371,-22.599311234905894)
goal = (11.308146203689894,-20.844874516245838)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.308146203689894,-20.844874516245838)
goal = (14.331182031872359,-10.751086888298076)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (14.331182031872359,-10.751086888298076)
goal = (3.1779487939276905,15.336068890484079)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (-14.743918744872204,-32.06330566953211)
goal = (-25.16114798445507,-34.056061173501895)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-25.16114798445507,-34.056061173501895)
goal = (-36.35663849552867,-35.95585551916808)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-36.35663849552867,-35.95585551916808)
goal = (-48.21023377278619,-26.797157801007728)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (-12.229861974664665,-14.192322346495597)
goal = (-18.03488918804779,-12.22967755833189)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-18.03488918804779,-12.22967755833189)
goal = (-26.624396737077102,-10.772884704796454)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-26.624396737077102,-10.772884704796454)
goal = (-31.76577837833674,5.2051335971326935)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (-9.043142603153164,-11.143882514107624)
goal = (-2.3110514323541835,-8.573307498821425)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.3110514323541835,-8.573307498821425)
goal = (-0.9440790061736593,-1.9145481410723022)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.9440790061736593,-1.9145481410723022)
goal = (-12.148568392851992,14.03310115985262)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (-16.609002655984447,-18.029239411703823)
goal = (-23.854770886572208,-15.805848150561669)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-23.854770886572208,-15.805848150561669)
goal = (-26.524443796943736,-9.703649519412398)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-26.524443796943736,-9.703649519412398)
goal = (-43.112167061676494,-12.713159355134351)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (3.671969185864924,-14.279952903291356)
goal = (9.991468141528813,-8.108029917207869)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.991468141528813,-8.108029917207869)
goal = (4.517022959617549,16.619648863005942)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (6.034847932055037,-17.182850946536742)
goal = (11.73751983789527,-7.711649031824063)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.73751983789527,-7.711649031824063)
goal = (-9.066986398963003,15.21869827050817)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (-16.191964988317174,-29.939383234970144)
goal = (-33.72376903238927,-18.409086151986713)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-33.72376903238927,-18.409086151986713)
goal = (-44.508459737996006,-13.138514012512221)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (-6.9450011624005725,-35.31832056027669)
goal = (-37.999060814790674,-36.379319272065594)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-37.999060814790674,-36.379319272065594)
goal = (-39.19648096672523,-2.9646852523090956)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (-0.08334084981683532,-36.38780959427723)
goal = (9.110378921812348,-6.318897247275508)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.110378921812348,-6.318897247275508)
goal = (-10.690370104500651,14.51184810933384)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (-18.40126971646812,-28.58722959911698)
goal = (-36.585149367300595,-21.524819005775946)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-36.585149367300595,-21.524819005775946)
goal = (-43.06557436582884,-6.722229466101968)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (-8.260539512811484,-9.030925699147105)
goal = (-11.482007941752741,0.9530107713912699)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.482007941752741,0.9530107713912699)
goal = (-13.953904672971447,14.638586059081263)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (-6.552737674703899,-36.73644038928999)
goal = (-41.246940507901016,-34.60252046217057)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-41.246940507901016,-34.60252046217057)
goal = (-45.153186197108866,-8.202517855170317)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (10.06776057724754,-21.405963443280974)
goal = (12.429093450325553,3.658944399041964)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (12.429093450325553,3.658944399041964)
goal = (5.842796100155418,19.827659617841974)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (-22.51070868777458,-24.69681023591666)
goal = (-37.81946509116803,-20.489885668609595)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-37.81946509116803,-20.489885668609595)
goal = (-47.867705448362656,-12.132418825071593)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (3.7604263915748035,-35.67511717515708)
goal = (-6.944754022527427,3.0195622193473923)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.944754022527427,3.0195622193473923)
goal = (-21.244513517982558,11.992839526969192)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (6.65539541600144,-34.62352420916406)
goal = (7.010480717619018,6.87238672025012)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.010480717619018,6.87238672025012)
goal = (0.9654360690823651,19.500505877885615)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (-20.989097303741445,-33.111737994591245)
goal = (-41.38922709979853,-30.49626566460124)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-41.38922709979853,-30.49626566460124)
goal = (-39.848470576599155,-1.7872597129053247)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (-5.328543514404046,-7.77536375234526)
goal = (-12.880442972753855,2.478095193245828)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-12.880442972753855,2.478095193245828)
goal = (-21.970763178324805,12.312782383623542)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (-6.164478421610816,-6.998020779727138)
goal = (-9.696418220659083,5.762775426550668)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.696418220659083,5.762775426550668)
goal = (-24.24315541149845,11.13304481440094)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (8.734858036320318,-16.006728876159162)
goal = (15.14324065571153,6.326505935840004)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (15.14324065571153,6.326505935840004)
goal = (6.368425267837097,21.13047807172174)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (6.213104872582271,-13.535163434204161)
goal = (8.692680196224941,7.578333314604301)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.692680196224941,7.578333314604301)
goal = (-3.5253862741025443,20.565446353817016)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (-22.44893431703696,-20.626265235977222)
goal = (-35.55144138856245,-14.809927295758264)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-35.55144138856245,-14.809927295758264)
goal = (-35.27248720576773,3.1716181627378504)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (5.681197179815015,-36.702755619445234)
goal = (-3.4630110647697876,6.071032498657374)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.4630110647697876,6.071032498657374)
goal = (-29.255238758988828,9.269624027750673)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (11.447933469040017,-34.73687062812006)
goal = (13.632945356789449,7.118830718515177)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (13.632945356789449,7.118830718515177)
goal = (18.633035520298193,20.06431535813043)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (-25.410740312750946,-31.722201850638896)
goal = (-41.532369122205225,-30.250114001138268)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-41.532369122205225,-30.250114001138268)
goal = (-41.28906576660835,-0.7026662556034822)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (-10.77645959398874,-5.950587576529411)
goal = (-11.33176362621247,6.643875984245625)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.33176362621247,6.643875984245625)
goal = (-17.72030006458713,16.46198555914546)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (11.86921240375569,-34.20479471890004)
goal = (1.0184024139658945,7.774085618928453)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.0184024139658945,7.774085618928453)
goal = (15.466340269310436,21.332484090048858)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (10.85328269901818,-17.926385844591213)
goal = (-4.7979296277105234,6.32032048923255)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.7979296277105234,6.32032048923255)
goal = (14.598723912129017,22.970248596017164)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (-26.75992300552529,-28.381360253886612)
goal = (-39.3565737391657,-22.94086531347182)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-39.3565737391657,-22.94086531347182)
goal = (-39.12037204475175,1.8477376649931685)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (13.668851771758114,-35.78548096383506)
goal = (-2.08813667937266,8.819171946747105)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.08813667937266,8.819171946747105)
goal = (9.383711331188074,23.713031145767737)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (11.308146203689894,-20.844874516245838)
goal = (8.780437346236454,11.414589336756457)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.780437346236454,11.414589336756457)
goal = (7.433398655626441,24.306506029868984)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (-25.16114798445507,-34.056061173501895)
goal = (-41.86187522763345,-35.38492427088704)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-41.86187522763345,-35.38492427088704)
goal = (-48.14804683722639,-0.2855552586872676)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (-18.03488918804779,-12.22967755833189)
goal = (-25.461544937283204,-5.7602504718032534)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-25.461544937283204,-5.7602504718032534)
goal = (-36.675113926392335,5.4930138689143675)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (-2.3110514323541835,-8.573307498821425)
goal = (-5.583909879433101,9.78148261680527)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.583909879433101,9.78148261680527)
goal = (-7.928642338837207,20.21356732448605)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (-23.854770886572208,-15.805848150561669)
goal = (-32.145545183942005,-9.24161902753643)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-32.145545183942005,-9.24161902753643)
goal = (-40.159084797942754,4.109145563252071)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (-1.701409820635746,-8.596747099020849)
goal = (-1.8147281484957247,11.092232627370471)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.8147281484957247,11.092232627370471)
goal = (-8.720484305273686,21.515529570880844)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (0.3799315797536238,-7.5899564222326426)
goal = (-17.168591362726673,18.241971824494527)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (8.16254142382067,-13.014125449555124)
goal = (3.761946203568577,24.783957740438368)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (-16.092028135569052,-4.9800910834902155)
goal = (-27.59888710273662,12.126039649218889)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (14.467139342975592,-28.33674328591657)
goal = (5.7895388895332545,25.89516425208754)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (14.611253269223582,-25.740182006586096)
goal = (-2.092262440080674,24.06336495258094)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (-28.557360775392603,-34.249003232517104)
goal = (-38.123173288110365,6.413493092690835)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (-2.1421889284752993,-5.161242254095747)
goal = (-11.110260466536772,21.458944428856796)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (15.86049742012969,-32.01276260238255)
goal = (-12.796747161976398,23.11985690434554)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (14.7942502551739,-19.311024036802454)
goal = (-21.78400378103913,18.268828570395428)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (-31.72033545618931,-29.468754308326545)
goal = (-37.83030457062592,7.334690888086733)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (18.792292444539868,-21.571787441659005)
goal = (-22.90140440013874,17.418284218651735)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (17.26804968968426,-17.105238437789613)
goal = (-19.936547425962786,20.520088840371905)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (-30.432966086934208,-33.71817884849117)
goal = (-44.309893410605504,6.0788649999799205)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (-0.16079352282253723,-4.475047609104955)
goal = (-26.941605432988673,14.664340081052643)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (-15.76449932309989,-3.655794251718909)
goal = (-29.87018785616929,11.195392131737549)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (9.050151202948854,-8.521271276095117)
goal = (-15.028397512980767,25.237539046030882)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (8.603357287799604,-7.695148720758098)
goal = (-17.345742721436245,23.951350671608537)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (-29.711833191413444,-16.140416316549736)
goal = (-40.19592029282582,8.02856871357831)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (19.262913394127395,-20.41690688410802)
goal = (-25.25435754196105,16.944215595733894)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (17.108764591592347,-16.438894497444814)
goal = (-31.89868824599197,10.41546223087218)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (-33.24502988238605,-29.851812493366907)
goal = (-47.22890158511383,8.111343878402046)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (-15.26877766553062,-3.1635816020605105)
goal = (-30.420063892078137,14.723635833614708)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (-32.34272950095917,-35.65754062206715)
goal = (-35.90898126446797,11.989660284826826)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (14.336587164733196,-11.416582390410195)
goal = (-24.272013292069303,21.674886914767583)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (-31.307848449351358,-15.914988177362392)
goal = (-39.560119325318546,11.900050037402764)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (17.843476127836993,-15.036325942359326)
goal = (-26.967219249237193,18.85809928229817)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (14.331182031872359,-10.751086888298076)
goal = (-29.641867178484222,15.532408244557104)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (-36.35663849552867,-35.95585551916808)
goal = (-45.65736338069675,11.786052824799917)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (-26.624396737077102,-10.772884704796454)
goal = (-34.210956486450684,16.229011428112408)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (-0.9440790061736593,-1.9145481410723022)
goal = (-27.555017428876965,19.773019051956148)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (-26.524443796943736,-9.703649519412398)
goal = (-41.782931072973,13.296824545943025)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (9.991468141528813,-8.108029917207869)
goal = (-28.170801571560123,21.009668957821674)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (11.73751983789527,-7.711649031824063)
goal = (-30.858327906500516,18.384889735136163)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (-33.72376903238927,-18.409086151986713)
goal = (-42.46266246460778,15.474521283776468)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (-37.999060814790674,-36.379319272065594)
goal = (-42.21772911916237,16.57112771520623)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (9.110378921812348,-6.318897247275508)
goal = (-29.74911220013546,21.129985337628845)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (-36.585149367300595,-21.524819005775946)
goal = (-35.48025754947936,16.964850464217903)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (-11.482007941752741,0.9530107713912699)
goal = (-28.057490863264142,23.59251597240172)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (-41.246940507901016,-34.60252046217057)
goal = (-44.020971209715796,18.16398546659486)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (12.429093450325553,3.658944399041964)
goal = (-30.24001535228748,23.436869061739863)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (-37.81946509116803,-20.489885668609595)
goal = (-37.052604678832495,18.36700694696662)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-6.944754022527427,3.0195622193473923)
goal = (-31.724798255391352,20.769715523684972)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (7.010480717619018,6.87238672025012)
goal = (-31.242171037434932,22.316225715927956)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-41.38922709979853,-30.49626566460124)
goal = (-43.65708303492154,19.7421007480893)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (-12.880442972753855,2.478095193245828)
goal = (-30.43443948232125,25.03439869913639)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (-9.696418220659083,5.762775426550668)
goal = (-37.47764418523307,21.701443418868408)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (15.14324065571153,6.326505935840004)
goal = (-39.0049874108765,19.819897615197682)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (8.692680196224941,7.578333314604301)
goal = (-39.051795954749124,19.87169446914038)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (-35.55144138856245,-14.809927295758264)
goal = (-39.181485772556584,21.372740239733353)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (-3.4630110647697876,6.071032498657374)
goal = (-37.38424050623721,26.165688421452735)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (13.632945356789449,7.118830718515177)
goal = (-39.4264542836699,22.24976844553342)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (-41.532369122205225,-30.250114001138268)
goal = (-48.11567059369648,19.664973483657853)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (-11.33176362621247,6.643875984245625)
goal = (-40.939460621680794,22.80878345649164)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (1.0184024139658945,7.774085618928453)
goal = (-42.46387020744599,22.32128834598057)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (-4.7979296277105234,6.32032048923255)
goal = (-43.52374318492329,22.67651863813382)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (-39.3565737391657,-22.94086531347182)
goal = (-44.610232843351284,24.79635831065798)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-2.08813667937266,8.819171946747105)
goal = (-43.24068899589541,25.8225147760092)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (8.780437346236454,11.414589336756457)
goal = (-45.093803325189526,25.731503177701654)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (-41.86187522763345,-35.38492427088704)
goal = (-48.0329243797423,25.155597145043636)
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
f = open('smo2sol-21.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
