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

obstacleList = [[(-0.0976483872639774,-0.10564467932051001),(1.1844203300023048,-1.6406690453183885),(2.719444696000184,-0.3586003280521064)],[(0.09344181768081013,-0.0037622982511678633),(1.8909609879261418,0.8731212929971426),(1.4525191923019865,1.7718808781198088),(0.5537596071793208,1.3334390824956532),(-0.3231239840689898,3.130958252740985),(-1.221883569191656,2.6925164571168296),(-0.7834417735675007,1.793756871994164),(-1.6822013586901665,1.3553150763700084),(-2.5590849499384776,3.1528342466153396),(-3.457844535061143,2.7143924509911845),(-3.019402739436988,1.815632865868518),(-3.918162324559653,1.3771910702443635),(-2.5809609438128325,0.9168732807458531),(-2.142519148188677,0.018113695623187376),(-0.3449999779433453,0.8949972868714979)],[(-2.6362836621064423,-2.171447968159453),(-1.9730452317281557,-0.28462132416977814),(-2.9164585537229932,0.04699789101936824),(-3.2480777689121356,-0.896415430975471),(-5.134904412901811,-0.23317700059718405),(-5.4665236280909575,-1.1765903225920222),(-4.523110306096116,-1.5082095377811646),(-4.854729521285263,-2.4516228597760032),(-3.5796969841012793,-1.839828752970309)],[(-0.09326431107488953,-0.7177047580761363),(-2.0642318461590263,-1.0572441160957373),(-1.8944621671492246,-2.042727883637807),(-0.9089783996071571,-1.8729582046280058),(-0.5694390415875523,-3.843925739712143),(0.4160447259545158,-3.674156060702342),(0.2462750469447137,-2.688672293160271),(1.2317588144867808,-2.518902614150472),(0.07650536793491211,-1.703188525618204)],[(1.6912367367553314,3.6472839237915786),(0.3491998675076484,5.130165254707803),(-1.1336814634085783,3.7881283854601153)],[(3.0364639773182462,-1.0866567512476395),(5.036115428750128,-1.123994035379791),(5.054784070816206,-0.12416830966384929),(4.054958345100264,-0.1054996675977744),(4.092295629232416,1.8941517838341084),(3.092469903516481,1.9128204259001838),(3.0738012614504027,0.9129947001842416),(2.0739755357344603,0.9316633422503215),(3.0551326193843242,-0.08683102553169897)],[(2.1999257025889793,-3.6134905987410137),(0.6692173603048053,-4.900709294558328),(1.3128267082134557,-5.666063465700415),(2.0781808793555427,-5.022454117791759),(3.365399575172855,-6.553162460075942),(4.130753746314942,-5.9095531121672895),(3.4871443984062918,-5.144198941025191),(4.252498569548378,-4.500589593116543),(2.8435350504976347,-4.378844769883103)],[(6.779700591705986,1.1062710772658166),(6.592106549807127,-0.8849116053236128),(7.587697891101842,-0.9787086262730467),(7.6814949120512726,0.016882715021669492),(9.672677594640707,-0.17071132687718227),(9.766474615590134,0.8248800144175337),(8.770883274295418,0.9186770353669614),(8.864680295244845,1.9142683766616826),(7.775291933000701,1.0124740563163892)],[(-3.9038285831817365,-7.261728006470083),(-5.869900578351039,-6.894902557774133),(-6.05331330269902,-7.8779385553587815),(-5.070277305114366,-8.061351279706761),(-5.437102753810313,-10.027423274876067),(-4.45406675622566,-10.210835999224042),(-4.270654031877689,-9.227800001639388),(-3.287618034293038,-9.411212725987365),(-4.0872413075297125,-8.244764004054735)],[(-2.358902014859919,2.972883137052348),(-0.9905650378277391,4.431531112845832),(-1.7198890257244739,5.115699601361923),(-2.404057514240567,4.386375613465183),(-3.8627054900340494,5.754712590497366),(-4.546873978550138,5.02538860260063),(-3.817549990653397,4.3412201140845355),(-4.501718479169493,3.611896126187795),(-3.088226002756658,3.6570516255684407)],[(5.777844918863362,6.48736803615264),(6.294680028738635,4.555301404485862),(7.260713344572029,4.813718959423501),(7.002295789634389,5.779752275256888),(8.934362421301168,6.296587385132165),(8.67594486636353,7.26262070096556),(7.709911550530138,7.004203146027915),(7.451493995592503,7.970236461861306),(6.743878234696751,6.745785591090278)],[(-7.568594026243649,-3.4212150346540535),(-5.7286486667517575,-4.2051800045884535),(-5.336666181784555,-3.285207324842508),(-6.256638861530501,-2.8932248398753084),(-5.472673891596104,-1.0532794803834102),(-6.392646571342048,-0.6612969954162127),(-6.784629056309253,-1.5812696751621604),(-7.704601736055196,-1.1892871901949604),(-7.1766115412764515,-2.5012423549081078)],[(-6.557909876240082,6.4142762260776),(-4.558815664700304,6.3540904022637585),(-4.528722752793382,7.353637508033651),(-5.52826985856327,7.383730419940568),(-5.468084034749426,9.382824631480347),(-6.467631140519312,9.412917543387268),(-6.497724052426236,8.413370437617377),(-7.497271158196127,8.4434633495243),(-7.4370853343822905,10.442557561064078),(-8.436632440152165,10.472650472971004),(-8.466725352059095,9.473103367201112),(-9.466272457828987,9.50319627910803),(-8.496818263966023,8.473556261431218),(-8.52691117587294,7.474009155661334),(-6.527816964333157,7.413823331847489)],[(-1.4526620703961235,-6.332559581949827),(0.21168906537294796,-5.2235351021189285),(-0.34282317454249966,-4.391359534234392),(-1.1749987424270363,-4.9458717741498415),(-2.284023222257932,-3.2815206383807656),(-3.116198790142473,-3.8360328782962165),(-2.5616865502270216,-4.668208446180755),(-3.39386211811156,-5.222720686096201),(-2.007174310311572,-5.50038401406529)],[(1.8801620971319342,1.9306725504216893),(3.782652313636195,2.5475444788683976),(3.165780385189491,4.450034695372661)],[(8.602172345760605,-5.267843493927633),(8.631976640609798,-3.268065580256085),(7.632087683774018,-3.2531634328314842),(7.617185536349424,-4.253052389667263),(5.617407622677872,-4.223248094818072),(5.602505475253283,-5.223137051653843),(6.602394432089057,-5.238039199078441),(6.587492284664452,-6.237928155914207),(4.587714370992905,-6.208123861065015),(4.572812223568311,-7.208012817900797),(5.572701180404087,-7.222914965325391),(5.557799032979484,-8.222803922161164),(3.558021119307936,-8.19299962731197),(3.5431189718833433,-9.192888584147749),(4.543007928719108,-9.207790731572343),(4.528105781294512,-10.207679688408119),(5.542896885554871,-9.22269287899693),(6.542785842390649,-9.237595026421534),(6.572590137239857,-7.237817112749988),(7.572479094075634,-7.252719260174587),(7.602283388924832,-5.252941346503036)],[(-0.022442822746903582,5.561475624726316),(1.448949502802951,6.916099050004714),(0.7716377901637573,7.651795212779643),(0.03594162738882873,6.974483500140445),(-1.3186817978895635,8.445875825690308),(-2.0543779606644965,7.768564113051108),(-1.377066248025298,7.032867950276176),(-2.1127624108002308,6.355556237636975),(-0.6997545353861018,6.297171787501244)],[(6.246352061466462,2.5280250527871324),(7.706441543995224,3.8948237588916137),(7.023042190942982,4.624868500155998),(6.292997449678604,3.9414691471037546),(4.926198743574115,5.401558629632515),(4.196154002309736,4.718159276580275),(4.879553355361981,3.9881145353158938),(4.149508614097597,3.304715182263655),(5.5629527084142225,3.258069794051512)]]
rand = (-12,12)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (9.26880565930645,-2.1349141454065315)
goal = (9.105866833901965,-1.5759461120051803)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.105866833901965,-1.5759461120051803)
goal = (9.704903929823564,-1.422017205901584)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.704903929823564,-1.422017205901584)
goal = (9.519365045048636,-3.8984750049273016)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.519365045048636,-3.8984750049273016)
goal = (7.555234600194757,-4.2410144174376985)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.555234600194757,-4.2410144174376985)
goal = (9.36660277250168,2.1304576218310096)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.36660277250168,2.1304576218310096)
goal = (4.191034183770256,2.0198548288461886)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.191034183770256,2.0198548288461886)
goal = (9.47230690001575,8.695944536772469)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (9.105866833901965,-1.5759461120051803)
goal = (8.568031900497433,-1.4975136184192852)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.568031900497433,-1.4975136184192852)
goal = (8.43613291364756,-1.5870007352129445)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.43613291364756,-1.5870007352129445)
goal = (7.416143131313904,-2.898620572610456)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.416143131313904,-2.898620572610456)
goal = (9.086238543106388,1.4583590451066843)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.086238543106388,1.4583590451066843)
goal = (7.449427089671383,1.9202939120610711)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.449427089671383,1.9202939120610711)
goal = (3.8777928819999836,1.9587832249480215)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.8777928819999836,1.9587832249480215)
goal = (5.388711257017622,8.129563411018975)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (8.568031900497433,-1.4975136184192852)
goal = (7.690826386260094,-0.6909607418889596)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.690826386260094,-0.6909607418889596)
goal = (6.66599961963127,-0.9732479865779773)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.66599961963127,-0.9732479865779773)
goal = (6.016380776930431,0.14544820224626953)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.016380776930431,0.14544820224626953)
goal = (7.730769132587399,2.114268182442787)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.730769132587399,2.114268182442787)
goal = (2.697858685409278,-2.1812892646660487)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.697858685409278,-2.1812892646660487)
goal = (3.365393230418226,7.785079106812827)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (9.704903929823564,-1.422017205901584)
goal = (9.673971470787487,-3.6342182568304926)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.673971470787487,-3.6342182568304926)
goal = (9.439078567128574,-4.036697075188319)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.439078567128574,-4.036697075188319)
goal = (9.113826194491194,1.8778238796507516)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.113826194491194,1.8778238796507516)
goal = (7.399106396187706,2.4889263633795906)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.399106396187706,2.4889263633795906)
goal = (4.804349612537733,3.225546704313734)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.804349612537733,3.225546704313734)
goal = (8.19291379321208,9.58713230308128)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (8.43613291364756,-1.5870007352129445)
goal = (6.025498407764836,-2.6227197627891004)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.025498407764836,-2.6227197627891004)
goal = (6.006384467072163,-3.1397600620254753)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.006384467072163,-3.1397600620254753)
goal = (4.791996181308599,-1.856127003880811)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.791996181308599,-1.856127003880811)
goal = (2.6736893594560165,0.98842574214137)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.6736893594560165,0.98842574214137)
goal = (-1.4887443795680158,2.4417699293062434)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (7.690826386260094,-0.6909607418889596)
goal = (7.250951652339081,1.1259328337981636)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.250951652339081,1.1259328337981636)
goal = (6.072333330061122,0.35119809158608106)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (6.072333330061122,0.35119809158608106)
goal = (5.657777817142408,1.180453066255831)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.657777817142408,1.180453066255831)
goal = (4.335095955303256,3.6089442344155866)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.335095955303256,3.6089442344155866)
goal = (1.4563331615344168,6.543864998484281)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (9.673971470787487,-3.6342182568304926)
goal = (9.263162903129642,-5.578167579809958)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.263162903129642,-5.578167579809958)
goal = (8.89314912083085,-6.565957540535947)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.89314912083085,-6.565957540535947)
goal = (8.420865280915367,-8.300640069911628)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.420865280915367,-8.300640069911628)
goal = (2.1262698400554108,-2.922854568461668)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.1262698400554108,-2.922854568461668)
goal = (-3.067603465826119,-5.47066491392329)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (9.519365045048636,-3.8984750049273016)
goal = (8.633437888955283,-5.654933993600254)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.633437888955283,-5.654933993600254)
goal = (8.062032635736196,-6.585618972897694)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.062032635736196,-6.585618972897694)
goal = (4.828121574721809,-4.755307269374144)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.828121574721809,-4.755307269374144)
goal = (3.152308229395098,-8.014992912278453)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.152308229395098,-8.014992912278453)
goal = (-2.7877262068549227,-7.346712929225562)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (7.416143131313904,-2.898620572610456)
goal = (5.835020157077738,-2.9457341887721755)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.835020157077738,-2.9457341887721755)
goal = (4.870780379376928,-2.69551430815559)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.870780379376928,-2.69551430815559)
goal = (1.8451329773974496,-6.121115573880867)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.8451329773974496,-6.121115573880867)
goal = (-1.9229650582698596,2.1765142463680096)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (6.66599961963127,-0.9732479865779773)
goal = (5.078385220140683,-1.9082831529665985)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.078385220140683,-1.9082831529665985)
goal = (4.503897197616272,-2.1125346552208377)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.503897197616272,-2.1125346552208377)
goal = (1.7506069005087301,0.49261557753542107)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.7506069005087301,0.49261557753542107)
goal = (-0.15696845582343677,5.040712282265883)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (9.439078567128574,-4.036697075188319)
goal = (7.7790772975787785,-7.224458732048997)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.7790772975787785,-7.224458732048997)
goal = (4.762703674073473,-4.629930982756727)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.762703674073473,-4.629930982756727)
goal = (3.5463419938744494,-9.227692793550515)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.5463419938744494,-9.227692793550515)
goal = (-2.6096073252927816,-9.376229526044405)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (6.025498407764836,-2.6227197627891004)
goal = (5.846667736937718,-3.5408686890599963)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.846667736937718,-3.5408686890599963)
goal = (4.224895127678684,-3.298590496358454)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.224895127678684,-3.298590496358454)
goal = (1.571622772176747,0.4254542833534174)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.571622772176747,0.4254542833534174)
goal = (-3.31250935895741,-0.6502363447640143)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (7.250951652339081,1.1259328337981636)
goal = (7.006470706464347,1.2269215794850954)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.006470706464347,1.2269215794850954)
goal = (5.778392154791742,2.252135553707296)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (5.778392154791742,2.252135553707296)
goal = (4.117964920496025,4.7803149147148805)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.117964920496025,4.7803149147148805)
goal = (3.2305523161972154,8.403703294556205)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (9.263162903129642,-5.578167579809958)
goal = (7.697557946422078,-7.674120146537135)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.697557946422078,-7.674120146537135)
goal = (8.04041984933277,-8.679014838766365)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (8.04041984933277,-8.679014838766365)
goal = (3.0953748882332306,-8.711534195741658)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.0953748882332306,-8.711534195741658)
goal = (-3.5054638443059822,-6.956646251299784)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (8.633437888955283,-5.654933993600254)
goal = (6.201056947211924,-6.1831161874019145)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.201056947211924,-6.1831161874019145)
goal = (7.632196175225465,-9.235192324972834)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.632196175225465,-9.235192324972834)
goal = (3.2421428335092326,-9.017768503115628)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.2421428335092326,-9.017768503115628)
goal = (-3.728428579344647,-6.040348765783154)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (7.555234600194757,-4.2410144174376985)
goal = (5.011079740289752,-3.429092126354078)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.011079740289752,-3.429092126354078)
goal = (4.115667684142432,-5.13366286988697)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (4.115667684142432,-5.13366286988697)
goal = (1.7185494988577936,-6.679764274568765)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.7185494988577936,-6.679764274568765)
goal = (-2.8343607830817596,0.9818721054290851)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (9.086238543106388,1.4583590451066843)
goal = (9.549608685221854,4.655524550924012)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.549608685221854,4.655524550924012)
goal = (9.616519833776108,7.457770510416772)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.616519833776108,7.457770510416772)
goal = (9.462228402978846,9.833818877270854)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (6.016380776930431,0.14544820224626953)
goal = (4.647857674603511,1.0627056690079062)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.647857674603511,1.0627056690079062)
goal = (3.447799045276305,4.298376655190307)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.447799045276305,4.298376655190307)
goal = (-1.6826722522784685,3.6885346280433424)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (9.113826194491194,1.8778238796507516)
goal = (7.863332588319929,4.787743967666675)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.863332588319929,4.787743967666675)
goal = (3.6602843555807834,4.942235532051567)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.6602843555807834,4.942235532051567)
goal = (3.954373619845816,8.832549591887364)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (6.006384467072163,-3.1397600620254753)
goal = (4.0716535636871924,-3.0065788501007678)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.0716535636871924,-3.0065788501007678)
goal = (1.4606656517639767,-6.858763490914976)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.4606656517639767,-6.858763490914976)
goal = (-4.135852074936734,-2.8263634088116367)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (6.072333330061122,0.35119809158608106)
goal = (5.378374116367132,2.3888487417424997)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.378374116367132,2.3888487417424997)
goal = (2.3172548956503363,3.3826192347960617)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.3172548956503363,3.3826192347960617)
goal = (1.8900906546556282,7.997684739706184)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (8.89314912083085,-6.565957540535947)
goal = (7.930299671096114,-9.569633918114347)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.930299671096114,-9.569633918114347)
goal = (2.6139867474173997,-8.831025454346078)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.6139867474173997,-8.831025454346078)
goal = (-3.3139882581217863,-9.725216932039197)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (8.062032635736196,-6.585618972897694)
goal = (4.3410294243704985,-6.1917066488007775)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.3410294243704985,-6.1917066488007775)
goal = (2.417266157133513,-8.78118473874098)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.417266157133513,-8.78118473874098)
goal = (-3.7419344266448498,-8.369615463292861)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (5.835020157077738,-2.9457341887721755)
goal = (3.5846615146550125,-3.5710249028476735)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.5846615146550125,-3.5710249028476735)
goal = (0.43729340836385866,-0.8580992478699123)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.43729340836385866,-0.8580992478699123)
goal = (-3.9113391890089186,-0.43028491450499473)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (5.078385220140683,-1.9082831529665985)
goal = (3.9703080167935685,-1.2063903313006001)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.9703080167935685,-1.2063903313006001)
goal = (1.6722820587472427,2.56988166576264)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.6722820587472427,2.56988166576264)
goal = (-3.3180973525734867,2.156065277993383)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (7.7790772975787785,-7.224458732048997)
goal = (5.477228766407922,-9.921532298717883)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.477228766407922,-9.921532298717883)
goal = (2.150734022491358,-8.140733226335755)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (2.150734022491358,-8.140733226335755)
goal = (-4.67619021796943,-4.077563052502802)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (5.846667736937718,-3.5408686890599963)
goal = (2.994098162781709,-3.1361293900164355)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.994098162781709,-3.1361293900164355)
goal = (1.8882467510287615,-7.819820248216771)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.8882467510287615,-7.819820248216771)
goal = (-3.864551635916601,0.7485114524043155)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (7.006470706464347,1.2269215794850954)
goal = (5.795792947922797,2.9405237937506232)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.795792947922797,2.9405237937506232)
goal = (3.084544443920045,5.831669107696808)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (3.084544443920045,5.831669107696808)
goal = (3.314857382453356,8.97202817590097)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (7.697557946422078,-7.674120146537135)
goal = (4.277526792271242,-6.07960509219607)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.277526792271242,-6.07960509219607)
goal = (1.0199391269827505,-6.983375142860706)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.0199391269827505,-6.983375142860706)
goal = (-5.168518111469236,-3.6403585272649783)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (6.201056947211924,-6.1831161874019145)
goal = (3.4482804299506835,-7.893393839369679)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.4482804299506835,-7.893393839369679)
goal = (0.5938519659056283,-6.920019698069153)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.5938519659056283,-6.920019698069153)
goal = (-5.124239938413424,-2.72208777856802)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (5.011079740289752,-3.429092126354078)
goal = (2.618286683711691,-3.0951331173838756)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.618286683711691,-3.0951331173838756)
goal = (-0.0739021192642646,-4.437171506872378)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-0.0739021192642646,-4.437171506872378)
goal = (-3.6148523433938955,2.4834838350322546)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (9.36660277250168,2.1304576218310096)
goal = (5.251441850334018,3.0149716133418103)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.251441850334018,3.0149716133418103)
goal = (9.691035403834414,8.495966687641669)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (9.691035403834414,8.495966687641669)
goal = (4.176877205574813,10.049296541168218)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (7.449427089671383,1.9202939120610711)
goal = (4.690502274352541,7.2893996217542)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.690502274352541,7.2893996217542)
goal = (1.1393605829491733,8.088002756821389)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (7.730769132587399,2.114268182442787)
goal = (6.246665817483819,7.918038386866499)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.246665817483819,7.918038386866499)
goal = (2.9575373418512303,9.533540620840885)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (7.399106396187706,2.4889263633795906)
goal = (2.978125212356181,6.010927126516542)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.978125212356181,6.010927126516542)
goal = (1.5362087475130526,8.687318769430508)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (4.791996181308599,-1.856127003880811)
goal = (-0.2255810506887137,-3.855492003088247)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.2255810506887137,-3.855492003088247)
goal = (-3.2042220108241786,3.408321234013913)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (5.657777817142408,1.180453066255831)
goal = (1.2802884366507303,3.5322224226434784)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.2802884366507303,3.5322224226434784)
goal = (0.049976115187920556,7.1183450614739066)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (8.420865280915367,-8.300640069911628)
goal = (0.3963159607528155,-6.511042794782924)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.3963159607528155,-6.511042794782924)
goal = (-5.783901280797914,-6.289462367879554)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (4.828121574721809,-4.755307269374144)
goal = (-1.2951268903831625,-4.0126914115704135)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.2951268903831625,-4.0126914115704135)
goal = (-5.638407126593564,-5.104520730898647)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (4.870780379376928,-2.69551430815559)
goal = (-0.8194793167139096,0.029346360123080117)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8194793167139096,0.029346360123080117)
goal = (-4.690033191742115,0.513963283287822)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (4.503897197616272,-2.1125346552208377)
goal = (0.4889492670643083,2.1106291671655306)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.4889492670643083,2.1106291671655306)
goal = (-4.144488725678545,2.148382143094313)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (4.762703674073473,-4.629930982756727)
goal = (-0.5858209285582952,-7.9946272347617775)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.5858209285582952,-7.9946272347617775)
goal = (-5.314971197706688,-0.3304537811655148)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (4.224895127678684,-3.298590496358454)
goal = (-2.2905499592809013,-2.3071894244238793)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.2905499592809013,-2.3071894244238793)
goal = (-5.694102504842835,-0.4385781069096897)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (5.778392154791742,2.252135553707296)
goal = (2.412777274608775,5.788603928961395)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.412777274608775,5.788603928961395)
goal = (2.2621151581350123,9.539490756504435)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (8.04041984933277,-8.679014838766365)
goal = (-0.3541807800094112,-9.744273368227468)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.3541807800094112,-9.744273368227468)
goal = (-5.875209493030557,-10.059740157612156)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (7.632196175225465,-9.235192324972834)
goal = (-0.5127077598548695,-9.18434879825937)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.5127077598548695,-9.18434879825937)
goal = (-6.214992076202863,-7.748097813259757)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (4.115667684142432,-5.13366286988697)
goal = (-1.0076892651539051,-7.37025854913373)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.0076892651539051,-7.37025854913373)
goal = (-6.242645901453949,-4.637683431105549)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (9.549608685221854,4.655524550924012)
goal = (9.310539889834326,8.512850197588026)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.310539889834326,8.512850197588026)
goal = (0.8613648219119199,9.344321435806986)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (4.647857674603511,1.0627056690079062)
goal = (0.3629562758484415,2.5089718182266143)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (0.3629562758484415,2.5089718182266143)
goal = (-0.022779772729471404,8.064063956495012)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (7.863332588319929,4.787743967666675)
goal = (8.950935755770873,8.67838274729046)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.950935755770873,8.67838274729046)
goal = (0.3663237524963332,8.46215339784765)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (4.0716535636871924,-3.0065788501007678)
goal = (-2.0775874807561623,-5.930703140698889)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.0775874807561623,-5.930703140698889)
goal = (-4.394951214857922,2.6584100269492517)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (5.378374116367132,2.3888487417424997)
goal = (1.0543616206063096,4.374728835373109)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.0543616206063096,4.374728835373109)
goal = (-0.6696528837622022,7.8133138118793966)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (7.930299671096114,-9.569633918114347)
goal = (-0.8695446887033071,-9.45946903136333)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.8695446887033071,-9.45946903136333)
goal = (-6.366879732632414,-8.512132613022992)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (4.3410294243704985,-6.1917066488007775)
goal = (-1.0997884215068225,-7.9838711753419584)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.0997884215068225,-7.9838711753419584)
goal = (-6.367786946871513,-4.31139680040583)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (3.5846615146550125,-3.5710249028476735)
goal = (-1.9165036563256397,-6.698311660935721)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-1.9165036563256397,-6.698311660935721)
goal = (-5.230060516874864,2.029046921350533)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (3.9703080167935685,-1.2063903313006001)
goal = (-0.3254849357884009,3.1684891784931573)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.3254849357884009,3.1684891784931573)
goal = (-6.559020208084309,-0.449744835204978)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (5.477228766407922,-9.921532298717883)
goal = (-2.1815038354549587,-8.733443384618694)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.1815038354549587,-8.733443384618694)
goal = (-6.755558120579791,-5.962448490824169)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (2.994098162781709,-3.1361293900164355)
goal = (-2.7290139800084336,-4.525066079806915)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.7290139800084336,-4.525066079806915)
goal = (-7.166717513973668,-1.1806400614865957)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (5.795792947922797,2.9405237937506232)
goal = (4.967778489470103,7.6371345852129835)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.967778489470103,7.6371345852129835)
goal = (0.6161330503763178,9.731405922324758)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (4.277526792271242,-6.07960509219607)
goal = (-2.365353421721095,-5.733896425915096)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.365353421721095,-5.733896425915096)
goal = (-7.154125811622611,-5.76104539675132)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (3.4482804299506835,-7.893393839369679)
goal = (-2.4080518785817047,-8.862931658870382)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.4080518785817047,-8.862931658870382)
goal = (-7.169863977555668,-6.023125732121636)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (2.618286683711691,-3.0951331173838756)
goal = (-2.906585841816592,-2.277617102560206)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-2.906585841816592,-2.277617102560206)
goal = (-6.842959558637849,0.4757393760601438)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (5.251441850334018,3.0149716133418103)
goal = (4.71828296021145,7.780419430225468)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.71828296021145,7.780419430225468)
goal = (-1.1269680189987206,8.345556918296971)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (4.191034183770256,2.0198548288461886)
goal = (1.5873689666215434,6.194701706082032)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.5873689666215434,6.194701706082032)
goal = (-2.3268096679849295,7.3419683317068625)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (3.8777928819999836,1.9587832249480215)
goal = (-3.761627314121272,6.380397592310693)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (2.697858685409278,-2.1812892646660487)
goal = (-6.957076700445645,0.6407031152217932)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (4.804349612537733,3.225546704313734)
goal = (-1.5191788537345268,9.612436572184603)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (2.6736893594560165,0.98842574214137)
goal = (-4.218946119744106,5.912706230180941)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (4.335095955303256,3.6089442344155866)
goal = (-2.106507297132149,9.32642704810819)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (2.1262698400554108,-2.922854568461668)
goal = (-7.474728500969665,-4.770567060472976)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (3.152308229395098,-8.014992912278453)
goal = (-7.553104900129899,-9.74930905677127)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (1.8451329773974496,-6.121115573880867)
goal = (-7.524986036986687,-10.207889842728509)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (1.7506069005087301,0.49261557753542107)
goal = (-4.143850773279473,6.417659526706654)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (3.5463419938744494,-9.227692793550515)
goal = (-7.7853607003807035,-9.837059412089193)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (1.571622772176747,0.4254542833534174)
goal = (-5.865000957124653,3.9374127155632817)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (4.117964920496025,4.7803149147148805)
goal = (-2.1205634072191186,9.372388625646646)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (3.0953748882332306,-8.711534195741658)
goal = (-8.286069577689865,-8.516946985433119)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (3.2421428335092326,-9.017768503115628)
goal = (-8.96620291762029,-7.463057956322052)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (1.7185494988577936,-6.679764274568765)
goal = (-7.854430883546397,-2.242074896092036)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (9.616519833776108,7.457770510416772)
goal = (-1.7652516990264289,9.822726217848757)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (3.447799045276305,4.298376655190307)
goal = (-4.113031650306021,6.775052682499307)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (3.6602843555807834,4.942235532051567)
goal = (-3.4937380808980985,8.403034647254938)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (1.4606656517639767,-6.858763490914976)
goal = (-8.99387185265867,-6.572675926888636)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (2.3172548956503363,3.3826192347960617)
goal = (-3.9630100698593402,7.479653189936464)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (2.6139867474173997,-8.831025454346078)
goal = (-9.1204392020113,-8.95604194841109)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (2.417266157133513,-8.78118473874098)
goal = (-9.280931943388815,-8.523336425367294)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (0.43729340836385866,-0.8580992478699123)
goal = (-6.156928105491783,3.734140638062998)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (1.6722820587472427,2.56988166576264)
goal = (-4.293490473092685,7.296968343772651)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (2.150734022491358,-8.140733226335755)
goal = (-8.999967623574893,-5.018614435670993)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (1.8882467510287615,-7.819820248216771)
goal = (-9.01370970705642,-4.176522839517413)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (3.084544443920045,5.831669107696808)
goal = (-3.641092072350715,9.50154728107934)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (1.0199391269827505,-6.983375142860706)
goal = (-7.643360573964589,-0.7903277862773344)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (0.5938519659056283,-6.920019698069153)
goal = (-9.45604081196029,-4.900886520925096)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (-0.0739021192642646,-4.437171506872378)
goal = (-8.484999350021642,-0.8205544730906489)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (9.691035403834414,8.495966687641669)
goal = (-4.148760416570201,9.23145677699608)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (4.690502274352541,7.2893996217542)
goal = (-5.568162075171804,9.509628412343694)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (6.246665817483819,7.918038386866499)
goal = (-5.5430315425896,9.934104537511029)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (2.978125212356181,6.010927126516542)
goal = (-6.2644544780202445,4.489071180532179)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (-0.2255810506887137,-3.855492003088247)
goal = (-8.60600724523206,-0.012585179106649136)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (1.2802884366507303,3.5322224226434784)
goal = (-6.4049443270243565,3.6677149380760223)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (0.3963159607528155,-6.511042794782924)
goal = (-9.334530708768636,-2.2488811330030147)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (-1.2951268903831625,-4.0126914115704135)
goal = (-8.80398122118867,-0.12726243945995286)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (-0.8194793167139096,0.029346360123080117)
goal = (-8.553587810125624,2.4009560231067297)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (0.4889492670643083,2.1106291671655306)
goal = (-6.737408807554153,6.588071959629822)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (-0.5858209285582952,-7.9946272347617775)
goal = (-9.12991548333534,-0.6266836341121227)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-2.2905499592809013,-2.3071894244238793)
goal = (-9.229179443618536,0.008196549650168805)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (2.412777274608775,5.788603928961395)
goal = (-5.6653586541489105,9.85596356044454)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-0.3541807800094112,-9.744273368227468)
goal = (-9.099620300835987,3.5079419032963504)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (-0.5127077598548695,-9.18434879825937)
goal = (-8.622683698385151,3.939841784708454)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (-1.0076892651539051,-7.37025854913373)
goal = (-9.116101871676769,4.024882354866646)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (9.310539889834326,8.512850197588026)
goal = (-5.603065158318151,10.094277201283074)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (0.3629562758484415,2.5089718182266143)
goal = (-6.673879426100205,7.1468551319439335)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (8.950935755770873,8.67838274729046)
goal = (-5.805451954860403,9.95189978891888)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (-2.0775874807561623,-5.930703140698889)
goal = (-9.29541265811847,5.2045812815942085)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (1.0543616206063096,4.374728835373109)
goal = (-5.992701345109014,9.568647458452254)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (-0.8695446887033071,-9.45946903136333)
goal = (-9.064443156320877,5.5046392668698445)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (-1.0997884215068225,-7.9838711753419584)
goal = (-8.499805428244695,5.950006521269653)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (-1.9165036563256397,-6.698311660935721)
goal = (-6.419218094678854,9.607657093543995)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (-0.3254849357884009,3.1684891784931573)
goal = (-6.483787336917773,10.026230746635786)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (-2.1815038354549587,-8.733443384618694)
goal = (-7.157912286511888,9.434060367111194)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-2.7290139800084336,-4.525066079806915)
goal = (-7.29766477466146,10.021039806967163)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (4.967778489470103,7.6371345852129835)
goal = (-7.349073199358832,10.427485584110332)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (-2.365353421721095,-5.733896425915096)
goal = (-9.11870364164235,10.081911813670395)
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
f = open('smo2sol-14.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
