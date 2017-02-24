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

obstacleList = [[(0.11019254188514943,-0.033252198573121027),(0.08691070491878392,2.9666574594111776),(-0.913059181075982,2.958896847089056),(-0.9208197933981044,3.9588667330838216),(1.0791199785914285,3.9743879577280654),(1.1567261018126462,-6.025310902219596),(3.1566658738021784,-6.009789677575353),(3.07905975058096,3.9899091823723096),(4.079029636575727,3.9976697946944317),(4.257523719984532,-19.00163758318519),(2.2575839479949953,-19.017158807829432),(2.179977824773777,-9.017459947881774),(0.1800380527842449,-9.032981172526018),(0.2576441760054621,-19.032680032473678),(-0.7423257099892999,-19.040440644795797),(-0.7888893839220328,-13.040621328827205),(-2.7888291559115723,-13.056142553471448),(-2.773307931267319,-15.056082325460983),(-4.773247703256854,-15.071603550105223),(-4.765487090934737,-16.071573436099985),(-2.7655473189452007,-16.056052211455746),(-2.7577867066230746,-17.056022097450512),(-5.757696364607374,-17.079303934416878),(-5.780978201573743,-14.07939427643258),(-6.78094808756851,-14.0871548887547),(-6.757666250602139,-17.087064546738997),(-7.757636136596918,-17.094825159061116),(-7.749875524274789,-18.094795045055882),(-2.75002609430096,-18.055991983445278),(-2.742265481978834,-19.055961869440043),(-25.741572859858454,-19.234455952848847),(-25.695009185925723,-25.23427526881744),(-18.695219983962367,-25.179950982562588),(-18.291668143212032,-77.17838505429044),(-5.292059625280101,-77.07749709410284),(-5.695611466030389,-25.079063022375006),(4.304087393917248,-25.00145689915379),(4.350651067849983,-31.001276215122388),(9.35050049782383,-30.962473153511773),(9.078879066549558,4.036472856305042),(11.078818838539092,4.051994080949282),(11.063297613894846,6.051933852938816),(5.063478297926248,6.0053701790060865),(5.016914623993522,12.005189494974681),(4.0169447379987515,11.99742888265256),(4.063508411931484,5.997609566683963),(2.06356863994195,5.98208834203972),(2.02476557833134,10.981937772013552),(1.0247956923365782,10.974177159691429),(1.063598753947185,5.974327729717598),(-0.9363410180423481,5.958806505073355),(-0.9441016303644705,6.958776391068121),(-1.9440715163592361,6.951015778745999),(-1.928550291714992,4.951076006756467),(-3.9284900637045257,4.935554782112222),(-4.006096186925744,14.935253642059884),(-6.006035958915275,14.91973241741564),(-5.990514734271034,12.919792645426108),(-8.990424392255333,12.896510808459741),(-8.982663779933207,11.896540922464977),(-5.98275412194891,11.919822759431343),(-5.9284298356940575,4.920033557467978),(-8.928339493678356,4.896751720501613),(-8.920578881356233,3.896781834506848),(-1.9207896793928703,3.9511061207617),(-1.9130290670707477,2.9511362347669343),(-2.9129989530655145,2.9433756224448118),(-2.9052383407433933,1.9434057364500452),(-6.905117884722457,1.9123632871615595),(-6.897357272400336,0.9123934011667933),(-4.897417500410803,0.9279146258110347),(-4.881896275766559,-1.0720251461784969),(-7.881805933750857,-1.0953069831448634),(-7.912848383039345,2.904572560834203),(-8.912818269034112,2.896811948512081),(-8.89729704438987,0.8968721765225512),(-12.897176588368934,0.8658297272340604),(-12.889415976046811,-0.13414015876070418),(-8.889536432067747,-0.10309770947221705),(-8.881775819745625,-1.1030675954669855),(-12.88165536372469,-1.1341100447554688),(-12.866134139080446,-3.134049816745008),(-4.866375051122317,-3.0719649181680286),(-4.858614438800193,-4.071934804162796),(-3.8586445528054285,-4.0641741918406735),(-3.8974476144160373,0.9356752381331583),(-2.897477728421271,0.9434358504552797),(-2.8897171160991495,-0.05653403553948637),(-1.889747230104383,-0.04877342321736459),(-1.8431835561716523,-6.0485927391859615),(-0.8432136701768869,-6.040832126863839),(-0.8897773441096168,-0.04101281089524281)],[(17.400649320507974,-33.71949229515208),(18.356933127973114,-34.011932510546565),(18.06449291257861,-34.96821631801171),(16.151925297648344,-34.38333588722271),(15.859485082253842,-35.33961969468785),(17.772052697184144,-35.92450012547683),(17.187172266395162,-37.83706774040712),(11.44946942160431,-36.08242644804017),(11.157029206209828,-37.03871025550529),(13.06959682114011,-37.62359068629431),(12.777156605745628,-38.57987449375945),(9.908305183350187,-37.702553847575956),(11.078066044928146,-33.87741861771543),(12.990633659858421,-34.46229904850438),(13.283073875252924,-33.506015241039265),(11.370506260322653,-32.921134810250265),(11.662946475717142,-31.96485100278512),(10.706662668252001,-31.67241078739064),(9.536901806674017,-35.49754601725115),(4.755482769348346,-34.03534494027877),(4.463042553953832,-34.991628747743846),(9.244461591279531,-36.453829824716315),(8.952021375885046,-37.41011363218147),(1.3017509161639325,-35.07059190902549),(0.716870485374951,-36.98315952395579),(8.367140945096075,-39.32268124711176),(5.442738791151182,-48.8855193217632),(7.3553064060814926,-49.47039975255218),(7.6477466214759655,-48.514115945087),(30.59855800063948,-55.532681114554634),(32.35319929300634,-49.79497826976387),(9.402387913842906,-42.776413100296196),(10.27970856002635,-39.907561677900745),(12.192276174956648,-40.49244210868974),(11.607395744167675,-42.40500972362003),(12.563679551632866,-42.697449939014525),(14.025880628605243,-37.9160309016888),(14.982164436070397,-38.2084711170833),(13.519963359097959,-42.989890154409004),(14.47624716656309,-43.28233036980349),(15.938448243535541,-38.500911332477784),(16.89473205100068,-38.79335154787228),(16.0174114048172,-41.66220297026767),(16.973695212282358,-41.95464318566218),(17.558575643071347,-40.0420755707319),(31.90283275504851,-44.4286788016491),(32.195272970442986,-43.47239499418396),(35.0641243928384,-44.349715640367435),(35.35656460823287,-43.39343183290226),(32.48771318583744,-42.516111186718845),(32.780153401231956,-41.559827379253676),(18.435896289254792,-37.17322414833647),(18.728336504649285,-36.21694034087133),(25.422323156905275,-38.26402184863272),(25.714763372299764,-37.307738041167596),(19.02077672004377,-35.26065653340619),(19.313216935438252,-34.30437272594105),(20.26950074290339,-34.596812941335536),(20.561940958297885,-33.640529133870395),(23.4307923806933,-34.517849780053844),(23.723232596087787,-33.56156597258868),(22.766948788622653,-33.26912575719423),(23.351829219411623,-31.35655814226395),(28.133248256737332,-32.81875921923636),(27.255927610553876,-35.6876106416318),(28.212211418019017,-35.98005085702621),(29.089532064202466,-33.11119943463082),(30.045815871667585,-33.40363965002528),(30.33825608706207,-32.4473558425602),(28.4256884721318,-31.86247541177121),(36.61401450317736,-5.086528802747274),(35.65773069571233,-4.794088587352828),(37.412371988079315,0.9436142574379787),(35.499804373148976,1.5284946882269566),(33.745163080782035,-4.209208156563875),(32.78887927331691,-3.9167679411693292),(24.600553242271232,-30.692714550193266),(23.644269434806105,-30.40027433479883),(24.229149865595083,-28.487706719868523),(23.27286605812995,-28.195266504474027),(21.810664981157505,-32.97668554179973),(20.85438117369236,-32.684245326405254),(21.14682138908686,-31.72796151894012),(20.190537581621708,-31.435521303545634),(21.36029844319967,-27.61038607368509),(20.404014635734516,-27.317945858290575),(19.819134204945556,-29.23051347322086),(15.993998975085022,-28.06075261164291),(15.7015587596905,-29.01703641910807),(19.526693989551056,-30.186797280686015),(19.234253774156574,-31.14308108815115),(18.277969966691433,-30.850640872756657),(17.98552975129694,-31.8069246802218),(16.072962136366687,-31.222044249432816),(15.780521920972184,-32.17832805689796),(17.69308953590246,-32.763208487686946)],[(-27.8904877290891,34.81391505870865),(-28.279435699813977,35.73517478108286),(-25.515656532691374,36.90201869325751),(-24.348812620516718,34.13823952613488),(-23.427552898142515,34.52718749685978),(-24.59439681031717,37.29096666398239),(-23.673137087942976,37.67991463470728),(-24.062085058667854,38.601174357081476),(-24.98334478104205,38.212226386356605),(-25.761240722491845,40.054745831105016),(-20.233682388246606,42.38843365545433),(-17.899994563897323,36.860875321209086),(-16.978734841523107,37.249823291934),(-19.31242266587241,42.77738162617921),(-18.39116294349821,43.16632959690406),(-15.279579177699128,35.79625181791047),(-14.358319455324942,36.18519978863538),(-15.914111338224446,39.87023867813215),(-6.701514114482432,43.75971838538102),(-7.479410055932242,45.60223783012951),(-16.692007279674264,41.71275812288061),(-17.469903221124042,43.55527756762901),(-16.548643498749826,43.9442255383539),(-17.326539440199557,45.78674498310227),(-26.539136663941616,41.8972652758534),(-26.928084634666504,42.81852499822757),(-22.32178602279548,44.76326485185205),(-22.710733993520368,45.684524574226245),(-27.31703260539138,43.739784720601826),(-27.705980576116247,44.661044442976014),(-28.627240298490452,44.27209647225115),(-25.90460450341627,37.82327841563171),(-26.82586422579045,37.43433044490683),(-29.54850002086466,43.88314850152625),(-30.469759743238875,43.49420053080138),(-27.74712394816466,37.04538247418194),(-28.668383670538866,36.65643450345706),(-29.05733164126375,37.57769422583126),(-29.978591363637953,37.18874625510637),(-31.534383246537498,40.873785144603175),(-32.455642968911704,40.48483717387829),(-32.06669499818682,39.56357745150411),(-32.987954720561014,39.17462948077922),(-34.93269457418548,43.78092809265021),(-35.85395429655964,43.39198012192535),(-35.465006325834764,42.47072039955114),(-38.228785492957385,41.303876487376456),(-41.34036925875644,48.67395426637012),(-2.647460919039897,65.00976903681531),(-4.981148743389372,70.53732737106061),(-43.67405708310577,54.20151260061539),(-44.451953024555564,56.04403204536375),(-46.29447246930391,55.26613610391398),(-40.07130493770577,40.52598054592673),(-40.99256466007997,40.13703257520186),(-42.54835654297955,43.82207146469866),(-43.46961626535371,43.43312349397376),(-41.9138243824542,39.74808460447685),(-42.8350841048284,39.359136633751994),(-42.05718816337861,37.51661718900366),(-34.68711038438499,40.628200954802736),(-33.90921444293523,38.7856815100543),(-35.75173388768364,38.00778556860452),(-35.362785916958735,37.086525846230344),(-31.677747027461923,38.64231772912991),(-30.899851086012156,36.7997982843815),(-31.821110808386365,36.4108503136566),(-31.43216283766148,35.48959059128242),(-33.274682282409884,34.71169464983262),(-32.88573431168499,33.79043492745841),(-31.043214866936594,34.56833086890819),(-30.65426689621171,33.64707114653399),(-29.733007173837493,34.036019117258896),(-29.34405920311262,33.11475939488467),(-31.186578647861026,32.33686345343491),(-30.79763067713613,31.415603731060706),(-28.955111232387733,32.19349967251048),(-28.566163261662844,31.272239950136274),(-31.329942428785444,30.105396037961597),(-30.940994458060565,29.184136315587402),(-30.019734735686363,29.573084286312287),(-29.24183879423656,27.73056484156389),(-31.084358238984997,26.95266890011412),(-30.695410268260105,26.031409177739903),(-28.85289082351174,26.80930511918972),(-28.074994882061933,24.96678567444129),(-27.153735159687795,25.35573364516616),(-29.09847501331218,29.962032257037183),(-28.177215290937994,30.350980227762065),(-27.788267320213105,29.42972050538787),(-26.867007597838874,29.81866847611274),(-28.03385151001354,32.582447643235355),(-27.112591787639342,32.971395613960254),(-24.389955992565167,26.52257755734081),(-23.46869627019095,26.91152552806569),(-23.857644240915903,27.83278525043988),(-22.936384518541637,28.22173322116478),(-22.158488577091916,26.37921377641633),(-28.607306633711318,23.656577981342195),(-28.21835866298636,22.735318258967986),(-25.454579495863822,23.902162171142653),(-16.119828198466706,1.791928834161716),(-15.19856847609255,2.180876804886566),(-14.420672534642883,0.33835736013809026),(-13.499412812268504,0.7273053308630466),(-14.277308753718303,2.5698247756114654),(-13.356049031344055,2.9587727463363684),(-22.690800328741215,25.06900608331726),(-21.769540606367023,25.457954054042155),(-20.99164466491725,23.61543460929374),(-20.07038494254305,24.004382580018614),(-20.848280883992782,25.84690202476706),(-6.108125326005531,32.07006955636505),(-6.886021267455273,33.9125890011135),(-21.626176825442563,27.68942146951544),(-22.015124796167438,28.610681191889654),(-18.33008590667061,30.166473074789167),(-18.719033877395496,31.08773279716339),(-24.246592211640706,28.754044972814125),(-24.63554018236561,29.675304695188323),(-20.950501292868793,31.23109657808784),(-21.72839723431854,33.07361602283625),(-25.41343612381538,31.517824139936707),(-26.191332065265133,33.360343584685126),(-25.270072342890934,33.74929155540999),(-25.659020313615812,34.67055127778419),(-28.422799480738423,33.50370736560957),(-28.8117474514633,34.42496708798378)],[(-27.771470721193953,-5.366684866522082),(-27.175196946980105,-4.56390370942834),(-26.372415789886354,-5.160177483642189),(-38.29789127416335,-21.21580062551707),(-36.69232895997574,-22.408348173944837),(-39.0774240568311,-25.61947280231984),(-43.09132984229987,-22.638103931250594),(-43.6876036165138,-23.44088508834428),(-41.27926014523254,-25.229706410985862),(-42.47180769366026,-26.835268725173332),(-41.669026536566406,-27.43154249938725),(-40.47647898813871,-25.82598018519974),(-39.67369783104494,-26.422253959413567),(-40.26997160525875,-27.225035116507303),(-39.46719044816497,-27.821308890721202),(-35.88954780288201,-23.004621948158647),(-35.0867666457882,-23.600895722372556),(-23.161291161511354,-7.545272580497544),(-21.555728847323856,-8.737820128925245),(-20.959455073110004,-7.93503897183145),(-22.565017387297516,-6.742491423403784),(-19.58364851622831,-2.7285856379350357),(-20.386429673322077,-2.132311863721204),(-23.367798544391263,-6.1462176491899285),(-24.973360858578765,-4.95367010076225),(-21.991991987509536,-0.93976431529354),(-22.79477314460329,-0.34349054107968247),(-25.77614201567251,-4.357396326548434),(-26.57892317276626,-3.761122552334598),(-25.982649398552407,-2.9583413952408373),(-26.78543055564616,-2.36206762102699),(-24.99660923300462,0.04627585025423819),(-25.799390390098374,0.6425496244680939),(-27.5882117127399,-1.765793846813152),(-28.39099286983365,-1.169520072599303),(-28.987266644047494,-1.9723012296930542),(-30.592828958235007,-0.7797536812653636),(-26.418912538738066,4.839714418390868),(-27.22169369583178,5.435988192604713),(-31.39561011532875,-0.18347990705152029),(-32.198391272422455,0.4127938671623559),(-28.024474852925543,6.0322619668185755),(-28.827256010019234,6.628535741032415),(-29.423529784233107,5.82575458393868),(-30.226310941326865,6.422028358152515),(-29.033763392899164,8.02759067234),(-26.625419921617944,6.238769349698453),(-26.029146147404088,7.041550506792195),(-26.83192730449791,7.637824281006067),(-25.63937975607017,9.243386595193547),(-26.44216091316392,9.839660369407376),(-27.634708461591625,8.234098055219894),(-28.43748961868525,8.830371829433789),(-26.64866829604389,11.238715300714976),(-27.45144945313757,11.834989074928824),(-31.029092098420595,7.018302132366371),(-32.63465441260817,8.210849680794025),(-28.460737993111188,13.83031778045028),(-29.263519150204868,14.42659155466411),(-30.45606669863263,12.821029240476609),(-32.06162901282012,14.013576788904285),(-32.65790278703394,13.210795631810608),(-31.05234047284648,12.018248083382911),(-32.24488802127419,10.412685769195395),(-35.45601264964919,12.797780866050772),(-33.0709175527938,16.008905494425765),(-34.676479866981175,17.20145304285352),(-37.061574963836605,13.990328414478505),(-37.86435612093037,14.586602188692364),(-38.46062989514427,13.783821031598581),(-32.841161795488034,9.609904612101637),(-33.43743556970185,8.807123455007897),(-35.04299788388926,9.99967100343565),(-35.639271658103176,9.196889846341875),(-30.019803558446945,5.022973426844922),(-30.616077332660847,4.220192269751175),(-37.03832658941073,8.990382463461998),(-37.63460036362464,8.187601306368236),(-36.83181920653099,7.59132753215431),(-38.02436675495862,5.985765217966858),(-42.03827254042737,8.967134089036056),(-42.634546314641206,8.164352931942323),(-41.02898400045372,6.971805383514647),(-42.221531548881416,5.36624306932713),(-47.84099964853766,9.540159488824001),(-48.43727342275152,8.737378331730278),(-42.81780532309524,4.563461912233382),(-43.41407909730913,3.7606807551396173),(-41.80851678312159,2.568133206711872),(-39.423421686266124,5.779257835086936),(-38.620640529172455,5.182984060873142),(-39.81318807760015,3.577421746685582),(-39.01040692050641,2.981147972471714),(-36.02903804943719,6.995053757940488),(-31.212351106874692,3.4174111126574207),(-31.808624881088534,2.6146299555636823),(-35.8225306665573,5.595998826632911),(-36.41880444077117,4.7932176695391675),(-32.404898655302404,1.8118487984699163),(-33.001172429516224,1.0090676413761859),(-33.80395358660998,1.6053414155900283),(-34.400227360823855,0.802560258496257),(-29.583540418261343,-2.775082386786786),(-30.179814192475185,-3.577863543880526),(-29.37703303538143,-4.17413731809439),(-30.56958058380914,-5.77969963228188),(-36.99182984055912,-1.0095094385711478),(-37.588103614772955,-1.812290595664865),(-31.16585435802299,-6.582480789375623),(-31.762128132236843,-7.385261946469354),(-37.38159623189308,-3.2113455269724698),(-37.97787000610691,-4.014126684066204),(-32.35840190645068,-8.188043103563114),(-32.954675680664536,-8.990824260656849),(-38.57414378032075,-4.816907841159947),(-39.17041755453461,-5.619688998253702),(-38.367636397440855,-6.215962772467556),(-43.13782659115161,-12.638212029217549),(-42.33504543405788,-13.234485803431369),(-37.5648552403471,-6.812236546681382),(-36.76207408325336,-7.408510320895228),(-42.12853805117801,-14.633540734738897),(-66.21197276399042,3.2546724916762706),(-69.1933416350597,-0.7592332937924118),(-45.109906922247205,-18.647446520207673),(-45.70618069646106,-19.450227677301424),(-43.297837225179784,-21.239048999942995),(-34.35373061197211,-9.197331643536794),(-33.55094945487837,-9.793605417750598),(-34.14722322909223,-10.596386574844338),(-33.344442071998465,-11.192660349058208),(-28.574251878287697,-4.770411092308235)],[(66.60322623182422,37.83233353631165),(66.86955455022202,36.86845116443247),(62.050142690826235,35.5368095724435),(62.31647100922399,34.572927200564365),(67.1358828686198,35.904568792553334),(67.40221118701763,34.940686420674155),(68.36609355889674,35.20701473907194),(68.89875019569241,33.27924999531365),(66.97098545193403,32.74659335851803),(67.23731377033185,31.78271098663886),(69.16507851409017,32.31536762343446),(69.69773515088579,30.38760287967618),(67.76997040712747,29.854946242880562),(68.03629872552533,28.89106387100143),(69.96406346928363,29.423720507797),(70.23039178768136,28.45983813591784),(71.19427415956063,28.726166454315663),(70.92794584116282,29.690048826194847),(71.89182821304193,29.956377144592583),(72.42448484983761,28.02861240083436),(67.60507299044173,26.69697080884527),(67.87140130883952,25.73308843696612),(72.6908131682354,27.064730028955168),(72.95714148663326,26.10084765707596),(67.17384725535818,24.50287774668909),(67.44017557375598,23.53899537480997),(73.22346980503109,25.136965285196865),(73.75612644182655,23.209200541438452),(66.04506746679321,21.07857399425607),(66.31139578519104,20.114691622376846),(74.02245476022445,22.245318169559347),(74.55511139702018,20.317553425801016),(64.91628767822829,17.65427024182299),(63.051989449443816,24.40144684497709),(66.90751893696033,25.466760118568324),(66.37486230016475,27.394524862326627),(65.41097992828563,27.1281965439288),(64.07933833629664,31.947608403324573),(66.00710308005493,32.48026504012025),(65.7407747616571,33.44414741199942),(63.813010017898755,32.911490775203816),(63.54668169950101,33.87537314708295),(62.58279932762181,33.60904482868517),(64.44709755640639,26.861868225531005),(62.51933281264812,26.329211588735383),(62.253004494250334,27.29309396061457),(60.32523975049196,26.760437323818962),(62.98852293447001,17.12161360502734),(62.02464056259083,16.85528528662948),(61.49198392579535,18.783050030387837),(60.5281015539161,18.516721711990076),(61.06075819071167,16.588956968231763),(60.096875818832764,16.32262864983388),(34.26302893424553,109.81921872211258),(35.22691130612524,110.08554704051045),(37.35753785330739,102.37448806547728),(38.321420225186074,102.64081638387485),(36.19079367800411,110.35187535890825),(38.118558421762145,110.88453199570381),(46.64106461049202,80.04029609557071),(50.49659409800853,81.10560936916178),(41.97408790927844,111.94984526929478),(48.72126451243287,113.81414349807942),(50.05290610442185,108.99473163868366),(51.01678847630103,109.26105995708157),(49.68514688431203,114.08047181647736),(51.612911628070144,114.61312845327299),(51.08025499127519,116.54089319703134),(59.7551963381871,118.93784806261161),(59.48886801978925,119.90173043449072),(67.19992699482307,122.03235698167308),(66.93359867642512,122.99623935355213),(62.114186817029264,121.66459776156334),(60.24988858824444,128.41177436471747),(59.286006216365,128.1454460463197),(61.15030444514988,121.3982694431654),(59.222539701391824,120.86561280637002),(58.95621138299349,121.82949517824906),(50.28127003608134,119.43254031266873),(50.01494171768351,120.39642268454783),(37.48447088325406,116.93415454537633),(33.22321778888931,132.35627249544302),(29.367688301372745,131.29095922185172),(33.62894139573811,115.86884127178551),(32.66505902385826,115.60251295338733),(32.39873070546105,116.56639532526683),(19.86825987103122,113.10412718609487),(47.566404984403626,12.86036051066236),(33.108169406216255,8.865435734695122),(31.2438711774316,15.612612337849285),(30.279988805552385,15.34628401945147),(32.14428703433696,8.59910741629757),(28.28875754682047,7.533794142706228),(29.886727457207193,1.750499911431234),(76.15308130740695,14.534259194526118),(78.81636449138486,4.895435475734402),(71.1053055163516,2.7648089285518935),(71.37163383474953,1.8009265566728558),(79.08269280978259,3.931553103855272),(79.34902112818075,2.967670731976213),(83.20455061569726,4.032984005567471),(75.74735770055861,31.021690418183844),(78.63900481619608,31.820675373377227),(77.57369154260483,35.67620486089393),(69.86263256757152,33.54557831371141),(69.32997593077593,35.47334305746974),(70.29385830265511,35.73967137586751),(70.02752998425731,36.70355374774671),(73.88305947177395,37.76886702133788),(73.61673115337614,38.732749393217155),(71.68896640961782,38.200092756421505),(71.42263809122005,39.16397512830065),(78.16981469437415,41.028273357085205),(77.90348637597637,41.99215572896446),(71.15630977282223,40.1278575001798),(70.88998145442444,41.091739872058994),(69.92609908254529,40.82541155366117),(70.72508403773865,37.93376443802369),(69.7612016658595,37.66743611962587),(69.49487334746173,38.63131849150502),(68.53099097558253,38.364990173107245),(67.46567770199141,42.220519660623886),(82.88779565205793,46.481772754988825),(82.35513901526232,48.40953749874709),(81.39125664338313,48.14320918034928),(81.12492832498529,49.107091552228496),(83.05269306874374,49.63974818902393),(89.44457271029106,26.506571263924435),(90.40845508217015,26.77289958232209),(91.74009667415915,21.953487722926226),(92.70397904603817,22.219816041323845),(91.37233745404927,27.03922790071985),(92.33621982592857,27.30555621911771),(91.53723487073505,30.197203334755223),(145.5146476959681,45.111589165032285),(141.78605123839876,58.60594237134066),(87.80863841316581,43.6915565410633),(85.94434018438113,50.438733144217494),(91.72763441565608,52.036703054604324),(93.32560432604294,46.25340882332941),(94.28948669792207,46.51973714172729),(92.69151678753533,52.30303137300201),(97.51092864693115,53.63467296499103),(98.30991360212457,50.74302584935354),(99.27379597400373,51.00935416775104),(98.4748110188102,53.90100128338901),(100.40257576256856,54.43365792018443),(99.07093417057962,59.25306977958029),(79.79328673299639,53.926503411624225),(79.26063009620081,55.85426815538253),(78.29674772432149,55.58793983698478),(80.42737427150402,47.87688086195143),(79.46349189962483,47.61055254355367),(62.418479522165164,109.29902434382004),(54.707420547131996,107.16839779663758),(71.75243292459157,45.47992599637123),(66.9330210651958,44.14828440438225),(66.66669274679789,45.1121667762614),(65.70281037491883,44.84583845786359),(67.56710860370339,38.098661854709455)]]
rand = (-85,161)

content = ""
starttime = datetime.datetime.now()
print "Path 44 of 109"
path = []
start = (102.09259043739901,-22.946080129340388)
goal = (118.52884440048415,-31.530298562054668)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-25-path-44.txt', 'a+')
f.write(pathStr)
f.close
