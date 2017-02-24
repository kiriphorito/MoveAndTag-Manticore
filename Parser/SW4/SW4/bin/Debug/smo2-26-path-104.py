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

obstacleList = [[(-0.07056816330694977,0.10195458983137207),(0.7909058264260836,0.6097562883451221),(1.433423787853705,-0.15651433567937728),(-2.3979293322687862,-3.3691041428174824),(-4.968001177979271,-0.3040216467194908),(-5.73427180200377,-0.9465396081471118),(-3.1641999562932877,-4.011622104245101),(-3.9304705803177806,-4.654140065672727),(-3.287952618890158,-5.420410689697225),(2.0759417492813257,-0.9227849597038755),(2.7450479168338493,-1.7207648315971045),(1.6523798161591172,1.117557986858872),(2.5138538058921505,1.625359685372622),(2.0060521073784003,2.4868336751056557),(1.1445781176453673,1.9790319765919056),(0.6367764191316169,2.8405059663249386),(3.221198388330717,4.363911061866189),(4.89426064681558,4.479508117337039),(5.275111920700893,3.8334026250372633),(4.363752209986655,2.4255945849668636),(6.036814468471517,2.541191640437712),(6.41766574235683,1.8950861481379369),(5.506306031642592,0.48727810806753646),(7.179368290127455,0.6028751635383858),(7.560219564012765,-0.04323032876138902),(6.648859853298529,-1.451038368831792),(7.607772385851852,-1.16733649201006),(7.891474262673581,-2.1262490245633847),(0.22017400224699646,-4.395864039137223),(0.5038758790687276,-5.3547765716905475),(8.175176139495312,-3.085161557116706),(8.742579893138775,-5.00298662222335),(6.944129504436834,-5.877958735709172),(6.385145393075619,-5.871895465492888),(6.275773878889896,-5.6470891669051415),(6.616014961879651,-5.203539839945947),(6.057030850518438,-5.197476569729662),(5.94765933633271,-4.972670271141917),(6.287900419322466,-4.529120944182721),(5.728916307961254,-4.523057673966433),(5.619544793775528,-4.2982513753786895),(5.959785876765285,-3.854702048419492),(4.951189168228585,-4.067381806574664),(6.04490431008586,-6.3154447924520865),(3.272806574640557,-7.664110415707053),(5.190631639747201,-7.096706662063597),(6.041737270212397,-9.973444259723564),(3.306850610963282,-11.304006156465416),(6.325439147034119,-10.93235679227689),(6.60914102385585,-11.891269324830212),(7.568053556409173,-11.607567448008481),(6.1495441723005255,-6.813004785241869),(9.026281769960494,-5.961899154776682),(9.30998364678223,-6.920811687330001),(10.268896179335554,-6.637109810508266),(8.566684918405175,-0.8836346151883325),(9.614133096382943,-0.573738765590314),(5.8056203575298175,5.887316157407438),(6.667094347262851,6.395117855921188),(6.1592926487491,7.256591845654223),(0.12897472061786713,3.701979956057972),(-0.37882697789588293,4.563453945791006),(-1.240300967628916,4.055652247277256),(-0.7324992691151667,3.1941782575442224),(-0.9093354147248089,2.5095404134208303),(-2.2015463993243585,1.7478378656502054),(-2.88618424344775,1.9246740112598473),(-2.3783825449340004,1.0632000215268131),(-0.22469757060141637,2.332704267811189),(0.2831041279123336,1.4712302780781554),(-0.5783698618206998,0.9634285795644054)],[(-3.7868235174468525,5.04511385955119),(-3.568124066468184,6.020906127475889),(-2.9708782070165043,6.3994525359489085),(-1.5071898051294523,6.071403359480906),(-1.1286433966564338,5.474157500029223),(-0.90994394567777,6.449949767953928),(-3.349424615489519,6.996698395400587),(-3.1307251645108467,7.9724906633252886),(-4.106517432435557,8.191190114303957),(-4.325216883414225,7.215397846379258),(-6.27680141926362,7.6527967483365895),(-5.402003615348969,11.55596582003539),(15.089634011069752,6.963277349483426),(15.308333462048402,7.939069617408096),(15.905579321500099,8.317616025881124),(17.369267723387132,7.989566849413157),(17.747814131860164,7.3923209899614575),(17.966513582838843,8.368113257886165),(15.527032913027075,8.914861885332805),(15.745732364005761,9.890654153257508),(3.0604328809846493,12.733747015980159),(8.309219704472632,36.15276144617296),(4.406050632773849,37.02755925008765),(-0.8427361907141444,13.608544819894826),(-4.7459052624129825,14.483342623809483),(-4.527205811434289,15.459134891734193),(-5.502998079359021,15.677834342712853),(-6.377795883273656,11.774665271014051),(-15.487975471063962,12.279271927934996),(-7.0338942362096555,8.847288467239956),(-7.252593687188323,7.871496199315255),(-10.179970490962429,8.527594552251239),(-10.398669941941087,7.551802284326557),(-4.543916334392888,6.239605578454557),(-4.762615785371553,5.263813310529854)],[(-13.187348561609296,-3.541182783988393),(-10.233807866030196,-3.015257542572738),(-10.409116279835407,-2.0307439773797213),(-8.440089149449339,-1.6801271497692567),(-7.770536774888869,-3.5647224956513575),(-10.43004170007187,-5.040199893962585),(-9.445528134878842,-4.864891480157368),(-8.919602893463173,-7.81843217573646),(-11.873143589042272,-8.344357417152132),(-11.697835175237048,-9.328870982345169),(-8.744294479657931,-8.802945740929479),(-8.568986065852734,-9.787459306122534),(-11.434872554529212,-10.805641330134701),(-10.434933830196794,-10.794571209066707),(-9.901754104826596,-13.788852321529967),(-9.434995105864367,-10.78350108799868),(-8.39367765204752,-10.771972871315578),(-8.218369238242298,-11.756486436508608),(-7.233855673049245,-11.58117802270337),(-8.4610145696858,-4.689583066352137),(-7.435760587608621,-4.507020168592382),(-6.403725101424997,-7.4119001271222205),(-6.754341929035429,-5.442872996736156),(-6.349739353341521,-4.862962007237028),(-4.872969005551958,-4.59999938652919),(-4.293058016052841,-5.004601962223123),(-4.468366429858083,-4.020088397030102),(-6.929650342840654,-4.458359431543128),(-7.455575584256322,-1.5048187359640424),(-6.4710620190632815,-1.329510322158833),(-6.646370432868495,-0.34499675696578835),(-10.584424693640635,-1.0462304121866621),(-10.759733107445857,-0.061716846993631336),(-13.713273803024952,-0.5876420884092899),(-13.537965389219734,-1.5721556536023344),(-15.506992519605813,-1.9227724812127596),(-19.889702864736336,22.690066648613094),(-20.874216429929444,22.514758234807825),(-21.208992617209553,23.457055907748934),(1.406151533375473,31.491684402474693),(-1.2720579648664927,39.030065786003085),(-21.060309096628384,31.999765853117946),(-22.535786494939643,34.659270778300964),(-22.002606769569393,31.66498966583774),(-23.887202115451593,30.995437291277174),(-24.94694452597816,33.97830418951379),(-22.8432435603154,22.164141407197423),(-24.812270690701478,21.81352457958695),(-23.059186552649223,11.968388927656655),(-51.71805143941418,1.786568687534929),(-21.30610241459709,2.1232532757262628),(-20.42956034557097,-2.7993145502388734),(-21.414073910764007,-2.9746229640440793),(-21.238765496958784,-3.959136529237112),(-19.269738366572724,-3.608519701626675),(-17.125402822381425,-4.242416439209262),(-16.950094408576213,-5.226930004402301),(-18.74381312515706,-6.562060397205778),(-16.599477580965768,-7.195957134788376),(-16.424169167160567,-8.180470699981399),(-18.217887883741405,-9.515601092784866),(-16.07355233955012,-10.149497830367462),(-15.89824392574489,-11.134011395560496),(-17.69196264232575,-12.469141788363993),(-16.69432565504607,-12.537847258389286),(-16.229859896418805,-13.071018487041831),(-16.332918101456812,-14.567473967961464),(-16.866089330109332,-15.031939726588668),(-15.868452342829599,-15.100645196613986),(-15.696688667766239,-12.606552728414659),(-13.578599967748357,-12.75242169833614),(-13.929216795358823,-10.783394567950065),(-13.524614219664937,-10.203483578450948),(-12.04784387187536,-9.940520957743109),(-11.467932882376243,-10.345123533437024),(-11.643241296181447,-9.360609968243981),(-14.10452520916406,-9.798881002757035),(-14.45514203677449,-7.829853872370965),(-10.517087776002352,-7.128620217150088),(-10.692396189807557,-6.14410665195704),(-14.630450450579707,-6.845340307177934),(-14.805758864384927,-5.860826741984906),(-10.86770460361278,-5.159593086764008),(-11.043013017418005,-4.175079521570989),(-14.981067278190142,-4.876313176791864),(-15.331684105800585,-2.9072860464058055),(-13.362656975414513,-2.5566692187953626)],[(16.13449633843679,-9.922952630249192),(16.496979767869167,-8.990962413891243),(18.360960200585076,-9.715929272756009),(18.64571359404786,-10.363166095651177),(18.101988449899284,-11.7611514201881),(17.454751627004118,-12.045904813650882),(18.386741843362067,-12.40838824308326),(19.292950416943025,-10.078412702188386),(20.224940633300978,-10.440896131620752),(20.509694026763764,-11.088132954515933),(19.965968882615186,-12.486118279052862),(19.318732059720023,-12.770871672515646),(20.25072227607796,-13.133355101948032),(21.156930849658927,-10.803379561053164),(22.088921066016876,-11.165862990485541),(22.45140449544926,-10.233872774127574),(21.519414279091308,-9.871389344695208),(20.76866577744955,-9.042910807083844),(20.949907492165735,-8.576915698904854),(22.06313942323988,-8.473404020158275),(21.31239092159811,-7.6449254825468955),(21.49363263631431,-7.1789303743679405),(22.606864567388456,-7.0754186956213445),(21.856116065746686,-6.246940158009986),(22.03735778046287,-5.780945049830997),(23.150589711537013,-5.677433371084399),(21.467850993537308,-4.486471404040684),(21.10536756410493,-5.418461620398631),(20.458130741209764,-5.70321501386141),(19.06014541667283,-5.159489869712841),(18.775392023210046,-4.512253046817675),(18.412908593777665,-5.444243263175617),(20.74288413467255,-6.350451836756585),(19.65543384637541,-9.146422485830447),(18.723443630017456,-8.783939056398046),(18.43869023655467,-8.136702233502888),(18.982415380703237,-6.73871690896596),(19.629652203598397,-6.453963515503174),(18.69766198724044,-6.0914800860707805),(17.791453413659504,-8.421455626965672),(16.859463197301558,-8.058972197533295),(17.22194662673392,-7.126981981175335),(16.289956410375982,-6.76449855174296),(15.914582159555103,-6.35025928293728),(16.005203016913196,-6.117261728847793),(16.561818982450262,-6.065505889474496),(16.1864447316294,-5.651266620668817),(16.277065588987476,-5.418269066579332),(16.83368155452454,-5.3665132272060285),(16.458307303703677,-4.952273958400352),(16.548928161061763,-4.7192764043108655),(17.105544126598836,-4.667520564937571),(16.26417476759899,-4.072039581415701),(15.35796619401804,-6.402015122310579),(14.425975977660082,-6.039531692878197),(14.063492548227709,-6.971521909236142),(13.416255725332542,-7.256275302698928),(12.018270400795618,-6.712550158550349),(11.733517007332843,-6.065313335655169),(11.371033577900445,-6.997303552013143),(13.701009118795326,-7.9035121255940926),(13.338525689362939,-8.83550234195204),(14.270515905720895,-9.197985771384424),(14.555269299183678,-9.845222594279589),(14.011544155035097,-11.243207918816514),(13.364307332139932,-11.527961312279306),(14.296297548497886,-11.890444741711686),(15.202506122078844,-9.56046920081681)],[(3.6509482886137663,-15.871558949225026),(4.603677509793864,-15.567738227236614),(6.426601841724385,-21.284113554317187),(4.5211433993641625,-21.891754998294033),(3.89286842777992,-21.5673007486982),(3.4371373447972897,-20.138206916928045),(3.7615915943931264,-19.509931945343794),(2.808862373213037,-19.813752667332206),(3.5684141781840597,-22.195575720282452),(2.615684957003978,-22.499396442270864),(1.2278581804486737,-19.79311913972478),(1.6629557358238845,-22.803217164259287),(0.7102265146437854,-23.1070378862477),(-1.7203392612635637,-15.485204116806914),(-2.673068482443666,-15.789024838795331),(-2.3692477604552353,-16.741754059975435),(-7.13289386635573,-18.26085766991755),(-7.6852136574428735,-18.17458572561673),(-7.7611688379399935,-17.936403420321682),(-7.360759407847039,-17.546310754032458),(-7.913079198934203,-17.46003880973167),(-7.9890343794313035,-17.22185650443661),(-7.58862494933835,-16.83176383814739),(-8.140944740425521,-16.745491893846605),(-8.216899920922604,-16.507309588551582),(-7.8164904908296675,-16.1172169222623),(-8.845174892506883,-16.182855338955726),(-8.085623087535833,-18.564678391905925),(-9.991081529896032,-19.172319835882792),(-9.38344008591918,-21.077778278243),(-1.7616063164783897,-18.64721250233565),(-1.4577855944899731,-19.599941723515748),(-9.57661750212824,-23.763422053181657),(-7.597189234434905,-23.477303663250794),(-7.066567368770213,-23.653202599229715),(-7.030802570028897,-23.90063113269138),(-7.489894838210847,-24.219589263635733),(-6.95927297254613,-24.395488199614732),(-6.92350817380479,-24.642916733076387),(-7.382600441986731,-24.961874864020785),(-6.851978576322062,-25.137773799999717),(-6.8162137775806935,-25.38520233346138),(-7.2753060457626555,-25.704160464405774),(-6.249827113174665,-25.80852980290196),(-6.607475100588261,-23.334244468285352),(-3.638332699048301,-22.905066883388997),(-3.209155114151958,-25.874209284928988),(-2.219440980305337,-25.73115008996352),(-2.6486185652016365,-22.762007688423573),(-0.5463234285247092,-22.458129387056033),(-0.24250270653630324,-23.41085860823613),(-1.19523192771641,-23.71467933022455),(-0.8914112057279837,-24.667408551404648),(0.06131801545211646,-24.363587829416222),(1.276600903405773,-28.174504714136614),(2.2293301245858586,-27.870683992148198),(1.0140472366321935,-24.059767107427795),(2.91950567899241,-23.452125663450964),(3.547780650576654,-23.776579913046803),(4.0035117335592725,-25.205673744816956),(3.679057483963457,-25.833948716401203),(4.6317867051435435,-25.530127994412787),(3.8722349001724927,-23.148304941462545),(6.730422563712821,-22.236842775497276),(7.034243285701205,-23.189571996677387),(7.986972506881322,-22.885751274688964),(6.467868896939224,-18.12210516878848),(11.880423502031384,-15.346451615677859),(5.860227452962382,-16.216646726428284),(5.556406730973955,-15.26391750524819),(6.509135952154064,-14.960096783259766),(5.597673786188803,-12.101909119719476),(2.7394861226485165,-13.013371285684725),(3.043306844636942,-13.96610050686483),(0.3370295420908467,-15.353927283420138),(3.3471275666253497,-14.918829728044932)],[(16.239646180684478,5.703478550720978),(14.415184432138751,3.3220211549035588),(16.796641827956186,1.4975594063578388),(17.4047957441381,2.291378538296985),(18.287794000450667,1.8220022720148403),(18.494604995465878,1.1458150107174854),(17.790540596042682,-0.17868237375136964),(17.114353334745292,-0.38549336876658913),(17.997351591057896,-0.8548696350487122),(19.17079225676324,1.352626005732703),(20.090330098046486,0.8638263557208488),(18.01294966032,3.0851976702361203),(18.621103576501906,3.8790168021752645),(17.827284444562757,4.487170718357173),(20.259900109290374,7.662447246113756),(19.466080977351236,8.270601162295645),(17.03346531262362,5.095324634539073)],[(38.784756266599146,13.214906413441511),(39.32843740079058,14.054198272287001),(40.01992389730904,14.20200363461403),(41.27886168557728,13.386481933326877),(41.4266670479043,12.694995436808416),(41.97034818209573,13.534287295653925),(39.872118534982015,14.893490131132486),(40.41579966917345,15.73278198997798),(39.57650781032796,16.276463124169407),(40.12018894451939,17.1157549830149),(42.638064521055846,15.484711580440617),(43.18174565524727,16.324003439286113),(40.6638700787108,17.95504684186039),(41.207551212902246,18.79433870070588),(44.5647186482842,16.61961416394015),(41.3026318431356,11.583863010867207),(42.141923701981085,11.040181876675778),(45.404010507129684,16.07593302974874),(46.243302365975175,15.532251895557295),(42.437534426635146,9.657208883638843),(43.27682628548063,9.113527749447417),(45.995231956437806,13.309987043674878),(52.709566827201726,8.960537970143434),(53.25324796139319,9.799829828988962),(46.538913090629244,14.149278902520377),(47.082594224820674,14.988570761365875),(47.92188608366617,14.44488962717445),(48.4655672178576,15.284181486019964),(46.78698350016662,16.371543754402754),(58.204287318186616,33.996672790158115),(64.07933033010511,30.190904850818157),(64.6230114642965,31.030196709663624),(58.74796845237809,34.83596464900364),(63.09741752590952,41.55029951976752),(58.900958231682054,44.26870519072467),(42.59052420593917,19.089949425359954),(41.75123234709366,19.633630559551374),(42.83859461547652,21.31221427724237),(41.99930275663102,21.855895411433796),(38.73721595148246,16.820144258360838),(37.897924092637,17.36382539255228),(37.35424295844555,16.5245335337068),(36.436989940680476,16.92283856932756),(37.6319050475428,19.674597622622798),(36.71465202977771,20.072902658243564),(36.31634699415695,19.155649640478483),(35.346093246322084,19.39773984022319),(35.42538081479631,20.39459162533742),(35.96345049159054,20.853373733657456),(37.45872816926188,20.73444238094611),(37.91751027758189,20.196372704151877),(37.996797846056104,21.19322448926614),(35.50466838327051,21.39144341045167),(35.587236971638795,22.42954618360008),(35.18893193601804,21.51229316583501),(34.53115290932512,21.252819174762863),(33.155273382677485,21.850276728193993),(32.895799391605344,22.50805575488692),(32.49749435598454,21.590802737121862),(34.790626900397264,20.59504014806995),(34.37583949848725,19.639830039967904),(33.36543542305135,19.891938238458273),(34.20472728189683,19.348257104266843),(33.11736501351398,17.66967338657588),(32.200111995748884,18.067978422196642),(31.841061745771547,18.496444194448287),(31.940638004676742,18.725757448889553),(32.498840772464455,18.755918185520432),(32.13979052248713,19.18438395777209),(32.23936678139229,19.413697212213357),(32.79756954918007,19.44385794884427),(32.43851929920268,19.87232372109587),(32.53809555810788,20.101636975537144),(33.096298325895596,20.131797712168012),(32.278621567035735,20.759416002230072),(31.282858977983828,18.466283457817415),(30.327648869881774,18.881070859727412),(32.57368387932252,16.83038152773038),(32.030002745131085,15.991089668884898),(32.869294603976584,15.447408534693459),(35.0440191407423,18.80457597007539),(35.91804195853617,18.238396622713392),(35.51973692291539,17.32114360494833),(34.56452681481339,17.73593100685837),(35.30301857682227,17.061668507941928),(35.33513320836845,16.35529137747927),(34.323739459993824,15.247553734465985),(33.61736232953118,15.215439102919742),(34.355854091540046,14.541176604003349),(36.04151033883112,16.38740600902549),(36.81056182425411,15.685241674861311),(36.26688069006268,14.845949816015834),(37.10617254890817,14.302268681824367),(34.93144801214242,10.945101246442466),(35.77073987098789,10.401420112251026),(37.94546440775367,13.758587547632953)],[(-11.092350819817845,-37.97954581771038),(-11.941748575063718,-37.45179260513535),(-10.358488937338606,-34.90359933939773),(-11.207886692584502,-34.375846126822694),(-12.79114633030959,-36.92403939256031),(-13.640544085555469,-36.39628617998527),(-14.168297298130502,-37.24568393523115),(-15.867092808622271,-36.190177510081114),(-16.18873735129311,-34.81302654226021),(-14.60547771356804,-32.264833276522594),(-13.22832674574713,-31.94318873385174),(-14.92712225623888,-30.887682308701706),(-15.454875468813906,-31.737080063947552),(-18.266945340839033,-30.578519303845386),(-15.982628681388942,-32.58647781919345),(-16.510381893963977,-33.435875574439315),(-16.98660793906293,-33.7286361489185),(-17.198957377874414,-33.59669784577472),(-17.147430210398383,-33.040060665008035),(-17.623656255497366,-33.3328212394872),(-17.836005694308824,-33.200882936343454),(-17.7844785268328,-32.6442457555768),(-18.260704571931747,-32.93700633005595),(-18.473054010743233,-32.805068026912195),(-18.421526843267227,-32.24843084614554),(-19.161629494653653,-32.965890298247565),(-17.038135106538988,-34.28527332968517),(-17.56588831911401,-35.13467108493104),(-18.415286074359884,-34.60691787235604),(-18.943039286934905,-35.456315627601846),(-18.09364153168903,-35.98406884017689),(-19.67690116941414,-38.53226210591453),(-24.773287700889405,-35.36574283046433),(-25.828794126039462,-37.0645383409561),(-20.73240759456419,-40.23105761640632),(-21.26016080713922,-41.08045537165219),(-24.65775182812273,-38.969442521352036),(-25.18550504069777,-39.81884027659789),(-23.486709530206003,-40.874346701748),(-25.803831050410324,-46.498486445798285),(-22.63731177496013,-41.40209991432304),(-21.78791401971426,-41.92985312689804),(-22.315667232289268,-42.77925088214398),(-21.466269477043408,-43.30700409471898),(-17.244243776443163,-36.511822052751924),(-14.69605051070552,-38.09508169047704),(-15.223803723280572,-38.9444794457229),(-12.675610457542927,-40.52773908344802),(-12.147857244967904,-39.67834132820214),(-7.051470713492662,-42.84486060365233),(-6.523717500917599,-41.99546284840645),(-8.222513011409372,-40.93995642325639),(-8.544157554080208,-39.56280545543547),(-8.01640434150519,-38.713407700189606),(-6.317608831013413,-39.768914125339656),(-5.674319745671746,-42.52321606098149),(-6.729826170821785,-44.22201157147323),(-8.106977138642707,-44.543656114144056),(-10.655170404380344,-42.96039647641899),(-10.976814947051201,-41.58324550859807),(-12.032321372201233,-43.282041019089824),(-7.7853325959719,-45.920807081964995),(-8.840839021121955,-47.61960259245674),(-11.595140956763789,-48.26289167779843),(-10.745743201517891,-48.79064489037347),(-11.904303961620002,-51.6027147623986),(-9.896345446272072,-49.3183981029485),(-8.197549935780213,-50.37390452809856),(-2.920017810029913,-41.87992697563977),(-2.070620054784065,-42.40768018821484),(-1.5428668422090297,-41.55828243296895),(-7.4886511289301545,-37.864009944943724),(-6.960897916355132,-37.014612189697836),(-5.583746948534204,-36.69296764702699),(-7.282542459025977,-35.63746122187692),(-9.921308521901128,-39.884449998106334),(-11.620104032392874,-38.828943572956256)],[(-9.644398482915914,7.149217493801799),(-10.572612438972095,6.777170652957085),(-11.222742837422548,7.0552542105628175),(-11.780813098689615,8.447575144647097),(-11.502729541083879,9.097705543097545),(-12.43094349714006,8.725658702252831),(-11.500826395028271,6.405123812112367),(-12.429040351084456,6.033076971267647),(-12.05699351023974,5.104863015211466),(-15.769849334464475,3.6166756518326046),(-15.397802493619743,2.6884616957764162),(-11.684946669395018,4.17664905915529),(-11.312899828550304,3.2484351030991054),(-8.52825796038176,4.3645756256332575)]]
rand = (-57,72)

content = ""
starttime = datetime.datetime.now()
print "Path 104 of 111"
path = []
start = (-39.59764667217985,28.596940403417015)
goal = (-40.14642010962771,35.835948165979836)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-26-path-104.txt', 'a+')
f.write(pathStr)
f.close
