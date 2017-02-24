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

obstacleList = [[(0.06738096423955033,-0.17320062717863804),(0.7126318778242788,-2.0662540491712855),(1.6591585888206013,-1.743628592378923),(1.336533132028238,-0.7971018813825989),(2.2830598430245623,-0.4744764245902356),(2.928310756609289,-2.3675298465828827),(3.8748374676056128,-2.04490438979052),(3.5522120108132498,-1.0983776787941941),(3.8641626379152303,-0.4638015948998524),(5.2839527044097165,0.02013659028869666),(5.9185287883040605,-0.2918140368132861),(5.595903331511695,0.6547126741830409),(3.2295865540208863,-0.15185096779786975),(2.906961097228522,0.7946757431984539),(1.9604343862321982,0.4720502864060899),(1.325858302337854,0.7840009135080704),(0.841920117149308,2.203790980002556),(1.1538707442512877,2.8383670638969),(0.20734403325496442,2.5157416071045358),(1.0139076752358742,0.14942482961372594)],[(-3.1827587251530893,-2.7360098764832674),(-3.480452257188622,-1.781348386223915),(-3.1519682780767124,-1.155170875076472),(-1.7199760426876822,-0.7086305770231718),(-1.0937985315402385,-1.0371145561350834),(-1.3914920635757728,-0.0824530658757281),(-3.778145789224154,-0.8266868959645624),(-4.075839321259686,0.12797459429479296),(-5.03050081151904,-0.16971893774073976),(-5.481089284901087,0.7230128403736225),(-3.6956257286723364,1.62418978713771),(-3.136612721269643,1.626301079300136),(-3.023965602924139,1.403118134771547),(-3.357684373635811,0.9546409535519302),(-2.7986713662331204,0.9567522457143589),(-2.6860242478876106,0.7335693011857671),(-3.019743018599285,0.28509211996615225),(-2.4607300111965955,0.28720341212858225),(-2.3480828928510853,0.06402046759999136),(-2.6818016635627573,-0.38445671361961997),(-1.676422767102877,-0.15705118476617175),(-2.8028939505579804,2.074778260519752),(-1.0174303943292333,2.975955207283824),(-1.4680188677112953,3.8686869853982015),(-4.146214202054389,2.516921565252076),(-5.497979622200514,5.195116899595181),(-6.390711400314885,4.744528426213132),(-5.038945980168755,2.066333091870038),(-5.931677758283126,1.6157446184879944),(-6.400912152755323,2.5454187670195445),(-6.264214443809404,1.5548059585171838),(-7.254827252311767,1.4181082495712687),(-7.818482511035904,1.8450657993494888),(-8.023529074454785,3.330985012103028),(-7.596571524676565,3.894640270827164),(-8.587184333178921,3.757942561881255),(-8.245440060814124,1.281410540625357),(-9.236052869316485,1.1447128316794402),(-10.363363386764775,1.998627931235867),(-10.773456513602541,4.970466356742953),(-9.91954141404609,6.097776874191231),(-11.900767031050831,5.82438145629939),(-11.627371613158976,3.8431558392946723),(-11.806675960811608,3.3136750078070074),(-12.054329162937183,3.279500580570537),(-12.370331219535743,3.7406325575852275),(-12.549635567188377,3.2111517260975777),(-12.797288769313965,3.1769772988610985),(-13.11329082591252,3.6381092758757942),(-13.292595173565148,3.1086284443881382),(-13.540248375690735,3.074454017151651),(-13.856250432289276,3.535585994166368),(-13.967205925468951,2.5107987584275255),(-11.490673904213047,2.8525430307923276),(-11.217278486321206,0.8713174137876001),(-13.198504103325932,0.5979219958957707),(-13.061806394380005,-0.39269081260659844),(-10.08996796887293,0.01740231423114791),(-9.953270259927004,-0.9732104942711859),(-13.915721493936461,-1.5200013300548867),(-13.779023784990539,-2.510614138557244),(-12.788410976488182,-2.3739164296113238),(-11.694829304920844,-10.298818897630202),(-9.713603687916114,-10.025423479738379),(-10.807185359483451,-2.100521011719485),(-9.816572550981093,-1.9638233027735832),(-9.679874842035174,-2.9544361112759185),(-8.689262033532811,-2.8177384023300056),(-9.099355160370568,0.15410002317707772),(-6.127516734863486,0.5641931500148236),(-5.985162301778396,-0.4674124697762774),(-6.939823792037748,-0.7651060018118094),(-6.04674319593115,-3.6290904725898625),(-5.092081705671795,-3.3313969405543316),(-4.794388173636264,-4.286058430813684),(-5.122872152748174,-4.912235941961129),(-6.554864388137203,-5.358776240014428),(-7.181041899284648,-5.030292260902511),(-6.883348367249116,-5.984953751161864),(-4.49669464160073,-5.24071992107304),(-3.9013075775296637,-7.150042901591745),(-2.946646087270311,-6.8523493695562125),(-3.244339619305849,-5.897687879296861),(-2.915855640193932,-5.271510368149419),(-1.4838634048049058,-4.824970070096118),(-0.8576858936574618,-5.15345404920803),(-1.1553794256929963,-4.198792558948675),(-3.5420331513413768,-4.943026389037507),(-4.137420215412441,-3.0337034085187997)],[(0.30485264914701343,7.223377137429555),(0.3162210641979181,8.223312514911134),(7.315768706568983,8.143733609554795),(7.327137121619885,9.14366898703637),(0.32758947924882276,9.223247892392713),(0.33895789429973283,10.223183269874296),(-0.6609774831818511,10.2345516849252),(-0.570030162774618,18.234034704777844),(-1.5699655402562036,18.245403119828744),(-1.6609128606634267,10.245920099976107),(-2.660848238145009,10.25728851502701),(-2.6949534832977267,7.257482382582273)],[(6.337289540453605,0.005402478522092206),(7.425460223575983,-0.2512774084830601),(7.540422405398396,-0.7378816591330438),(6.682176085920847,-1.4544102734278532),(7.770346769043224,-1.7110901604330078),(7.885308950865643,-2.197694411082985),(7.027062631388089,-2.9142230253777957),(8.115233314510471,-3.1709029123829486),(8.230195496332886,-3.65750716303293),(7.371949176855333,-4.374035777327744),(9.433328361277676,-4.40079130068807),(9.203403997632844,-3.427582799388108),(11.047739237160911,-2.654001824806287),(11.434529724451817,-3.5761694445703283),(7.204366862051707,-6.241825490944446),(6.514784971174301,-6.085374809341848),(5.715088157262049,-4.816325950621818),(5.871538838864649,-4.126744059744394),(5.025506266384637,-4.659875269019222),(6.3583342895717,-6.774956700219271),(3.8202365721316243,-8.374350328043745),(3.2639374835729056,-8.429407789561154),(3.1306546812542195,-8.217899646441156),(3.4203881651755017,-7.739825898683737),(2.864089076616791,-7.794883360201141),(2.7308062742981103,-7.583375217081145),(3.0205397582193867,-7.105301469323716),(2.464240669660685,-7.1603589308411415),(2.3309578673419793,-6.948850787721125),(2.6206913512632863,-6.470777039963709),(1.64137597646456,-6.792400106118532),(2.9742039996515897,-8.907481537318567),(2.128171427171605,-9.440612746593404),(0.5287777993471146,-6.902515029153343),(0.4737203378296968,-6.3462159405946155),(0.6852284809497222,-6.212933138275921),(1.1633022287071269,-6.5026666221972125),(1.1082447671897242,-5.946367533638512),(1.319752910309723,-5.8130847313197975),(1.7978266580671463,-6.1028182152411),(1.7427691965497312,-5.546519126682382),(1.954277339669745,-5.413236324363679),(2.432351087427152,-5.702969808284972),(2.1107280212723447,-4.72365443348625),(-0.004353409927721152,-6.05648245667331),(-0.5374846192025355,-5.2104498841933),(-1.3835171916825484,-5.743581093468138),(-0.3172547731329063,-7.435646238428167),(-0.47370545473551573,-8.12522812930558),(-1.7427543134555412,-8.924924943217823),(-2.4323362043329473,-8.76847426161524),(-1.8992049950581213,-9.614506834095264),(0.21587643614191165,-8.281678810908181),(1.2821388546915866,-9.973743955868237),(-1.2559588627485105,-11.573137583692699),(-3.921614909122618,-7.342974721292606),(-4.7676474816026415,-7.87610593056742),(-4.234516272327826,-8.722138503047445),(-4.390966953930438,-9.411720393924842),(-5.660015812650465,-10.211417207837101),(-6.349597703527853,-10.054966526234548),(-5.816466494253029,-10.900999098714532),(-3.7013850630529737,-9.568171075527486),(-2.101991435228517,-12.106268792967528),(-4.004115969428063,-13.30490107509877),(12.594901186324549,-6.342672303862419),(12.981691673615448,-7.264839923626458),(12.71400310737889,-7.919318977153927),(11.330751677732858,-8.499504708090276),(10.676272624205389,-8.231816141853713),(11.063063111496291,-9.153983761617752),(13.368482160906362,-8.18700754339048),(14.14206313548819,-10.031342782918532),(15.064230755252211,-9.644552295627632),(14.677440267961309,-8.722384675863601),(15.59960788772533,-8.335594188572689),(16.759979349598062,-11.1020970478648),(-3.824881575575067,-24.073739221902745),(18.30714129876171,-14.790767526920922),(19.467512760634445,-17.55727038621301),(21.311848000162495,-16.783689411631183),(17.44394312725339,-7.562013213990874),(19.288278366781455,-6.788432239409075),(18.901487879490542,-5.866264619645053),(14.290649780670392,-7.800217056099573),(13.517068806088574,-5.955881816571515),(19.9722421444368,-3.2483484055351295),(19.58545165714589,-2.326180785771117),(13.130278318797664,-5.033714196807471),(11.969906856924933,-2.267211337515391),(14.812730641787795,-1.0748281162191773),(8.973479633988013,-2.454374298088141),(8.743555270343187,-1.48116579678818),(15.556014779442933,0.12830474872562414),(15.326090415798102,1.101513250025597),(14.352881914498141,0.8715888863807455),(14.122957550853304,1.8447973876807238),(29.69429357165271,5.523587205998033),(29.234444844363054,7.470004208597937),(13.663108823563643,3.791214390280661),(13.433184459918824,4.76442289158061),(12.459975958618866,4.534498527935785),(13.379673413198178,0.6416645227359228),(11.43325641059825,0.18181579544627455),(10.513558956018922,4.074649800646138),(9.54035045471897,3.844725437001301),(9.770274818363802,2.8715169357013353),(6.965611496286322,1.6951395941168663),(10.000199182008625,1.8983084344013774),(10.460047909298286,-0.04810856819855401),(8.51363090669836,-0.5079572954882164),(8.283706543053526,0.465251205811753)],[(-17.590801485775607,-5.9847515169828895),(-17.640282627445487,-6.983526575051042),(-19.63783274358179,-6.884564291711292),(-19.489389318572165,-3.8882391175068274),(-20.488164376640327,-3.8387579758369617),(-20.636607801649944,-6.835083150041424),(-21.635382859718096,-6.785602008371561),(-22.110029817917237,-6.261473908502529),(-22.035808105412436,-4.763311321400311),(-21.51168000554341,-4.288664363201157),(-22.51045506361156,-4.239183221531291),(-22.634157917786247,-6.736120866701667),(-24.631708033922557,-6.637158583361917),(-24.681189175592433,-7.635933641430101),(-23.68241411752428,-7.685414783099963),(-22.70837963029106,-8.234283453803926),(-22.733120201126003,-8.733670982837985),(-23.756635830029076,-9.18357737020221),(-22.78260134279587,-9.732446040906146),(-22.807341913630804,-10.231833569940221),(-23.830857542533895,-10.681739957304414),(-22.856823055300673,-11.230608628008385),(-22.881563626135602,-11.72999615704246),(-23.905079255038707,-12.17990254440666),(-21.932269709737334,-12.778252356780479),(-21.68486400138797,-7.784377066439719),(-19.68731388525166,-7.88333934977946),(-19.336369781227194,-10.904405094818856),(-19.123364275647507,-9.927354095846567),(-18.14631327667522,-10.140359601426267),(-18.359318782254935,-11.117410600398543),(-18.954347034530898,-11.49943334709485),(-20.419923532989323,-11.179925088725303),(-20.801946279685623,-10.584896836449325),(-21.01495178526532,-11.561947835421597),(-18.57232428783461,-12.094461599370831),(-18.785329793414288,-13.071512598343116),(-24.860641292827683,-12.770530563837264),(-23.883590293855445,-12.98353606941691),(-24.309601305014823,-14.937638067361483),(-26.307151421151136,-14.83867578402175),(-25.564934296103047,0.14295008700057465),(-27.562484412239343,0.24191237034030078),(-27.513003270569456,1.240687428408477),(-15.527702573751599,0.6469137283699924),(-15.428740290411865,2.644463844506305),(-27.414040987229747,3.2382375445447362),(-27.364559845559846,4.237012602612943),(-28.363334903628015,4.286493744282788),(-28.561259470307494,0.29139351201018915),(-30.558809586443783,0.39035579534997744),(-31.301026711491915,-14.591270075672439),(-33.422568554134926,-14.486165015602655),(-24.948617821753917,-17.86879106427834),(-25.80063984407264,-21.776995060167486),(-22.869486847155745,-22.416011576906598),(-20.95243729693856,-13.622552586156026),(-18.998335298994,-14.048563597315399),(-19.21134080457364,-15.025614596287692),(-18.234289805601396,-15.238620101867378),(-17.169262277702934,-10.35336510700596),(-16.192211278730632,-10.56637061258564),(-15.979205773150946,-9.58931961361336),(-18.910358770067827,-8.950303096874283),(-18.68853882718351,-7.932820491449322),(-17.689763769115356,-7.982301633119195),(-17.739244910785228,-8.98107669118735),(-16.740469852717066,-9.030557832857225),(-16.690988711047194,-8.03178277478907),(-15.697668412912428,-7.916393064184251),(-15.005330149283528,-13.876314852992849),(-14.012009851148775,-13.760925142388018),(-14.12739956175358,-12.767604844253263),(-13.688434267988606,-12.213249839883463),(-12.19845382078646,-12.040165273976255),(-11.644098816416681,-12.479130567741223),(-11.759488527021484,-11.485810269606453),(-14.2427892723584,-11.774284546118498),(-14.358178982963214,-10.780964247983725),(-13.91921368919824,-10.226609243613943),(-12.429233241996094,-10.053524677706704),(-11.8748782376263,-10.49248997147168),(-11.99026794823112,-9.499169673336924),(-14.47356869356803,-9.787643949848963),(-14.704348114777662,-7.801003353579439),(-13.669922966007794,-7.6808386707646035),(-14.646973964980077,-7.467833165184906),(-15.02899671167637,-6.872804912908923),(-14.70948845330684,-5.407228414450495),(-14.114460201030852,-5.025205667754205),(-15.091511200003135,-4.81220016217451),(-15.624024963952369,-7.254827659605223),(-16.641507569377325,-7.033007716720913),(-16.592026427707456,-6.034232658652761)],[(-17.80639084497395,11.728512902526854),(-18.72219798597349,12.130131229417263),(-17.91896133219268,13.96174551141634),(-13.339925627194976,11.95365387696429),(-13.082831220140408,11.29494114301931),(-13.68525871047602,9.921230431520026),(-14.343971444421012,9.664136024465455),(-13.42816430342147,9.262517697575028),(-12.424118486195434,11.552035550073875),(-11.508311345195903,11.150417223183462),(-11.106693018305482,12.066224364183006),(-12.93830730030458,12.86946101796382),(-13.195401707359146,13.528173751908813),(-12.59297421702353,14.901884463408129),(-11.934261483078544,15.158978870462668),(-12.850068624078101,15.560597197353108),(-13.85411444130411,13.271079344854238),(-15.685728723303187,14.074315998635067),(-16.043227712080363,14.504076947330159),(-15.942823130357755,14.733028732580035),(-15.384514978135382,14.761171354384718),(-15.742013966912562,15.1909323030798),(-15.64160938518994,15.419884088329688),(-15.083301232967568,15.44802671013437),(-15.440800221744752,15.877787658829464),(-15.340395640022148,16.106739444079366),(-14.78208748779976,16.134882065884028),(-15.597490047076707,16.765452178024322),(-16.60153586430273,14.475934325525476),(-17.517343005302262,14.877552652415876),(-17.11572467841188,15.79335979341542),(-18.031531819411413,16.194978120305816),(-18.433150146301813,15.279170979306283),(-19.34895728730135,15.680789306196692),(-17.742483979739728,19.34401787019486),(-18.658291120739275,19.745636197085247),(-19.461527774520082,17.914021915086177),(-20.120240508465056,17.656927508031607),(-21.493951219964366,18.259354998367215),(-21.751045627018925,18.9180677323122),(-22.152663953909347,18.002260591312627),(-19.8631461014105,16.99821477408662),(-20.264764428300893,16.082407633087087),(-21.18057156930044,16.484025959977494),(-21.582189896190837,15.568218818977964),(-18.834768473192224,14.363363838306741),(-19.63800512697303,12.531749556307652),(-20.55381226797257,12.933367883198068),(-20.95543059486297,12.017560742198516),(-21.87123773586251,12.419179069088944),(-22.12833214291709,13.077891803033923),(-21.525904652581463,14.451602514533228),(-20.867191918636507,14.708696921587778),(-21.782999059636037,15.110315248478194),(-22.78704487686205,12.820797395979332),(-26.450273440860215,14.427270703540986),(-26.851891767750622,13.511463562541433),(-23.18866320375247,11.904990254979806),(-25.999991491985345,5.49434026798305),(-25.084184350985776,5.092721941092645),(-22.272856062752922,11.503371928089406),(-21.357048921753385,11.101753601199013),(-21.758667248643793,10.185946460199464),(-19.011245825645183,8.981091479528239),(-18.609627498754783,9.89689862052779),(-17.950914764809795,10.153993027582343),(-16.577204053310506,9.551565537246717),(-16.32010964625593,8.892852803301748),(-15.918491319365513,9.808659944301294),(-18.208009171864365,10.812705761527319)],[(22.8990426301665,-15.312551994607503),(26.83452766434581,-16.542168823441056),(27.206595227715383,-18.50725552083847),(25.24150853031798,-18.87932308420803),(24.65721996512624,-18.481068300701082),(24.378169292599047,-17.00725327765303),(24.776424076106014,-16.422964712461294),(23.793880727407316,-16.60899849414608),(24.258965181619267,-19.06535686589283),(23.276421832920583,-19.251390647577608),(23.46245561460535,-20.23393399627632),(24.44499896330405,-20.04790021459152),(24.81706652667363,-22.012986911988936),(21.86943648057753,-22.5710882570433),(20.82006346117383,-19.716475101789587),(20.886893131878832,-22.75712203872808),(18.921806434481393,-23.129189602097636),(19.107840216166213,-24.111732950796366),(25.003100308358416,-22.99553026068766),(25.189134090043222,-23.97807360938632),(24.790879306536276,-24.56236217457808),(23.31706428348821,-24.841412847105246),(22.732775718296445,-24.443158063598304),(22.91880949998125,-25.42570141229701),(25.375167871728006,-24.960616958085016),(25.56120165341276,-25.943160306783746),(26.5437450021115,-25.757126525098947),(26.171677438741945,-23.792039827701537),(26.96818700575584,-22.623462697318057),(29.91581705185194,-22.065361352263682),(31.084394182235428,-22.8618709192776),(30.712326618865852,-20.896784221880196),(25.799609875372347,-21.82695313030414),(25.427542312002767,-19.861866432906744),(27.392629009400174,-19.489798869537157),(27.601564596165304,-20.59329883187388),(28.19801636843797,-18.684308186702268),(31.061502336195403,-19.578985845111298),(31.359728222331732,-18.624490522525512),(28.496242254574298,-17.729812864116475),(28.831181424998835,-16.657813797694587),(32.882311433431674,-15.890776865875123),(30.973320788260043,-15.294325093602396),(31.385071201040653,-12.32271583302072),(34.35668046162237,-12.734466245801336),(33.25867936087409,-20.658757607352555),(34.24921578106796,-20.796007744946138),(34.660966193848566,-17.824398484364377),(36.35264603052975,-16.54446906046383),(40.81005992140232,-17.162094679634748),(42.08998934530288,-18.853774516315877),(42.50173975808351,-15.882165255734096),(35.0727166066292,-14.852789223782667),(35.34721688181628,-12.871716383394855),(37.328289722204076,-13.14621665858196),(37.602789997391156,-11.165143818194135),(36.61225357719726,-11.02789368060063),(35.332324153296696,-9.336213843919456),(35.94994977246765,-4.878799953046872),(37.64162960914882,-3.598870529146364),(34.67002034856708,-3.1871201163657314),(33.64064431661555,-10.61614326782004),(31.659571476227732,-10.341642992632934),(31.808180782492315,-9.269127271951287),(30.018825465674233,-14.996099207466065),(28.946826399252355,-14.66116003704153),(28.179789467432897,-10.61003002860873),(27.88156358129656,-11.56452535119455),(24.868964670470966,-11.147095354078424),(27.58333769516021,-12.519020673780368),(26.95017263859933,-14.54551506278804),(24.98508594120193,-14.9175826261576),(24.400797376010182,-14.51932784265065),(24.121746703483012,-13.0455128196026),(24.52000148698997,-12.461224254410851),(23.53745813829128,-12.647258036095648),(24.002542592503225,-15.103616407842393)],[(7.8603581188025835,3.5173561489564005),(7.892843897195937,4.516828346769914),(8.408822885299369,5.000321556479997),(9.908031182019643,4.951592888889966),(10.391524391729725,4.4356139007865325),(10.42401017012308,5.43508609860004),(7.925329675589285,5.516300544583426),(7.957815453982634,6.515772742396944),(4.959398860542088,6.6132300775769925),(4.861941525362036,3.61481348413645),(5.861413723175558,3.582327705743098),(6.26369248690226,0.5676682231058781),(6.86088592098907,3.54984192734975)],[(12.607567058238677,11.236970866724903),(11.86083506251998,11.902095912539572),(12.525960108334646,12.648827908258273),(13.045205630171656,12.855912644663954),(13.23188862910133,12.68963138321029),(13.08600910512367,12.149984123897271),(13.605254626960681,12.357068860302952),(13.791937625890352,12.19078759884929),(13.646058101912699,11.651140339536267),(14.165303623749706,11.858225075941952),(14.351986622679382,11.691943814488283),(14.206107098701724,11.152296555175258),(15.057915143446069,11.732747289440304),(13.191085154149315,13.395559903976975),(14.521335245778644,14.889023895414379),(15.227263766545345,14.92982737036639),(16.347361760123384,13.932139801644393),(16.388165235075398,13.226211280877713),(17.053290280890067,13.972943276596409),(15.18646029159333,15.63575589113307),(15.851585337407986,16.382487886851784),(15.104853341689283,17.04761293266645),(12.444353158430612,14.060684949791643),(9.45742517555582,16.721185133050326),(8.792300129741147,15.974453137331615),(11.779228112615945,13.313952954072944),(11.114103066801281,12.567220958354245),(10.367371071082573,13.232346004168907),(9.7022460252679,12.48561400845021),(10.448978020986603,11.82048896263554),(10.489781495938624,11.114560441868854),(9.492093927216619,9.99446244829081),(8.786165406449943,9.953658973338776),(9.53289740216864,9.288533927524114),(11.195710016705307,11.155363916820871),(11.942442012424008,10.490238871006202)],[(-2.175725648876141,-27.14245144214361),(-1.9375789129177803,-28.11368062835681),(-2.9088080991309884,-28.35182736431519),(-5.308917701094391,-22.762598982994284),(-5.158442903744538,-24.756930283295873),(-21.113093306157005,-25.960728662094848),(-20.962618508807154,-27.955059962396437),(-5.007968106394668,-26.751261583597422),(-4.851266471557407,-28.82812083623193),(-8.736183216410232,-29.780707780065416),(-8.498036480451859,-30.751936966278656),(-3.6418905493858453,-29.56120328648676),(-3.4037438134274844,-30.53243247269996),(-18.943410792838755,-34.34278024803399),(-17.99082384900531,-38.227696992886735),(-12.163448731726096,-36.79881657713649),(-6.16317472681764,-50.77188753043872),(-7.307302800660035,-35.608082897344666),(-2.4511568695940467,-34.417349217552825),(-2.2130101336356205,-35.38857840376603),(-1.2417809474224217,-35.15043166780763),(-2.670661363172622,-29.3230565505284),(-1.6994321769594185,-29.084909814570022),(-1.4612854410010598,-30.05613900078325),(-0.49005625478783665,-29.817992264824884),(0.224383953087266,-32.73167982346451),(1.1956131393004688,-32.49353308750615),(0.48117293142536477,-29.579845528866528),(1.4524021176385902,-29.341698792908144),(1.2142553816802173,-28.370469606694947),(2.185484567893437,-28.132322870736584),(4.090658455560323,-35.9021563604423),(5.061887641773536,-35.66400962448392),(3.633007226023384,-29.836634507204625),(3.756741154597502,-29.2914832301084),(3.999548451150784,-29.231946546118856),(4.361429115683301,-29.658024455235836),(4.485163044257404,-29.112873178139623),(4.727970340810716,-29.05333649415004),(5.089851005343195,-29.479414403267057),(5.213584933917317,-28.934263126170862),(5.4563922304706205,-28.87472644218128),(5.818272895003106,-29.300804351298307),(5.822933455598039,-28.270038481095497),(3.394860490065006,-28.865405320991428),(3.1567137541066543,-27.894176134778192),(4.127942940319863,-27.656029398819836),(3.8897962043614944,-26.68480021260661),(0.9761086457218533,-27.399240420481718),(0.7379619097634902,-26.42801123426851),(-0.23326727644972323,-26.666157970226877),(-0.8379552375355086,-26.299616745099456),(-1.1951753414730435,-24.842772965779645),(-0.8286341163456494,-24.238085004693843),(-1.7998633025588484,-24.47623174065223),(-1.2044964626629282,-26.904304706185247)],[(-3.813358861440605,17.301566096295446),(-4.156364956416162,18.2408992740389),(1.4796341100445698,20.298935843892245),(1.1366280150690224,21.238269021635702),(-4.499371051391719,19.180232451782356),(-4.842377146367279,20.119565629525813),(-7.660376679597644,19.09054734459914),(-6.631358394670979,16.272547811368774)]]
rand = (-56,47)

content = ""
starttime = datetime.datetime.now()
print "Path 102 of 107"
path = []
start = (11.94323480076499,2.9147693683005116)
goal = (17.562376357768358,17.077471776114365)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-29-path-102.txt', 'a+')
f.write(pathStr)
f.close
