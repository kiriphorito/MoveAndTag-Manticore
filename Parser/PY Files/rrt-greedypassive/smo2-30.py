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

obstacleList = [[(-0.04943873358795161,-0.12154559011750217),(0.9485039727294952,-0.18565763974789667),(1.4154193010730214,-0.7166850177218173),(1.3192512266274294,-2.2135990771979874),(0.7882238486535091,-2.6805144055415138),(1.7861665549709553,-2.7446264551719084),(1.946446679046942,-0.24976968937829114),(2.9443893853643885,-0.31388173900868566),(3.008501434994783,0.6840609673087623),(2.0105587286773363,0.7481730169391565),(2.07467077830773,1.7461157232566034),(6.054289089817501,2.1493998596866444),(6.317299650324139,1.1846069051819048),(3.422920786809919,0.39557522366199366),(3.6859313473165556,-0.5692177308427484),(4.650724301821295,-0.3062071703361086),(5.439755983341207,-3.200586033850328),(6.404548937845945,-2.937575473343692),(6.141538377339308,-1.972782518838955),(8.904411960600209,-0.7013543600666701),(5.878527816832671,-1.007989564334212),(5.615517256326036,-0.04319660982947189),(6.580310210830776,0.21981395067716508),(6.854204481078753,-0.7849033568919279),(7.049193667694944,2.2502208937941544),(8.126439065842805,2.3593861317916853),(2.1387828279381242,2.74405842957405),(2.202894877568519,3.742001135891497),(1.2049521712510711,3.806113185521891),(1.1408401216206774,2.8081704792044446),(0.6098127436467562,2.341255150860918),(-0.8871013158294137,2.4374232253065085),(-1.3540166441729398,2.9684506032804294),(-1.418128693803334,1.9705078969629826),(1.0767280719902836,1.8102277728869973),(1.0126160223598892,0.8122850665695506),(0.014673316042442479,0.8763971161999446)],[(-0.5961850621127621,-2.5368680453489945),(-2.6584931330631454,1.0334100139193456),(-0.4787457023912782,4.5332244573547875),(-4.049023761659607,2.4709163864043977),(-5.74668312875921,3.528249197443088),(-5.906764767774435,4.216997241977662),(-5.113765159495412,5.49024176730237),(-4.42501711496084,5.650323406317597),(-5.273846798510644,6.178989811836937),(-6.595512812309015,4.05691560296243),(-7.5488382050950475,4.650663817076282),(-6.548471851084192,2.9188245757599804),(-7.414391471742343,2.4186413987545587),(-9.91530735676948,6.74823950204531),(-9.948919040107647,7.3062451066257506),(-9.732439134943101,7.431290900877114),(-9.26586764127586,7.123376884799386),(-9.299479324614028,7.681382489379825),(-9.082999419449493,7.806428283631178),(-8.616427925782235,7.498514267553459),(-8.650039609120421,8.05651987213388),(-8.433559703955888,8.181565666385241),(-7.966988210288624,7.873651650307524),(-8.250691482129524,8.864617065217029),(-10.415490533774907,7.6141591227034695),(-11.415856887785747,9.345998364019776),(-12.281776508443892,8.845815187014361),(-8.2803110924005,1.9184582217491184),(-10.0121503337168,0.9180918677382941),(-9.511967156711377,0.0521722470801107),(-6.048288674078768,2.0529049551018272),(-5.486530134144657,1.0803857578079459),(-7.666277564816543,-2.4194286856275053),(-4.095999505548203,-0.3571206146771133),(-2.3983401384486007,-1.4144534257158008),(-2.9270065439679507,-2.2632831092656045),(-3.4035471676150735,-2.555531349660668),(-3.6157545885025213,-2.42336474828083),(-3.5636288066302972,-1.8667833051260931),(-4.040169430277421,-2.159031545521156),(-4.252376851164871,-2.0268649441413196),(-4.200251069292648,-1.4702835009865802),(-4.676791692939774,-1.76253174138165),(-4.888999113827223,-1.6303651400018104),(-4.8368733319549975,-1.0737836968470722),(-5.5777471583617935,-1.7904467790170289),(-3.455672949487296,-3.1121127928154015),(-3.9843393550066404,-3.9609424763652012),(-3.135509671456842,-4.489608881884549),(-2.6068432659374925,-3.6407791983347515),(-1.7580135823876937,-4.169445603854094),(-3.344012798945732,-6.7159346545034975),(-2.4951831153959234,-7.244601060022845),(-1.9665167098765841,-6.395771376473044),(-1.1176870263267826,-6.92443778199239),(-5.347018270481562,-13.715075250390788),(-4.49818858693174,-14.243741655910148),(-3.969522181412416,-13.394911972360338),(-3.280774136877818,-13.234830333345117),(-2.0075296115531382,-14.027829941624134),(-1.8474479725379105,-14.716577986158695),(-1.3187815670185552,-13.867748302608904),(-3.4408557758930693,-12.546082288810537),(-0.2688573427769829,-7.453104187511736),(0.5799723407728129,-7.981770593031083),(1.1086387462921632,-7.132940909481279),(-1.4378503043572353,-5.546941692923243),(-0.909183898837893,-4.698112009373442),(1.6373051518115136,-6.284111225931475),(2.1659715573308618,-5.435281542381674),(1.317141873781059,-4.906615136862328),(2.4787262485641914,-2.095792883453253),(0.4683121902312558,-4.377948731342984),(-2.0781768604181496,-2.7919495147849482),(-1.5495104548988023,-1.943119831235151)],[(4.719494973512677,5.626751800787084),(6.813976568750034,7.774575547562407),(6.098035319824921,8.472736079308193),(5.399874788079139,7.756794830383083),(3.9679922902289237,9.153115893874654),(3.941321214459941,11.274268564880995),(7.0830436073159895,14.496004185043965),(9.204196278322307,14.522675260812957),(7.056372531546984,16.61715685605031),(1.8201685434536037,11.24759748911201),(-0.32765520332170706,13.342079084349379),(-1.0258157350675212,12.626137835424247),(4.70171425633336,7.0408535814579745),(4.003553724587573,6.324912332532869)],[(18.955932279117278,2.3707807430262102),(19.59483067022873,3.1400718750660834),(20.298925431804406,3.2052682455302923),(21.45286212986422,2.2469206588631105),(21.518058500328422,1.5428258972874425),(22.156956891439876,2.312117029327309),(20.233729061340178,3.909363007105963),(20.872627452451642,4.678654139145843),(20.103336320411792,5.317552530257281),(20.03813994994757,6.021647291832959),(20.99648753661475,7.175583989892762),(21.700582298190422,7.240780360356962),(20.93129116615053,7.879678751468436),(19.33404518837189,5.956450921368743),(18.564754056332035,6.595349312480205),(17.925855665220574,5.826058180440325),(17.15656453318068,6.464956571551774),(17.091368162716485,7.169051333127459),(18.049715749383676,8.322988031187265),(18.753810510959333,8.388184401651472),(17.984519378919455,9.027082792762926),(16.387273401140803,7.103854962663231),(15.617982269100931,7.742753353774685),(14.979083877989478,6.9734622217348186),(15.748375010029342,6.3345638306233525),(15.813571380493558,5.630469069047697),(14.85522379382638,4.476532370987881),(14.15112903225071,4.411336000523654),(14.92042016429059,3.7724376094122163),(16.51766614206923,5.695665439511911),(17.286957274109106,5.056767048400451),(16.64805888299765,4.287475916360579),(17.41735001503753,3.648577525249122),(16.778451623926074,2.8792863932092487),(14.66616733919907,2.683697281816618),(11.204357245019617,5.558740041818154),(11.008768133627001,7.6710243265451545),(10.369869742515556,6.90173319450527),(9.665774980939881,6.836536824041066),(8.511838282880065,7.794884410708257),(8.446641912415846,8.498979172283882),(7.8077435213043955,7.729688040244028),(9.730971351404072,6.132442062465352),(9.09207296029263,5.3631509304255305),(14.861756450591702,0.5714129970896251),(12.306162886145877,-2.505751531069876),(13.075454018185747,-3.1446499221813218),(13.714352409297204,-2.3753587901414486),(45.242223509401434,-20.75995407791467),(17.54774275596594,2.240388002097796),(18.186641147077406,3.009679134137665)],[(-14.17978153692173,-11.858066470811073),(-14.901682663051275,-11.004334215304175),(-14.704569986074425,-10.544827313750977),(-13.588443505991183,-10.479545766151478),(-14.310344632120728,-9.62581351064458),(-14.113231955143865,-9.166306609091386),(-12.997105475060623,-9.10102506149189),(-13.719006601190182,-8.247292805984987),(-13.521893924213325,-7.787785904431792),(-12.40576744413008,-7.722504356832295),(-14.046682373366012,-6.474546747371695),(-14.440907727319729,-7.393560550478089),(-15.359921530426105,-6.999335196524386),(-15.720872093490874,-6.572469068770937),(-15.622315755002475,-6.342715617994348),(-15.064252514960835,-6.310074844194598),(-15.425203078025614,-5.883208716441149),(-15.326646739537194,-5.653455265664549),(-14.768583499495572,-5.620814491864805),(-15.129534062560337,-5.1939483641113515),(-15.03097772407191,-4.964194913334752),(-14.472914484030293,-4.931554139535003),(-15.293371948648247,-4.307575334804705),(-16.2789353335325,-6.605109842570688),(-17.19794913663891,-6.210884488617),(-17.460343361215255,-5.554264910086945),(-16.869005330284708,-4.1757442054273435),(-16.212385751754617,-3.9133499808509935),(-17.131399554861037,-3.5191246268973035),(-18.116962939745285,-5.816659134663289),(-19.954990545958093,-5.028208426755911),(-20.349215899911805,-5.947222229862303),(-19.430202096805424,-6.341447583816006),(-19.905755207225738,-8.284087344340197),(-20.87707508748783,-8.046310789130011),(-18.737086090596268,0.6955681332288943),(-15.745503331988056,0.470996099355693),(-15.28433487786555,-0.06502969939121428),(-15.396620894802153,-1.5608210786953158),(-15.932646693549058,-2.0219895328178232),(-14.935452440679647,-2.09684687744223),(-14.74830907911866,0.396138754731302),(-12.671390253813506,0.24022868222998817),(-18.26153298017594,2.6382078937531173),(-18.023756424965747,3.6095277740151896),(-19.96639618549003,4.085080884435543),(-22.81971484801204,-7.5707576787096755),(-24.762354608536256,-7.095204568289361),(-24.30701515753732,-1.0295087315065583),(-24.70124051149106,-1.9485225346129589),(-25.67256039175316,-1.7107459794028124),(-24.959230726122637,1.2032136613835078),(-25.930550606384706,1.4409902165937112),(-26.168327161594867,0.4696703363316015),(-26.530045409265522,0.04345453500306817),(-26.77287537933106,0.10289867380558704),(-26.89681707179149,0.6480027527391865),(-27.25853531946212,0.2217869514106834),(-27.501365289527605,0.2812310902132502),(-27.62530698198806,0.8263351691468159),(-27.98702522965867,0.40011936781832524),(-28.22985519972415,0.4595635066209258),(-28.353796892184647,1.004667585554479),(-28.834403417460322,0.09279184409490249),(-26.40610371680506,-0.5016495439305189),(-26.643880272015238,-1.472969424192625),(-27.655394597787083,-1.22535337430506),(-25.09546586544476,-2.8675363377193754),(-26.704994369060437,-6.619651457868975),(-30.59027389010884,-5.668545237028294),(-32.29513709542289,-4.22167224634591),(-32.05736054021273,-3.250352366083826),(-29.876944224478358,-2.754585596242011),(-31.581807429792384,-1.3077126055595976),(-31.344030874582216,-0.33639272529747544),(-29.163614558847858,0.15937404454427373),(-30.868477764161888,1.6062470352267209),(-30.6307012089517,2.577566915488857),(-28.450284893217365,3.0733336853305744),(-29.334934304858795,3.539590501347705),(-28.705979459117863,5.4381204068321995),(-27.442237083505134,6.0729079367039756),(-24.594442225278378,5.129475668092544),(-23.959654695406634,3.8657332924798027),(-23.330699849665653,5.7642631979643415),(-25.229229755150122,6.393218043705291),(-23.027887795056774,13.03807271290104),(-23.97715274779901,13.352550135771546),(-26.178494707892405,6.707695466575748),(-28.077024613376913,7.336650312316712),(-27.422042820397554,9.313743787213577),(-31.104233128141715,4.472104133381935),(-32.097787859055586,4.995759786433384),(-34.47555341115725,-4.717439016187589),(-43.21743233351615,-2.5774500192960144),(-43.93076199914668,-5.491409660082374),(-20.619084872856238,-11.198046985126485),(-20.876540417421204,-12.249755756408033),(-17.592174490592598,-7.129898291723392),(-16.67316068748622,-7.524123645677094),(-19.432738165162114,-13.957220267421853),(-18.51372436205571,-14.351445621375555),(-15.754146884379814,-7.918348999630789),(-14.835133081273417,-8.312574353584488),(-16.017809143134517,-11.069615762903679)],[(7.920498942666547,-10.685529071579568),(7.129897864812931,-10.073197514580665),(9.579224092808555,-6.910793203166207),(14.935162116929142,-9.794181467306029),(10.191555649807466,-6.120192125312598),(10.803887206806358,-5.329591047458977),(11.50535352423263,-5.2404562870316225),(12.691255141013059,-6.158953622529998),(12.780389901440394,-6.860419939956237),(13.392721458439333,-6.069818862102657),(11.416218763805276,-4.538989969605361),(12.028550320804179,-3.7483888917517634),(11.237949242950565,-3.1360573347528478),(6.339296786959321,-9.46086595758176),(5.548695709105701,-8.848534400582857),(4.936364152106797,-9.639135478436472),(5.7269652299604115,-10.25146703543537),(5.816099990387768,-10.952933352861628),(4.897602654889409,-12.138834969642057),(4.196136337463154,-12.2279697300694),(4.986737415316762,-12.840301287068305),(6.517566307814026,-10.863798592434275),(7.30816738566764,-11.476130149433178)],[(-9.398530488377414,-10.803233866142232),(-10.364090163906644,-11.063415558411087),(-10.97696084780569,-10.710726566780892),(-11.367233386208968,-9.262387053487052),(-11.014544394578794,-8.649516369588012),(-11.980104070108016,-8.909698061856862),(-11.329649839435874,-11.323597250679938),(-12.295209514965109,-11.583778942948795),(-12.035027822696257,-12.549338618478025),(-11.069468147167026,-12.289156926209174),(-10.80928645489817,-13.254716601738405),(-11.16197544652837,-13.86758728563744),(-12.610314959822198,-14.257859824040734),(-13.223185643721253,-13.905170832410533),(-12.963003951452375,-14.870730507939781),(-10.549104762629305,-14.220276277267638),(-10.028741378091587,-16.151395628326107),(-9.063181702562344,-15.891213936057238),(-9.58354508710007,-13.960094584998782),(-9.230856095469898,-13.347223901099737),(-7.782516582176032,-12.956951362696444),(-7.169645898276996,-13.309640354326636),(-7.429827590545847,-12.344080678797402),(-9.843726779368927,-12.994534909469552),(-10.10390847163779,-12.028975233940312),(-9.138348796108556,-11.768793541671462)],[(2.1077787075110574,-13.102117191897271),(-0.7498900551875591,-12.189029439982617),(-1.6629778071022034,-15.046698202681247),(-0.7104215528693305,-15.351060786652797),(-0.38632471773867616,-15.979520205754998),(-0.8428685936959917,-17.408354587104302),(-1.471328012798213,-17.73245142223497),(-0.5187717585653591,-18.036814006206512),(0.2421347013635311,-15.655423370624337),(1.1946909555964034,-15.959785954595889),(1.4990535395679503,-15.007229700363006),(3.4041660480337095,-15.615954868306115),(1.2736279602328884,-22.283848647936235),(2.226184214465763,-22.58821123190778),(2.834909382408815,-20.683098723442033),(3.7874656366417057,-20.98746130741358),(4.587840598888787,-21.768102018501573),(4.435659306903048,-22.24438014561801),(3.3309217606843564,-22.416295688762887),(4.131296722931493,-23.196936399850877),(3.9791154309457215,-23.67321452696731),(2.874377884727053,-23.845130070112198),(3.6747528469741333,-24.625770781200195),(3.522571554988388,-25.102048908316625),(2.4178340087697263,-25.273964451461527),(4.1707652252496885,-26.358967746521046),(5.692578145107441,-21.59618647535668),(10.455359416271817,-23.117999395214433),(10.759722000243332,-22.165443140981587),(3.139271966380373,-19.73054246920916),(4.356722302266567,-15.92031745227767),(9.119503573430935,-17.44213037213542),(9.423866157402488,-16.48957411790251),(1.8034161235395065,-14.05467344613014)],[(-2.8248693250708374,5.491909463623024),(-2.7409303791328545,6.606788045960501),(-2.2781911570102675,6.796188184052804),(-1.4366516587030769,6.060109877899931),(-1.3527127127650935,7.174988460237406),(-0.8899734906425087,7.36438859832971),(-0.04843399233531498,6.628310292176841),(0.03550495360266126,7.74318887451432),(0.4982441757252545,7.932589012606613),(1.3397836740324438,7.196510706453747),(1.0449223437858208,9.236867733036398),(0.11944389954065349,8.858067456851792),(-1.395757205197766,12.559981233832483),(-2.3212356494429534,12.181180957647882),(-0.8060345447045161,8.479267180667179),(-1.7315129889496967,8.10046690448258),(-2.28895228011844,8.142436377451574),(-2.383652349164592,8.373805988512864),(-2.015613196088154,8.794575737666463),(-2.5730524872568954,8.836545210635451),(-2.6677525563030384,9.067914821696746),(-2.299713403226617,9.488684570850342),(-2.8571526943953445,9.530654043819332),(-2.951852763441494,9.762023654880627),(-2.583813610365062,10.182793404034223),(-3.6039921236563788,10.035362738910909),(-2.656991433194877,7.72166662829798),(-3.582469877440044,7.34286635211337)]]
rand = (-49,50)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 195"
path = []
start = (10.212821270758674,10.49422742642497)
goal = (7.8557323752786985,10.073504700718686)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.8557323752786985,10.073504700718686)
goal = (12.57075036083264,14.416969432021709)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (12.57075036083264,14.416969432021709)
goal = (14.919918687005662,7.479575820053263)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 195"
path = []
start = (7.8557323752786985,10.073504700718686)
goal = (7.9702960043172055,9.488078972114458)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.9702960043172055,9.488078972114458)
goal = (6.269431254118764,9.667809932346302)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 195"
path = []
start = (6.269431254118764,9.667809932346302)
goal = (5.138724734511172,8.919495441069678)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 195"
path = []
start = (5.138724734511172,8.919495441069678)
goal = (6.512364943997987,6.561590641828012)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 195"
path = []
start = (6.512364943997987,6.561590641828012)
goal = (5.046636104944554,4.903170566044906)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 195"
path = []
start = (5.046636104944554,4.903170566044906)
goal = (4.383269016266567,5.2974204685044555)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.383269016266567,5.2974204685044555)
goal = (5.530307225195472,3.602162331420864)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 195"
path = []
start = (4.383269016266567,5.2974204685044555)
goal = (3.0570184779796605,4.77771044409776)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 195"
path = []
start = (5.530307225195472,3.602162331420864)
goal = (6.895948621305983,3.463234080960497)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.895948621305983,3.463234080960497)
goal = (4.441156980372988,2.721040374069961)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 195"
path = []
start = (6.895948621305983,3.463234080960497)
goal = (9.973577573396653,3.33983307700964)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 195"
path = []
start = (3.0570184779796605,4.77771044409776)
goal = (-0.9865071715793192,5.878392189476841)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 195"
path = []
start = (9.973577573396653,3.33983307700964)
goal = (10.032041067758634,2.4240925502449606)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 195"
path = []
start = (10.032041067758634,2.4240925502449606)
goal = (9.933281765062937,1.531080128505451)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 195"
path = []
start = (-0.9865071715793192,5.878392189476841)
goal = (-1.925904417142526,6.066014450645689)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 195"
path = []
start = (-1.925904417142526,6.066014450645689)
goal = (-3.279601307907022,4.6044994460124755)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 195"
path = []
start = (-3.279601307907022,4.6044994460124755)
goal = (-5.129882421497818,7.055538119663169)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.129882421497818,7.055538119663169)
goal = (-1.925076728024976,0.9788096897780605)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 195"
path = []
start = (-5.129882421497818,7.055538119663169)
goal = (-6.71294779378843,6.651786303229976)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 195"
path = []
start = (-6.71294779378843,6.651786303229976)
goal = (-6.4987659713756685,8.226843035237252)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.4987659713756685,8.226843035237252)
goal = (-6.772413470498144,2.8741448396400386)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 195"
path = []
start = (-6.4987659713756685,8.226843035237252)
goal = (-7.160954457616867,11.126468259266176)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 195"
path = []
start = (-7.160954457616867,11.126468259266176)
goal = (-4.201173618804852,12.05211776998106)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.201173618804852,12.05211776998106)
goal = (-7.277485947151014,14.291580270557262)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.277485947151014,14.291580270557262)
goal = (-11.598180064458688,10.897304033508817)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 195"
path = []
start = (-4.201173618804852,12.05211776998106)
goal = (-1.952557111885767,10.733693296913088)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 195"
path = []
start = (-1.952557111885767,10.733693296913088)
goal = (-0.9669746023970234,11.677863651804365)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 195"
path = []
start = (-0.9669746023970234,11.677863651804365)
goal = (-0.43692598167313434,13.44233411712758)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 195"
path = []
start = (-0.43692598167313434,13.44233411712758)
goal = (-2.432667322979519,15.634200703731672)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 195"
path = []
start = (-7.277485947151014,14.291580270557262)
goal = (-7.595832713115307,16.230347854398993)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 195"
path = []
start = (-6.772413470498144,2.8741448396400386)
goal = (-9.933175497093949,2.3826972812486034)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 195"
path = []
start = (-9.933175497093949,2.3826972812486034)
goal = (-8.080616969873468,-2.135637506462057)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 195"
path = []
start = (-1.925076728024976,0.9788096897780605)
goal = (-1.13719901670418,0.06901238869367887)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 195"
path = []
start = (-1.13719901670418,0.06901238869367887)
goal = (2.8081977634785957,-2.0783709373771124)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 195"
path = []
start = (-11.598180064458688,10.897304033508817)
goal = (-13.778877839162519,11.212473889901815)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 195"
path = []
start = (-13.778877839162519,11.212473889901815)
goal = (-14.757929891346265,8.681015849625076)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-14.757929891346265,8.681015849625076)
goal = (-14.926331998011637,16.43868059993853)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 195"
path = []
start = (-14.757929891346265,8.681015849625076)
goal = (-14.471533392969672,6.145569051532469)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 195"
path = []
start = (12.57075036083264,14.416969432021709)
goal = (18.51275054257855,16.022838643490914)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 195"
path = []
start = (-8.080616969873468,-2.135637506462057)
goal = (-6.772467415821637,-4.862550820950407)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 195"
path = []
start = (-6.772467415821637,-4.862550820950407)
goal = (-7.466402758093295,-8.327606934549298)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 195"
path = []
start = (-7.466402758093295,-8.327606934549298)
goal = (-8.182390799034799,-8.988615902411354)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.182390799034799,-8.988615902411354)
goal = (-3.972303513909317,-7.817685086215683)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 195"
path = []
start = (-3.972303513909317,-7.817685086215683)
goal = (-2.5679878821858892,-7.2426079035628845)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 195"
path = []
start = (-2.5679878821858892,-7.2426079035628845)
goal = (0.05182375678248974,-8.928723880576658)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 195"
path = []
start = (0.05182375678248974,-8.928723880576658)
goal = (3.7303679465689825,-10.984488230449577)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 195"
path = []
start = (3.7303679465689825,-10.984488230449577)
goal = (7.224723866151848,-11.859811342263859)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 195"
path = []
start = (7.224723866151848,-11.859811342263859)
goal = (7.718894629922822,-14.148423207574888)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 195"
path = []
start = (7.718894629922822,-14.148423207574888)
goal = (5.988625635181897,-14.185836996766836)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.988625635181897,-14.185836996766836)
goal = (10.166001260641998,-15.210871827995954)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 195"
path = []
start = (5.988625635181897,-14.185836996766836)
goal = (5.092249581180106,-17.25059216672983)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 195"
path = []
start = (10.166001260641998,-15.210871827995954)
goal = (14.021477337059181,-14.949188934054948)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 195"
path = []
start = (5.092249581180106,-17.25059216672983)
goal = (4.030572410092873,-17.843408724483577)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 195"
path = []
start = (4.030572410092873,-17.843408724483577)
goal = (4.877227354107674,-18.52498980440327)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (4.877227354107674,-18.52498980440327)
goal = (-0.5395431438006142,-16.24177910341725)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 195"
path = []
start = (4.877227354107674,-18.52498980440327)
goal = (7.096993941689121,-22.399976876487358)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 195"
path = []
start = (14.021477337059181,-14.949188934054948)
goal = (16.491457725497447,-15.09221386527947)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 195"
path = []
start = (16.491457725497447,-15.09221386527947)
goal = (15.614008691067902,-12.243858753159325)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 195"
path = []
start = (15.614008691067902,-12.243858753159325)
goal = (16.41908848167965,-11.980086634980035)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (16.41908848167965,-11.980086634980035)
goal = (14.529264262025912,-9.855557124713929)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 195"
path = []
start = (14.529264262025912,-9.855557124713929)
goal = (12.737485315941953,-9.020849834966032)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (12.737485315941953,-9.020849834966032)
goal = (18.520113733722916,-7.157649711997546)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 195"
path = []
start = (12.737485315941953,-9.020849834966032)
goal = (11.669425450400297,-10.270671147928397)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (11.669425450400297,-10.270671147928397)
goal = (11.786506065731885,-6.009975049057708)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 195"
path = []
start = (7.096993941689121,-22.399976876487358)
goal = (7.713384364013216,-25.351230949875262)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 195"
path = []
start = (18.520113733722916,-7.157649711997546)
goal = (19.251981545835264,-5.923340779680544)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 195"
path = []
start = (-0.5395431438006142,-16.24177910341725)
goal = (-4.094400941641496,-16.581570947650395)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 195"
path = []
start = (-4.094400941641496,-16.581570947650395)
goal = (-4.849537882117382,-16.139168688228985)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 195"
path = []
start = (-4.849537882117382,-16.139168688228985)
goal = (-6.790693504254776,-18.6553097554484)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 195"
path = []
start = (-6.790693504254776,-18.6553097554484)
goal = (-8.012427493849628,-17.72009418654875)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-8.012427493849628,-17.72009418654875)
goal = (-5.541623831598315,-21.122521348190748)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 195"
path = []
start = (-8.012427493849628,-17.72009418654875)
goal = (-10.461016299774386,-20.479494376239025)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 195"
path = []
start = (-5.541623831598315,-21.122521348190748)
goal = (-5.172422916555497,-22.05404532483992)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.172422916555497,-22.05404532483992)
goal = (-4.264772235063376,-20.190880123156447)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 195"
path = []
start = (-5.172422916555497,-22.05404532483992)
goal = (-8.20487662971162,-23.67350739682582)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 195"
path = []
start = (-4.264772235063376,-20.190880123156447)
goal = (-0.904485196689059,-20.627475228607302)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 195"
path = []
start = (-0.904485196689059,-20.627475228607302)
goal = (-0.34365707530591294,-20.44623615351641)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.34365707530591294,-20.44623615351641)
goal = (-1.6458832427078889,-23.96528346652378)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 195"
path = []
start = (-8.20487662971162,-23.67350739682582)
goal = (-10.408637704935757,-25.015932926441934)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 195"
path = []
start = (-14.926331998011637,16.43868059993853)
goal = (-18.98509073225753,13.208366321304936)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 195"
path = []
start = (-18.98509073225753,13.208366321304936)
goal = (-22.866400901494845,10.366071219659908)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 195"
path = []
start = (-22.866400901494845,10.366071219659908)
goal = (-23.705596900655387,10.796017325459509)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 195"
path = []
start = (-23.705596900655387,10.796017325459509)
goal = (-26.317692912960407,11.760777476163014)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 195"
path = []
start = (-26.317692912960407,11.760777476163014)
goal = (-27.296502514296723,8.811871747705709)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-27.296502514296723,8.811871747705709)
goal = (-28.479279320590525,14.679493508149214)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 195"
path = []
start = (-27.296502514296723,8.811871747705709)
goal = (-28.860160996839806,4.139646652845368)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 195"
path = []
start = (-28.479279320590525,14.679493508149214)
goal = (-30.100999363823828,15.64240812862301)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 195"
path = []
start = (-30.100999363823828,15.64240812862301)
goal = (-30.001441079882504,15.830099031794589)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-30.001441079882504,15.830099031794589)
goal = (-31.21959010002003,12.825284346473019)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 195"
path = []
start = (-30.001441079882504,15.830099031794589)
goal = (-29.744468980452588,16.10862776604983)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 195"
path = []
start = (-31.21959010002003,12.825284346473019)
goal = (-31.31817207227071,9.922508849943377)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 195"
path = []
start = (-31.31817207227071,9.922508849943377)
goal = (-33.11306837361236,9.75911412406775)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 195"
path = []
start = (-33.11306837361236,9.75911412406775)
goal = (-33.86832430916721,9.888842535385379)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-33.86832430916721,9.888842535385379)
goal = (-32.51787532003323,7.594480265043423)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 195"
path = []
start = (-33.86832430916721,9.888842535385379)
goal = (-34.471809550745895,11.13390197413976)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 195"
path = []
start = (-34.471809550745895,11.13390197413976)
goal = (-36.96332600246211,9.394311704290125)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 195"
path = []
start = (-36.96332600246211,9.394311704290125)
goal = (-38.490771480679996,11.06943925006155)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-38.490771480679996,11.06943925006155)
goal = (-40.18352287487914,6.431467727395969)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 195"
path = []
start = (-38.490771480679996,11.06943925006155)
goal = (-39.29263281257359,13.12279093444507)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 195"
path = []
start = (-39.29263281257359,13.12279093444507)
goal = (-39.13499711128948,13.444443794254447)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-39.13499711128948,13.444443794254447)
goal = (-40.63876130631674,12.576306815050692)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 195"
path = []
start = (-40.63876130631674,12.576306815050692)
goal = (-41.45993530273332,15.23121398469916)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 195"
path = []
start = (-40.18352287487914,6.431467727395969)
goal = (-43.1696717742945,4.6673243512796745)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-43.1696717742945,4.6673243512796745)
goal = (-39.00250565798356,3.047140677678591)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 195"
path = []
start = (-39.00250565798356,3.047140677678591)
goal = (-38.80742598102192,0.8505898648925161)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 195"
path = []
start = (-38.80742598102192,0.8505898648925161)
goal = (-37.229751996112796,1.42101132408445)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-37.229751996112796,1.42101132408445)
goal = (-40.33410540857776,-0.7771440609714304)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-40.33410540857776,-0.7771440609714304)
goal = (-41.09871907906896,1.6884363267902174)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 195"
path = []
start = (-37.229751996112796,1.42101132408445)
goal = (-34.51029552429867,-0.04971189554903788)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 195"
path = []
start = (-40.33410540857776,-0.7771440609714304)
goal = (-38.715733868150096,-2.3491008833022704)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 195"
path = []
start = (-38.715733868150096,-2.3491008833022704)
goal = (-38.6424936177943,-2.531162791719545)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 195"
path = []
start = (-38.6424936177943,-2.531162791719545)
goal = (-38.602906251309165,-3.272356817559828)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 195"
path = []
start = (-38.602906251309165,-3.272356817559828)
goal = (-35.843895779691096,-3.4071630733883467)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 195"
path = []
start = (-35.843895779691096,-3.4071630733883467)
goal = (-34.44808044925506,-8.22346925983868)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 195"
path = []
start = (-34.51029552429867,-0.04971189554903788)
goal = (-34.52167060879689,-0.33832599833133514)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 195"
path = []
start = (-28.860160996839806,4.139646652845368)
goal = (-27.111364228002767,1.4064048734532513)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 195"
path = []
start = (-27.111364228002767,1.4064048734532513)
goal = (-26.91500424523006,0.6966982919923481)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-26.91500424523006,0.6966982919923481)
goal = (-23.924165714341665,2.195425159828396)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 195"
path = []
start = (-26.91500424523006,0.6966982919923481)
goal = (-27.697937648448608,-1.84768752805142)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 195"
path = []
start = (-27.697937648448608,-1.84768752805142)
goal = (-28.57877743609499,-5.844820527663128)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 195"
path = []
start = (-23.924165714341665,2.195425159828396)
goal = (-24.660080250887756,4.433130927948461)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.660080250887756,4.433130927948461)
goal = (-22.638966298553616,-0.08322145848262608)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 195"
path = []
start = (-24.660080250887756,4.433130927948461)
goal = (-22.44125845150049,4.408777828444922)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 195"
path = []
start = (-22.44125845150049,4.408777828444922)
goal = (-22.57851375203105,5.321918392818265)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 195"
path = []
start = (-22.638966298553616,-0.08322145848262608)
goal = (-23.598450650844303,-2.1174130683886148)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-23.598450650844303,-2.1174130683886148)
goal = (-16.001282775045393,-0.637375778572121)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 195"
path = []
start = (-23.598450650844303,-2.1174130683886148)
goal = (-20.28670192657453,-5.743342988566056)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 195"
path = []
start = (-34.44808044925506,-8.22346925983868)
goal = (-35.940180255718246,-10.298371288819176)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-35.940180255718246,-10.298371288819176)
goal = (-32.9774352364294,-10.50466949660009)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 195"
path = []
start = (-35.940180255718246,-10.298371288819176)
goal = (-40.19084674282647,-10.115758525377657)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 195"
path = []
start = (-32.9774352364294,-10.50466949660009)
goal = (-28.131182676552353,-11.1979250617538)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 195"
path = []
start = (-40.19084674282647,-10.115758525377657)
goal = (-43.78882140311762,-10.612931379504376)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 195"
path = []
start = (-43.78882140311762,-10.612931379504376)
goal = (-43.07745850072414,-13.856749134282355)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 195"
path = []
start = (-43.07745850072414,-13.856749134282355)
goal = (-40.07675414633988,-16.184352453548478)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 195"
path = []
start = (-40.07675414633988,-16.184352453548478)
goal = (-38.0376513848437,-17.11484502077137)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 195"
path = []
start = (-38.0376513848437,-17.11484502077137)
goal = (-38.05681936157336,-18.83052875919799)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 195"
path = []
start = (-38.05681936157336,-18.83052875919799)
goal = (-38.62041912338496,-20.542531310649437)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 195"
path = []
start = (-38.62041912338496,-20.542531310649437)
goal = (-39.48883875652807,-22.252865892474805)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-39.48883875652807,-22.252865892474805)
goal = (-35.52240377793544,-21.759749754568453)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 195"
path = []
start = (-39.48883875652807,-22.252865892474805)
goal = (-41.88349291217588,-22.03724765906492)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 195"
path = []
start = (-35.52240377793544,-21.759749754568453)
goal = (-34.22449109161483,-24.247171490431203)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-34.22449109161483,-24.247171490431203)
goal = (-33.955274689428485,-19.234656862646037)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 195"
path = []
start = (-33.955274689428485,-19.234656862646037)
goal = (-33.521326035311375,-19.3410181850718)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 195"
path = []
start = (-33.521326035311375,-19.3410181850718)
goal = (-33.4291778842217,-18.992758101808047)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 195"
path = []
start = (-33.4291778842217,-18.992758101808047)
goal = (-33.87649080964991,-17.522270489856126)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 195"
path = []
start = (-28.131182676552353,-11.1979250617538)
goal = (-26.07236380889445,-12.474519439013562)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 195"
path = []
start = (-26.07236380889445,-12.474519439013562)
goal = (-22.03263510985645,-11.303670225329114)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 195"
path = []
start = (-22.03263510985645,-11.303670225329114)
goal = (-21.39979850406873,-12.171577326654448)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 195"
path = []
start = (-21.39979850406873,-12.171577326654448)
goal = (-19.89646498566484,-11.75253194121706)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 195"
path = []
start = (-19.89646498566484,-11.75253194121706)
goal = (-19.2502572912381,-16.1843460900118)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 195"
path = []
start = (-19.2502572912381,-16.1843460900118)
goal = (-20.301999970105612,-18.071879048456022)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-20.301999970105612,-18.071879048456022)
goal = (-15.229286002947067,-15.231796684569913)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 195"
path = []
start = (-20.301999970105612,-18.071879048456022)
goal = (-21.24529182394962,-17.1249362891626)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-21.24529182394962,-17.1249362891626)
goal = (-16.65020764874104,-20.567597474614697)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 195"
path = []
start = (-21.24529182394962,-17.1249362891626)
goal = (-22.436852367938414,-16.588853634933848)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 124 of 195"
path = []
start = (-22.436852367938414,-16.588853634933848)
goal = (-24.603260390635533,-17.649731816440784)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 125 of 195"
path = []
start = (-24.603260390635533,-17.649731816440784)
goal = (-24.66434777147841,-19.589109975997665)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.66434777147841,-19.589109975997665)
goal = (-26.95464286452093,-17.21292503571579)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 126 of 195"
path = []
start = (-26.95464286452093,-17.21292503571579)
goal = (-28.480482831423416,-20.53342703987993)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 127 of 195"
path = []
start = (-28.480482831423416,-20.53342703987993)
goal = (-27.450856352174743,-25.480437575957488)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 128 of 195"
path = []
start = (-15.229286002947067,-15.231796684569913)
goal = (-15.10436895167814,-14.152494815340297)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 129 of 195"
path = []
start = (-27.450856352174743,-25.480437575957488)
goal = (-27.31003627694017,-25.886629456807512)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 130 of 195"
path = []
start = (14.919918687005662,7.479575820053263)
goal = (18.580903119888397,6.734297834369258)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 131 of 195"
path = []
start = (18.580903119888397,6.734297834369258)
goal = (22.30863551225623,4.986107079280448)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 132 of 195"
path = []
start = (22.30863551225623,4.986107079280448)
goal = (24.841517089843023,6.725288320016283)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 133 of 195"
path = []
start = (24.841517089843023,6.725288320016283)
goal = (22.82904404542903,8.4989913208802)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (22.82904404542903,8.4989913208802)
goal = (25.780825539691918,3.0962714963659934)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 134 of 195"
path = []
start = (25.780825539691918,3.0962714963659934)
goal = (28.32463313138453,3.1693392539519856)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (28.32463313138453,3.1693392539519856)
goal = (27.008885489782102,0.3409174476115773)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 135 of 195"
path = []
start = (28.32463313138453,3.1693392539519856)
goal = (28.66864571457893,3.8111454725581915)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 136 of 195"
path = []
start = (28.66864571457893,3.8111454725581915)
goal = (31.326997268838497,2.737043001444764)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 137 of 195"
path = []
start = (31.326997268838497,2.737043001444764)
goal = (32.91193817058047,1.3243647148524467)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.91193817058047,1.3243647148524467)
goal = (33.22140241702736,5.791348777793452)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 138 of 195"
path = []
start = (32.91193817058047,1.3243647148524467)
goal = (36.90294152114433,0.583417523461204)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 139 of 195"
path = []
start = (27.008885489782102,0.3409174476115773)
goal = (28.94636620212556,-0.7092620921750878)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 140 of 195"
path = []
start = (28.94636620212556,-0.7092620921750878)
goal = (30.14737787825645,-1.8167311472341758)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 141 of 195"
path = []
start = (30.14737787825645,-1.8167311472341758)
goal = (30.59707890441088,-2.6197792736117975)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 142 of 195"
path = []
start = (30.59707890441088,-2.6197792736117975)
goal = (32.449149525023316,-3.8162996729437424)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 143 of 195"
path = []
start = (32.449149525023316,-3.8162996729437424)
goal = (30.639236410596467,-6.486359782706593)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (30.639236410596467,-6.486359782706593)
goal = (36.24210753939313,-3.582423998446856)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 144 of 195"
path = []
start = (33.22140241702736,5.791348777793452)
goal = (34.57769796191677,6.978235332434309)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 145 of 195"
path = []
start = (34.57769796191677,6.978235332434309)
goal = (33.59389456875155,7.88359968598369)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 146 of 195"
path = []
start = (33.59389456875155,7.88359968598369)
goal = (34.59475786275358,9.240481166060484)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (34.59475786275358,9.240481166060484)
goal = (31.59875481597672,7.5025046162661155)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 147 of 195"
path = []
start = (34.59475786275358,9.240481166060484)
goal = (33.84953349040627,10.409339074710132)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 148 of 195"
path = []
start = (33.84953349040627,10.409339074710132)
goal = (33.47508117665906,12.55434499791292)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 149 of 195"
path = []
start = (31.59875481597672,7.5025046162661155)
goal = (29.454184236357747,8.401357420662258)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 150 of 195"
path = []
start = (33.47508117665906,12.55434499791292)
goal = (32.5669828513314,12.240856353167572)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.5669828513314,12.240856353167572)
goal = (37.3263167248429,13.697763220662786)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 151 of 195"
path = []
start = (32.5669828513314,12.240856353167572)
goal = (32.62696208490853,13.506203797379698)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.62696208490853,13.506203797379698)
goal = (30.98192912254595,12.34087152096135)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 152 of 195"
path = []
start = (29.454184236357747,8.401357420662258)
goal = (27.993407489757388,10.324028817883928)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 153 of 195"
path = []
start = (27.993407489757388,10.324028817883928)
goal = (26.952675519950333,12.708178277513206)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 154 of 195"
path = []
start = (26.952675519950333,12.708178277513206)
goal = (27.08247483882225,15.154896275640425)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 155 of 195"
path = []
start = (37.3263167248429,13.697763220662786)
goal = (38.77157443448429,16.408326516063063)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 156 of 195"
path = []
start = (38.77157443448429,16.408326516063063)
goal = (40.4707163561595,14.334318897524206)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 157 of 195"
path = []
start = (40.4707163561595,14.334318897524206)
goal = (42.13563010853818,15.728862081973922)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (42.13563010853818,15.728862081973922)
goal = (43.314097857352806,11.59585009181415)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 158 of 195"
path = []
start = (43.314097857352806,11.59585009181415)
goal = (44.05822720543844,10.294632055449277)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 159 of 195"
path = []
start = (36.90294152114433,0.583417523461204)
goal = (39.14207524960469,1.2660731183589888)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 160 of 195"
path = []
start = (39.14207524960469,1.2660731183589888)
goal = (40.48302691954089,1.0854387233088403)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (40.48302691954089,1.0854387233088403)
goal = (38.46290588842139,4.939000090899388)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 161 of 195"
path = []
start = (40.48302691954089,1.0854387233088403)
goal = (40.54709462893611,-2.4287081709736356)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (40.54709462893611,-2.4287081709736356)
goal = (44.22858698632349,0.16585844123603977)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 162 of 195"
path = []
start = (40.54709462893611,-2.4287081709736356)
goal = (41.02514365393239,-3.5830206268635294)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 163 of 195"
path = []
start = (41.02514365393239,-3.5830206268635294)
goal = (40.32026682250394,-4.670143537659939)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 164 of 195"
path = []
start = (40.32026682250394,-4.670143537659939)
goal = (40.20036326421652,-8.759562894803235)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 165 of 195"
path = []
start = (38.46290588842139,4.939000090899388)
goal = (39.00990752373687,7.12136602893505)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 166 of 195"
path = []
start = (39.00990752373687,7.12136602893505)
goal = (40.00229389707867,7.658609977497818)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 167 of 195"
path = []
start = (44.22858698632349,0.16585844123603977)
goal = (45.0955400028991,0.0566476966257774)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 168 of 195"
path = []
start = (40.20036326421652,-8.759562894803235)
goal = (41.028803113534465,-10.06167991438394)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (41.028803113534465,-10.06167991438394)
goal = (37.400845792661016,-8.761683805848001)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 169 of 195"
path = []
start = (41.028803113534465,-10.06167991438394)
goal = (43.53026029823785,-10.45828438206356)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (43.53026029823785,-10.45828438206356)
goal = (38.79255180540615,-14.766729229854384)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 170 of 195"
path = []
start = (37.400845792661016,-8.761683805848001)
goal = (35.5876930308969,-9.018119806661751)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 171 of 195"
path = []
start = (35.5876930308969,-9.018119806661751)
goal = (32.138307426345335,-13.32108686185142)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 172 of 195"
path = []
start = (38.79255180540615,-14.766729229854384)
goal = (40.90197719353874,-15.945682412278481)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 173 of 195"
path = []
start = (32.138307426345335,-13.32108686185142)
goal = (30.465008376278206,-15.037914720263746)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 174 of 195"
path = []
start = (30.465008376278206,-15.037914720263746)
goal = (32.829623035725206,-16.416245422657802)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (32.829623035725206,-16.416245422657802)
goal = (27.004798582219223,-13.595526392738197)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 175 of 195"
path = []
start = (32.829623035725206,-16.416245422657802)
goal = (31.295918036764988,-18.5831643887473)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 176 of 195"
path = []
start = (31.295918036764988,-18.5831643887473)
goal = (29.67499672545445,-20.377035827727134)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 177 of 195"
path = []
start = (29.67499672545445,-20.377035827727134)
goal = (29.274825605397425,-21.12957658313027)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 178 of 195"
path = []
start = (29.274825605397425,-21.12957658313027)
goal = (29.418776788936924,-22.766388749631393)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 179 of 195"
path = []
start = (29.418776788936924,-22.766388749631393)
goal = (31.302837117056995,-22.28735621795684)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (31.302837117056995,-22.28735621795684)
goal = (27.108683915672643,-24.736315983223665)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (27.108683915672643,-24.736315983223665)
goal = (31.16153592914472,-25.896245068695006)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 180 of 195"
path = []
start = (31.302837117056995,-22.28735621795684)
goal = (34.60326658685362,-23.346420463441863)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 181 of 195"
path = []
start = (27.108683915672643,-24.736315983223665)
goal = (25.922560835701518,-21.899346566484194)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 182 of 195"
path = []
start = (25.922560835701518,-21.899346566484194)
goal = (22.981091156939755,-21.792126869775974)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 183 of 195"
path = []
start = (22.981091156939755,-21.792126869775974)
goal = (22.508155185599193,-21.2619930448424)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (22.508155185599193,-21.2619930448424)
goal = (20.439149989347527,-25.970434196776825)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 184 of 195"
path = []
start = (22.508155185599193,-21.2619930448424)
goal = (20.813429085610466,-20.794979114357798)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (20.813429085610466,-20.794979114357798)
goal = (23.85038148842409,-19.279139111928508)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 185 of 195"
path = []
start = (34.60326658685362,-23.346420463441863)
goal = (35.963616549812656,-23.12174982170741)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 186 of 195"
path = []
start = (27.004798582219223,-13.595526392738197)
goal = (27.06479411576568,-11.02081233546859)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 187 of 195"
path = []
start = (27.06479411576568,-11.02081233546859)
goal = (24.522657630025932,-12.064926523439393)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 188 of 195"
path = []
start = (24.522657630025932,-12.064926523439393)
goal = (24.42488138896114,-11.968988445670425)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 189 of 195"
path = []
start = (24.42488138896114,-11.968988445670425)
goal = (24.024455296930242,-12.066697527969342)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (24.024455296930242,-12.066697527969342)
goal = (24.16854962630451,-10.683587645427076)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 190 of 195"
path = []
start = (24.024455296930242,-12.066697527969342)
goal = (22.659339053401126,-13.02044463312298)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 191 of 195"
path = []
start = (22.659339053401126,-13.02044463312298)
goal = (22.87928228545266,-15.28789064074774)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 192 of 195"
path = []
start = (20.439149989347527,-25.970434196776825)
goal = (17.98540429121362,-25.60711114241667)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 193 of 195"
path = []
start = (17.98540429121362,-25.60711114241667)
goal = (14.093616319898032,-24.552436035321758)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 194 of 195"
path = []
start = (14.093616319898032,-24.552436035321758)
goal = (14.42264696476294,-22.031792089910176)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 195 of 195"
path = []
start = (18.51275054257855,16.022838643490914)
goal = (19.597628346531238,15.164677186532103)
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
f = open('smo2sol-30.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
