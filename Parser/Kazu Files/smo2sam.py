#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""
@brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT) 

@author: AtsushiSakai

@license: MIT

"""

import shapely
from shapely.geometry import Polygon, LineString, Point
import matplotlib.pyplot as plt
from ast import literal_eval

import random
import math
import copy

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
        p2 = LineString([(nearestNode.x,nearestNode.y),(node.x,node.y)])
        for poly in obstacleList:
            p1 = Polygon(poly)
            if p1.intersects(p2):
#                print "collision"
                return False
#        print "safe"
        return True

def LineCollisionCheck(first,second, obstacleList):
    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a=y2-y1
        b=-(x2-x1)
        c=y2*(x2-x1)-x2*(y2-y1)
    except ZeroDivisionError:
        return False

    p2 = LineString([(x1,y1),(x2,y2)])
    for poly in obstacleList:
        p1 = Polygon(poly)
        if p1.intersects(p2):
#            print "collision"
            return False
#    print "safe"
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

obstacleList = [[(-0.04943873358795161,-0.12154559011750217),(0.9485039727294952,-0.18565763974789667),(1.4154193010730214,-0.7166850177218173),(1.3192512266274294,-2.2135990771979874),(0.7882238486535091,-2.6805144055415138),(1.7861665549709553,-2.7446264551719084),(1.946446679046942,-0.24976968937829114),(2.9443893853643885,-0.31388173900868566),(3.008501434994783,0.6840609673087623),(2.0105587286773363,0.7481730169391565),(2.07467077830773,1.7461157232566034),(6.054289089817501,2.1493998596866444),(6.317299650324139,1.1846069051819048),(3.422920786809919,0.39557522366199366),(3.6859313473165556,-0.5692177308427484),(4.650724301821295,-0.3062071703361086),(5.439755983341207,-3.200586033850328),(6.404548937845945,-2.937575473343692),(6.141538377339308,-1.972782518838955),(8.904411960600209,-0.7013543600666701),(5.878527816832671,-1.007989564334212),(5.615517256326036,-0.04319660982947189),(6.580310210830776,0.21981395067716508),(6.854204481078753,-0.7849033568919279),(7.049193667694944,2.2502208937941544),(8.126439065842805,2.3593861317916853),(2.1387828279381242,2.74405842957405),(2.202894877568519,3.742001135891497),(1.2049521712510711,3.806113185521891),(1.1408401216206774,2.8081704792044446),(0.6098127436467562,2.341255150860918),(-0.8871013158294137,2.4374232253065085),(-1.3540166441729398,2.9684506032804294),(-1.418128693803334,1.9705078969629826),(1.0767280719902836,1.8102277728869973),(1.0126160223598892,0.8122850665695506),(0.014673316042442479,0.8763971161999446)],[(-0.5961850621127621,-2.5368680453489945),(-2.6584931330631454,1.0334100139193456),(-0.4787457023912782,4.5332244573547875),(-4.049023761659607,2.4709163864043977),(-5.74668312875921,3.528249197443088),(-5.906764767774435,4.216997241977662),(-5.113765159495412,5.49024176730237),(-4.42501711496084,5.650323406317597),(-5.273846798510644,6.178989811836937),(-6.595512812309015,4.05691560296243),(-7.5488382050950475,4.650663817076282),(-6.548471851084192,2.9188245757599804),(-7.414391471742343,2.4186413987545587),(-9.91530735676948,6.74823950204531),(-9.948919040107647,7.3062451066257506),(-9.732439134943101,7.431290900877114),(-9.26586764127586,7.123376884799386),(-9.299479324614028,7.681382489379825),(-9.082999419449493,7.806428283631178),(-8.616427925782235,7.498514267553459),(-8.650039609120421,8.05651987213388),(-8.433559703955888,8.181565666385241),(-7.966988210288624,7.873651650307524),(-8.250691482129524,8.864617065217029),(-10.415490533774907,7.6141591227034695),(-11.415856887785747,9.345998364019776),(-12.281776508443892,8.845815187014361),(-8.2803110924005,1.9184582217491184),(-10.0121503337168,0.9180918677382941),(-9.511967156711377,0.0521722470801107),(-6.048288674078768,2.0529049551018272),(-5.486530134144657,1.0803857578079459),(-7.666277564816543,-2.4194286856275053),(-4.095999505548203,-0.3571206146771133),(-2.3983401384486007,-1.4144534257158008),(-2.9270065439679507,-2.2632831092656045),(-3.4035471676150735,-2.555531349660668),(-3.6157545885025213,-2.42336474828083),(-3.5636288066302972,-1.8667833051260931),(-4.040169430277421,-2.159031545521156),(-4.252376851164871,-2.0268649441413196),(-4.200251069292648,-1.4702835009865802),(-4.676791692939774,-1.76253174138165),(-4.888999113827223,-1.6303651400018104),(-4.8368733319549975,-1.0737836968470722),(-5.5777471583617935,-1.7904467790170289),(-3.455672949487296,-3.1121127928154015),(-3.9843393550066404,-3.9609424763652012),(-3.135509671456842,-4.489608881884549),(-2.6068432659374925,-3.6407791983347515),(-1.7580135823876937,-4.169445603854094),(-3.344012798945732,-6.7159346545034975),(-2.4951831153959234,-7.244601060022845),(-1.9665167098765841,-6.395771376473044),(-1.1176870263267826,-6.92443778199239),(-5.347018270481562,-13.715075250390788),(-4.49818858693174,-14.243741655910148),(-3.969522181412416,-13.394911972360338),(-3.280774136877818,-13.234830333345117),(-2.0075296115531382,-14.027829941624134),(-1.8474479725379105,-14.716577986158695),(-1.3187815670185552,-13.867748302608904),(-3.4408557758930693,-12.546082288810537),(-0.2688573427769829,-7.453104187511736),(0.5799723407728129,-7.981770593031083),(1.1086387462921632,-7.132940909481279),(-1.4378503043572353,-5.546941692923243),(-0.909183898837893,-4.698112009373442),(1.6373051518115136,-6.284111225931475),(2.1659715573308618,-5.435281542381674),(1.317141873781059,-4.906615136862328),(2.4787262485641914,-2.095792883453253),(0.4683121902312558,-4.377948731342984),(-2.0781768604181496,-2.7919495147849482),(-1.5495104548988023,-1.943119831235151)],[(4.719494973512677,5.626751800787084),(6.813976568750034,7.774575547562407),(6.098035319824921,8.472736079308193),(5.399874788079139,7.756794830383083),(3.9679922902289237,9.153115893874654),(3.941321214459941,11.274268564880995),(7.0830436073159895,14.496004185043965),(9.204196278322307,14.522675260812957),(7.056372531546984,16.61715685605031),(1.8201685434536037,11.24759748911201),(-0.32765520332170706,13.342079084349379),(-1.0258157350675212,12.626137835424247),(4.70171425633336,7.0408535814579745),(4.003553724587573,6.324912332532869)],[(18.955932279117278,2.3707807430262102),(19.59483067022873,3.1400718750660834),(20.298925431804406,3.2052682455302923),(21.45286212986422,2.2469206588631105),(21.518058500328422,1.5428258972874425),(22.156956891439876,2.312117029327309),(20.233729061340178,3.909363007105963),(20.872627452451642,4.678654139145843),(20.103336320411792,5.317552530257281),(20.03813994994757,6.021647291832959),(20.99648753661475,7.175583989892762),(21.700582298190422,7.240780360356962),(20.93129116615053,7.879678751468436),(19.33404518837189,5.956450921368743),(18.564754056332035,6.595349312480205),(17.925855665220574,5.826058180440325),(17.15656453318068,6.464956571551774),(17.091368162716485,7.169051333127459),(18.049715749383676,8.322988031187265),(18.753810510959333,8.388184401651472),(17.984519378919455,9.027082792762926),(16.387273401140803,7.103854962663231),(15.617982269100931,7.742753353774685),(14.979083877989478,6.9734622217348186),(15.748375010029342,6.3345638306233525),(15.813571380493558,5.630469069047697),(14.85522379382638,4.476532370987881),(14.15112903225071,4.411336000523654),(14.92042016429059,3.7724376094122163),(16.51766614206923,5.695665439511911),(17.286957274109106,5.056767048400451),(16.64805888299765,4.287475916360579),(17.41735001503753,3.648577525249122),(16.778451623926074,2.8792863932092487),(14.66616733919907,2.683697281816618),(11.204357245019617,5.558740041818154),(11.008768133627001,7.6710243265451545),(10.369869742515556,6.90173319450527),(9.665774980939881,6.836536824041066),(8.511838282880065,7.794884410708257),(8.446641912415846,8.498979172283882),(7.8077435213043955,7.729688040244028),(9.730971351404072,6.132442062465352),(9.09207296029263,5.3631509304255305),(14.861756450591702,0.5714129970896251),(12.306162886145877,-2.505751531069876),(13.075454018185747,-3.1446499221813218),(13.714352409297204,-2.3753587901414486),(45.242223509401434,-20.75995407791467),(17.54774275596594,2.240388002097796),(18.186641147077406,3.009679134137665)],[(-14.17978153692173,-11.858066470811073),(-14.901682663051275,-11.004334215304175),(-14.704569986074425,-10.544827313750977),(-13.588443505991183,-10.479545766151478),(-14.310344632120728,-9.62581351064458),(-14.113231955143865,-9.166306609091386),(-12.997105475060623,-9.10102506149189),(-13.719006601190182,-8.247292805984987),(-13.521893924213325,-7.787785904431792),(-12.40576744413008,-7.722504356832295),(-14.046682373366012,-6.474546747371695),(-14.440907727319729,-7.393560550478089),(-15.359921530426105,-6.999335196524386),(-15.720872093490874,-6.572469068770937),(-15.622315755002475,-6.342715617994348),(-15.064252514960835,-6.310074844194598),(-15.425203078025614,-5.883208716441149),(-15.326646739537194,-5.653455265664549),(-14.768583499495572,-5.620814491864805),(-15.129534062560337,-5.1939483641113515),(-15.03097772407191,-4.964194913334752),(-14.472914484030293,-4.931554139535003),(-15.293371948648247,-4.307575334804705),(-16.2789353335325,-6.605109842570688),(-17.19794913663891,-6.210884488617),(-17.460343361215255,-5.554264910086945),(-16.869005330284708,-4.1757442054273435),(-16.212385751754617,-3.9133499808509935),(-17.131399554861037,-3.5191246268973035),(-18.116962939745285,-5.816659134663289),(-19.954990545958093,-5.028208426755911),(-20.349215899911805,-5.947222229862303),(-19.430202096805424,-6.341447583816006),(-19.905755207225738,-8.284087344340197),(-20.87707508748783,-8.046310789130011),(-18.737086090596268,0.6955681332288943),(-15.745503331988056,0.470996099355693),(-15.28433487786555,-0.06502969939121428),(-15.396620894802153,-1.5608210786953158),(-15.932646693549058,-2.0219895328178232),(-14.935452440679647,-2.09684687744223),(-14.74830907911866,0.396138754731302),(-12.671390253813506,0.24022868222998817),(-18.26153298017594,2.6382078937531173),(-18.023756424965747,3.6095277740151896),(-19.96639618549003,4.085080884435543),(-22.81971484801204,-7.5707576787096755),(-24.762354608536256,-7.095204568289361),(-24.30701515753732,-1.0295087315065583),(-24.70124051149106,-1.9485225346129589),(-25.67256039175316,-1.7107459794028124),(-24.959230726122637,1.2032136613835078),(-25.930550606384706,1.4409902165937112),(-26.168327161594867,0.4696703363316015),(-26.530045409265522,0.04345453500306817),(-26.77287537933106,0.10289867380558704),(-26.89681707179149,0.6480027527391865),(-27.25853531946212,0.2217869514106834),(-27.501365289527605,0.2812310902132502),(-27.62530698198806,0.8263351691468159),(-27.98702522965867,0.40011936781832524),(-28.22985519972415,0.4595635066209258),(-28.353796892184647,1.004667585554479),(-28.834403417460322,0.09279184409490249),(-26.40610371680506,-0.5016495439305189),(-26.643880272015238,-1.472969424192625),(-27.655394597787083,-1.22535337430506),(-25.09546586544476,-2.8675363377193754),(-26.704994369060437,-6.619651457868975),(-30.59027389010884,-5.668545237028294),(-32.29513709542289,-4.22167224634591),(-32.05736054021273,-3.250352366083826),(-29.876944224478358,-2.754585596242011),(-31.581807429792384,-1.3077126055595976),(-31.344030874582216,-0.33639272529747544),(-29.163614558847858,0.15937404454427373),(-30.868477764161888,1.6062470352267209),(-30.6307012089517,2.577566915488857),(-28.450284893217365,3.0733336853305744),(-29.334934304858795,3.539590501347705),(-28.705979459117863,5.4381204068321995),(-27.442237083505134,6.0729079367039756),(-24.594442225278378,5.129475668092544),(-23.959654695406634,3.8657332924798027),(-23.330699849665653,5.7642631979643415),(-25.229229755150122,6.393218043705291),(-23.027887795056774,13.03807271290104),(-23.97715274779901,13.352550135771546),(-26.178494707892405,6.707695466575748),(-28.077024613376913,7.336650312316712),(-27.422042820397554,9.313743787213577),(-31.104233128141715,4.472104133381935),(-32.097787859055586,4.995759786433384),(-34.47555341115725,-4.717439016187589),(-43.21743233351615,-2.5774500192960144),(-43.93076199914668,-5.491409660082374),(-20.619084872856238,-11.198046985126485),(-20.876540417421204,-12.249755756408033),(-17.592174490592598,-7.129898291723392),(-16.67316068748622,-7.524123645677094),(-19.432738165162114,-13.957220267421853),(-18.51372436205571,-14.351445621375555),(-15.754146884379814,-7.918348999630789),(-14.835133081273417,-8.312574353584488),(-16.017809143134517,-11.069615762903679)],[(7.920498942666547,-10.685529071579568),(7.129897864812931,-10.073197514580665),(9.579224092808555,-6.910793203166207),(14.935162116929142,-9.794181467306029),(10.191555649807466,-6.120192125312598),(10.803887206806358,-5.329591047458977),(11.50535352423263,-5.2404562870316225),(12.691255141013059,-6.158953622529998),(12.780389901440394,-6.860419939956237),(13.392721458439333,-6.069818862102657),(11.416218763805276,-4.538989969605361),(12.028550320804179,-3.7483888917517634),(11.237949242950565,-3.1360573347528478),(6.339296786959321,-9.46086595758176),(5.548695709105701,-8.848534400582857),(4.936364152106797,-9.639135478436472),(5.7269652299604115,-10.25146703543537),(5.816099990387768,-10.952933352861628),(4.897602654889409,-12.138834969642057),(4.196136337463154,-12.2279697300694),(4.986737415316762,-12.840301287068305),(6.517566307814026,-10.863798592434275),(7.30816738566764,-11.476130149433178)],[(-9.398530488377414,-10.803233866142232),(-10.364090163906644,-11.063415558411087),(-10.97696084780569,-10.710726566780892),(-11.367233386208968,-9.262387053487052),(-11.014544394578794,-8.649516369588012),(-11.980104070108016,-8.909698061856862),(-11.329649839435874,-11.323597250679938),(-12.295209514965109,-11.583778942948795),(-12.035027822696257,-12.549338618478025),(-11.069468147167026,-12.289156926209174),(-10.80928645489817,-13.254716601738405),(-11.16197544652837,-13.86758728563744),(-12.610314959822198,-14.257859824040734),(-13.223185643721253,-13.905170832410533),(-12.963003951452375,-14.870730507939781),(-10.549104762629305,-14.220276277267638),(-10.028741378091587,-16.151395628326107),(-9.063181702562344,-15.891213936057238),(-9.58354508710007,-13.960094584998782),(-9.230856095469898,-13.347223901099737),(-7.782516582176032,-12.956951362696444),(-7.169645898276996,-13.309640354326636),(-7.429827590545847,-12.344080678797402),(-9.843726779368927,-12.994534909469552),(-10.10390847163779,-12.028975233940312),(-9.138348796108556,-11.768793541671462)],[(2.1077787075110574,-13.102117191897271),(-0.7498900551875591,-12.189029439982617),(-1.6629778071022034,-15.046698202681247),(-0.7104215528693305,-15.351060786652797),(-0.38632471773867616,-15.979520205754998),(-0.8428685936959917,-17.408354587104302),(-1.471328012798213,-17.73245142223497),(-0.5187717585653591,-18.036814006206512),(0.2421347013635311,-15.655423370624337),(1.1946909555964034,-15.959785954595889),(1.4990535395679503,-15.007229700363006),(3.4041660480337095,-15.615954868306115),(1.2736279602328884,-22.283848647936235),(2.226184214465763,-22.58821123190778),(2.834909382408815,-20.683098723442033),(3.7874656366417057,-20.98746130741358),(4.587840598888787,-21.768102018501573),(4.435659306903048,-22.24438014561801),(3.3309217606843564,-22.416295688762887),(4.131296722931493,-23.196936399850877),(3.9791154309457215,-23.67321452696731),(2.874377884727053,-23.845130070112198),(3.6747528469741333,-24.625770781200195),(3.522571554988388,-25.102048908316625),(2.4178340087697263,-25.273964451461527),(4.1707652252496885,-26.358967746521046),(5.692578145107441,-21.59618647535668),(10.455359416271817,-23.117999395214433),(10.759722000243332,-22.165443140981587),(3.139271966380373,-19.73054246920916),(4.356722302266567,-15.92031745227767),(9.119503573430935,-17.44213037213542),(9.423866157402488,-16.48957411790251),(1.8034161235395065,-14.05467344613014)],[(-2.8248693250708374,5.491909463623024),(-2.7409303791328545,6.606788045960501),(-2.2781911570102675,6.796188184052804),(-1.4366516587030769,6.060109877899931),(-1.3527127127650935,7.174988460237406),(-0.8899734906425087,7.36438859832971),(-0.04843399233531498,6.628310292176841),(0.03550495360266126,7.74318887451432),(0.4982441757252545,7.932589012606613),(1.3397836740324438,7.196510706453747),(1.0449223437858208,9.236867733036398),(0.11944389954065349,8.858067456851792),(-1.395757205197766,12.559981233832483),(-2.3212356494429534,12.181180957647882),(-0.8060345447045161,8.479267180667179),(-1.7315129889496967,8.10046690448258),(-2.28895228011844,8.142436377451574),(-2.383652349164592,8.373805988512864),(-2.015613196088154,8.794575737666463),(-2.5730524872568954,8.836545210635451),(-2.6677525563030384,9.067914821696746),(-2.299713403226617,9.488684570850342),(-2.8571526943953445,9.530654043819332),(-2.951852763441494,9.762023654880627),(-2.583813610365062,10.182793404034223),(-3.6039921236563788,10.035362738910909),(-2.656991433194877,7.72166662829798),(-3.582469877440044,7.34286635211337)]]

rand = (0,10)

content = ""
path = []

start = (10.212821270758674,10.49422742642497)
goal = (7.8557323752786985,10.073504700718686)
path += rrtpath(obstacleList,start,goal,rand) # rrt  returns smoothie path



start = (7.8557323752786985,10.073504700718686)
goal = (6.269431254118764,9.667809932346302)
path += rrtpath(obstacleList,start,goal,rand)[1:] # rrt  returns smoothie path

drawPolygons(obstacleList) # draw map

pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr

print content


#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()