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

obstacleList = [[(8.40260913159547E-5,-0.04705259816401304),(0.6033271969453917,0.7505047206651231),(1.400884515774528,0.14726154981104783),(1.9838269596997096,-4.055139919238589),(-3.4453615779869744,-11.233155788700813),(-7.647763047036607,-11.816098232625995),(-2.862419134061788,-15.435557257750451),(6.186228428749346,-3.472197475313406),(6.983785747578483,-4.075440646167481),(7.587028918432558,-3.277883327338345),(1.2065703677994675,1.5480620394942592),(1.8098135386535432,2.3456193583233955),(1.0122562198244072,2.9488625291774717),(-0.7974732927378201,0.5561905726900628)],[(-7.578873080015682,11.617826207472184),(-8.577279795819585,11.561398999162185),(-9.028697462299611,19.548652725593424),(-10.027104178103507,19.49222551728342),(-9.57568651162349,11.504971790852181),(-10.574093227427397,11.448544582542182),(-10.404811602497405,8.453324435130478),(-9.406404886693505,8.509751643440481),(-9.186608378280667,7.534205901360924),(-10.16215412036022,7.314409392948102),(-11.04134015401151,11.216592361266281),(-12.016885896091061,10.996795852853454),(-11.797089387678232,10.021250110773915),(-13.748180871837324,9.581657093948259),(-15.066959922314282,15.43493154642551),(-16.042505664393826,15.215135038012692),(-14.723726613916867,9.36186058553543),(-15.69927235599641,9.142064077122605),(-15.479475847583586,8.166518335043058),(-11.577292879265412,9.045704368694375),(-11.137699862439767,7.0946128845352785),(-13.088791346598848,6.655019867709618),(-12.86899483818604,5.679474125630081),(-8.966811869867852,6.55866015928138),(-8.73791990386154,5.5427451001837555),(-8.6294423062803,6.536843994068561),(-5.647145624625869,6.211411201324857),(-5.538668027044618,7.205510095209682),(-8.520964708699074,7.530942887953377),(-8.407998170889586,8.56617885175047),(-7.40959145508569,8.622606060060468),(-7.466018663395686,9.621012775864378),(-4.470798515983964,9.790294400794386),(-3.680817599643963,-4.187399620460315),(-1.6840041680360809,-4.074545203840275),(-2.473985084376145,9.90314881741439),(-0.47717165276834184,10.016003234034406),(-0.5335988610783478,11.014409949838297),(-4.527225724293963,10.788701116598293),(-4.809361765843983,15.78073469561782),(-4.143281668356282,16.526614916384375),(7.790801863908713,5.869333356581127),(9.122962058884102,7.361093798114264),(-2.8111214733809007,18.018375357917495),(-1.9834232433622674,18.945236467959543),(4.007017051461154,19.283799717819548),(4.232725884701165,15.290172854603934),(5.2311326005050836,15.346600062913964),(5.005423767265067,19.34022692612956),(7.002237198872888,19.453081342749552),(10.166738971214595,16.627142820267874),(9.828175721354565,22.617583115091335),(-5.147925015703988,21.771174990441246),(-5.204352224013995,22.76958170624514),(-7.201165655621825,22.656727289625135),(-6.524039155901773,10.675846699978282),(-7.522445871705684,10.619419491668278)],[(14.09371837469607,26.389384816862236),(14.876167274441567,27.012099614572275),(16.121596869861648,25.447201815081264),(14.556699070370657,24.2017722196612),(9.574980688690315,30.461363417625194),(14.26967408716337,34.197652203885426),(24.85582564823401,20.896020908211916),(20.783847047471106,16.37728322220623),(26.101255243654098,19.33112310872092),(29.214829232204345,15.418878609993499),(31.562175931440812,17.28702300312359),(25.95774275205044,24.32906310083309),(25.3790168824572,27.0635788473344),(26.35707800713907,27.841972344471944),(28.89192612609607,26.664243592245718),(28.313200256502846,29.398759338747023),(29.291261381184725,30.1771528358846),(31.826109500141687,28.99942408365834),(31.247383630548445,31.733939830159706),(32.225444755230356,32.51233332729725),(34.76029287418727,31.33460457507102),(32.624780010318986,36.0252425709361),(22.844168763500235,28.241307599560553),(16.617020786399856,36.06579659701555),(18.96436748563636,37.933940990145665),(17.718937890216253,39.49883878963666),(8.32955109327025,32.026261217116186),(7.0841214978501315,33.59115901660716),(4.736774798613661,31.723014623477056),(5.982204394033751,30.15811682398608),(5.742603240980554,28.050371277802775),(2.2215831921258147,25.24815468810758),(0.11383764594249257,25.487755841160748),(0.7365524436525472,24.705306941415262),(0.06712328535802747,23.96243104193738),(-0.6390292435281815,23.925707671345677),(-1.7533430927450215,24.92985140878747),(-1.7900664633366947,25.636003937673593),(-2.4594956216312553,24.893128038195773),(-0.6023058729365047,23.21955514245952),(-1.299436856728974,22.44593809841233),(1.3592672413625646,23.922858041669773),(1.9819820390726193,23.140409141924263),(7.850348787163865,27.81077012474956),(12.209352371134145,22.333627826531078),(11.426903471388648,21.71091302882104),(12.049618269098692,20.92846412907554),(13.614516068589706,22.173893724495613),(14.357391968067578,21.50446456620111),(14.394115338659299,20.798312037314886),(13.389971601217548,19.683998188098045),(12.683819072331298,19.64727481750637),(13.426694971809223,18.977845659211845),(15.100267867545478,20.83503540790658),(15.873884911592564,20.137904424114144),(14.396964968335192,22.79660852220566),(15.179413868080687,23.419323319915698),(17.04755826121083,21.071976620679216),(16.967691210193063,20.369394771951416),(15.794017860574824,19.435322575386365),(15.091436011847083,19.5151896264041),(15.714150809557138,18.732740726658594),(17.670273058920873,20.28952772093371),(18.292987856630962,19.507078821188244),(15.163192257648863,17.016219630348033),(12.903823414646006,19.052208930729513),(14.380743357903429,16.393504832638),(13.598294458157897,15.770790034927956),(14.221009255867965,14.98834113518245),(15.003458155613448,15.611055932892494),(15.550361304913714,15.726801106811148),(15.70604000434121,15.531188881874764),(15.470494253895955,15.02421925808337),(16.017397403196217,15.139964432002005),(16.173076102623728,14.94435220706564),(15.937530352178484,14.437382583274253),(16.484433501478772,14.553127757192877),(16.64011220090626,14.35751553225653),(16.404566450461033,13.850545908465119),(17.342694049634034,14.277648481238781),(15.785907055358898,16.233770730602526),(18.915702654341004,18.72462992144273),(19.538417452051014,17.94218102169723),(20.320866351796518,18.564895819407255),(15.96186276782619,24.042038117625737),(16.744311667571694,24.66475291533578),(17.989741262991775,23.099855115844797),(18.772190162737264,23.72256991355481),(15.658616174187065,27.63481441228231),(16.441065073932577,28.257529209992352),(15.818350276222535,29.039978109737856),(15.035901376477032,28.41726331202782),(14.333319527749271,28.497130363045546),(13.399247331184199,29.670803712663798),(13.479114382201919,30.373385561391554),(12.69666548245641,29.750670763681512),(14.253452476731539,27.79454851431778),(13.471003576986035,27.171833716607733)],[(-18.654059671964447,1.0051478316189262),(-19.323053437268733,0.26187981671179095),(-21.887354364642277,1.897227105171083),(-21.26418120848873,1.1151432110461559),(-22.046265102613656,0.49197005489259793),(-22.7488936277529,0.5714254238782944),(-23.683653361983232,1.7445512650656942),(-23.60419799299754,2.4471797902049452),(-24.386281887122475,1.824006634051396),(-22.828348996738587,-0.13120310126096335),(-24.392516784988455,-1.3775494135680688),(-23.769343628834896,-2.1596333076929826),(-20.641008052335174,0.33305931692122226),(-19.992047202573016,-0.4813881981953414),(-20.661040967877305,-1.22465621310248),(-19.917772952970175,-1.8936499784067617),(-19.713387386842673,-2.4139638647856967),(-19.88063582816875,-2.5997808685124784),(-20.41951827694838,-2.4511009895871174),(-20.215132710820885,-2.971414875966042),(-20.38238115214696,-3.157231879692825),(-20.921263600926608,-3.0085520007674664),(-20.71687803479911,-3.528865887146391),(-20.88412647612516,-3.7146828908731817),(-21.42300892490482,-3.566003011947814),(-20.846989351323746,-4.420813780978889),(-19.174504938063038,-2.5626437437110523),(-18.431236923155904,-3.231637509015337),(-17.762243157851625,-2.488369494108203),(-17.018975142944488,-3.157363259412492),(-20.363943969465907,-6.873703333948155),(-19.620675954558756,-7.542697099252457),(-16.275707128037347,-3.8263570247167853),(-15.532439113130216,-4.495350790021066),(-19.54640170495594,-8.954958879463874),(-18.05986567514165,-10.292946410072442),(-17.39087190983738,-9.549678395165314),(-16.64760389493024,-10.218672160469595),(-21.330560252060227,-15.42154826481953),(-20.58729223715307,-16.09054203012384),(-15.90433588002314,-10.887665925773893),(-15.161067865115994,-11.556659691078172),(-14.492074099811717,-10.81339167617104),(-16.721878144533115,-8.806410380258175),(-16.052884379228807,-8.063142365351036),(-14.6406225990174,-7.988868115748193),(-12.410818554295982,-9.99584941166106),(-12.33654430469316,-11.408111191872491),(-10.998556774084582,-9.921575162058238),(-14.714896848620269,-6.576606335536793),(-14.045903083315942,-5.833338320629636),(-13.302635068408836,-6.502332085933938),(-12.633641303104543,-5.759064071026815),(-17.093249392547335,-1.745101479201073),(-16.42425562724305,-1.001833464293941),(-17.16752364215018,-0.33283969898964627),(-13.822554815628738,3.383500375546009),(-14.565822830535875,4.0524941408503015),(-15.234816595840147,3.3092261259431517),(-29.356908879075718,16.020107666724638),(-25.34294628725,20.479715756167444),(-26.086214302157142,21.14870952147171),(-30.100176893982848,16.689101432028913),(-30.843444908889992,17.358095197333174),(-32.850426204802865,15.12829115261179),(-17.241797891753023,1.0794220812217672),(-17.910791657057313,0.33615406631463596)],[(-3.997322004454697,-1.6610863057627367),(-4.990039971863609,-1.7815480722428845),(-5.833272337224625,5.167477699619508),(-6.825990304633526,5.047015933139368),(-5.982757939272519,-1.9020098387230324),(-6.975475906681433,-2.0224716052031804),(-6.855014140201287,-3.015189572612093),(-5.862296172792374,-2.8947278061319475),(-5.305706305847845,-3.3308559065963252),(-5.1250136561276225,-4.819932857709697),(-5.561141756592002,-5.376522724654224),(-4.568423789183095,-5.256060958174077),(-4.869578205383465,-2.7742660396517964),(-3.8768602379745456,-2.653804273171648)],[(20.406552158561784,3.0047313921663696),(19.782614252693328,2.223257470899327),(16.656718567625152,4.719009094373132),(16.577950559925856,5.42171500794087),(17.513857418728534,6.593925889841447),(18.216563332296282,6.672693897540745),(17.435089411029242,7.296631803409195),(15.875244646358103,5.342947000241587),(15.093770725091051,5.96688490611002),(14.469832819222614,5.185410984842989),(17.59572850429079,2.6896593613691864),(15.099976880816964,-0.4362363236989797),(12.755555117015838,1.4355773939063818),(12.131617211147397,0.6541034726393158),(14.476038974948533,-1.2177102449660402),(13.852101069080074,-1.9991841662330727),(14.633574990347162,-2.62312207210157),(15.257512896215584,-1.8416481508345046),(23.07225210888604,-8.081027209519004),(24.320127920622973,-6.518079366984903),(16.505388707952502,-0.27870030830042936),(18.377202425557833,2.065721455500732),(19.15867634682488,1.4417835496322822),(18.53473844095643,0.6603096283652343),(20.879160204757582,-1.2115040892401168),(21.503098110626027,-0.4300301679730727),(22.28457203189308,-1.0539680738415176),(22.363340039592362,-1.7566739874092776),(21.427433180789706,-2.928884869309837),(20.724727267221947,-3.007652877009138),(21.506201188488987,-3.6315907828775873),(23.066045953160106,-1.6779059797099731),(23.84751987442715,-2.301843885578428),(24.4714577802956,-1.5203699643113766),(22.127036016494472,0.35144375329397404),(22.750973922362917,1.1329176745610128),(21.96950000109587,1.756855580429471),(22.45647503903122,2.6302714678667627),(29.44380213852955,-1.265528835615974),(29.930777176464908,-0.39211294817867515),(28.183945401590325,0.5818371276920202),(39.87134631203853,21.543818426187013),(36.37768276228939,23.491718577928356),(24.69028185184115,2.5297372794333643),(22.943450076966563,3.503687355304047),(23.450576758067697,4.4132462971648305),(21.188026079828823,2.3807934862979185)],[(4.781537263074279,3.446889966800974),(4.404384028777382,2.5207390947183606),(1.4373547953810846,3.1891233615677526),(4.027230794480484,1.5945882226357455),(3.6500775601835853,0.6684373505531269),(4.576228432266196,0.2912841162562283),(4.953381666563099,1.2174349883388431),(10.510286899058784,-1.0454844174425517),(10.887440133355685,-0.11933354535993113),(5.330534900859995,2.14358586042146),(5.707688135156892,3.0697367325040736)]]
rand = (-8, 10)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 123"
path = []
start = (-29.94983412446257,4.093055135337504)
goal = (-31.20818387780966,3.6756682782481853)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-31.20818387780966,3.6756682782481853)
goal = (-32.721552965343044,6.77370395888677)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-32.721552965343044,6.77370395888677)
goal = (-24.710757732879813,2.672138134909332)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-24.710757732879813,2.672138134909332)
goal = (-21.065921125453002,10.719647786723655)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-21.065921125453002,10.719647786723655)
goal = (-21.383816798168894,-7.750982107189488)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-21.383816798168894,-7.750982107189488)
goal = (-12.747220584281703,16.236479365786213)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-12.747220584281703,16.236479365786213)
goal = (2.6464990791349763,10.26821262311044)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 2 of 123"
path = []
start = (-31.20818387780966,3.6756682782481853)
goal = (-30.832259999830715,1.725028362333763)
print "     Node 1 and 2 of 8"
path += rrtpath(obstacleList,start,goal,rand)
start = (-30.832259999830715,1.725028362333763)
goal = (-32.39632035097078,8.780798411497933)
print "     Node 2 and 3 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-32.39632035097078,8.780798411497933)
goal = (-27.716677673311814,10.120244052658908)
print "     Node 3 and 4 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-27.716677673311814,10.120244052658908)
goal = (-32.537163268807,-8.34831602717992)
print "     Node 4 and 5 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-32.537163268807,-8.34831602717992)
goal = (-30.41896358047055,-12.936156154951052)
print "     Node 5 and 6 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-30.41896358047055,-12.936156154951052)
goal = (-7.733847698638289,3.4592484868774456)
print "     Node 6 and 7 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.733847698638289,3.4592484868774456)
goal = (-11.232653780710244,32.569772046850446)
print "     Node 7 and 8 of 8"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 3 of 123"
path = []
start = (-30.832259999830715,1.725028362333763)
goal = (-29.665069361287483,-4.107607701856487)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-29.665069361287483,-4.107607701856487)
goal = (-25.580359974551094,-4.8352760418795135)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-25.580359974551094,-4.8352760418795135)
goal = (-26.295457533222454,-7.890373351762181)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-26.295457533222454,-7.890373351762181)
goal = (-19.90526977142031,-9.930829464805626)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-19.90526977142031,-9.930829464805626)
goal = (-7.6539844940148924,1.3045413582476506)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.6539844940148924,1.3045413582476506)
goal = (6.430040009145664,6.824806457879262)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 4 of 123"
path = []
start = (-32.721552965343044,6.77370395888677)
goal = (-31.350474575001275,10.376761308276318)
print "     Node 1 and 2 of 7"
path += rrtpath(obstacleList,start,goal,rand)
start = (-31.350474575001275,10.376761308276318)
goal = (-32.04243379232117,12.231815511852382)
print "     Node 2 and 3 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-32.04243379232117,12.231815511852382)
goal = (-27.38720035159396,16.430575668440138)
print "     Node 3 and 4 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-27.38720035159396,16.430575668440138)
goal = (-20.551621391281913,16.44503764201175)
print "     Node 4 and 5 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-20.551621391281913,16.44503764201175)
goal = (-22.699097255131502,28.44711473712447)
print "     Node 5 and 6 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-22.699097255131502,28.44711473712447)
goal = (-12.330333038805165,34.72560002195867)
print "     Node 6 and 7 of 7"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 5 of 123"
path = []
start = (-32.39632035097078,8.780798411497933)
goal = (-32.29484671974041,16.291868354102572)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-32.29484671974041,16.291868354102572)
goal = (-32.14496274020945,18.572628960663888)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-32.14496274020945,18.572628960663888)
goal = (-19.596560795831053,18.7158865838008)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-19.596560795831053,18.7158865838008)
goal = (-17.99647748132761,26.11527412348841)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-17.99647748132761,26.11527412348841)
goal = (-6.48578208612334,34.08213065552897)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 6 of 123"
path = []
start = (-29.665069361287483,-4.107607701856487)
goal = (-26.068076997549966,-5.338068982200104)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-26.068076997549966,-5.338068982200104)
goal = (-27.475049333524936,-8.907567624513733)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-27.475049333524936,-8.907567624513733)
goal = (-23.700426217709886,-14.345322367522577)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-23.700426217709886,-14.345322367522577)
goal = (-5.392243533519167,2.9338411960229323)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.392243533519167,2.9338411960229323)
goal = (6.041813441245139,11.367518726251891)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 7 of 123"
path = []
start = (-31.350474575001275,10.376761308276318)
goal = (-31.884050506194573,16.27985865867073)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-31.884050506194573,16.27985865867073)
goal = (-22.09338098940799,12.726563731544367)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-22.09338098940799,12.726563731544367)
goal = (-23.425035498688054,23.666556621557493)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-23.425035498688054,23.666556621557493)
goal = (-11.423210056245072,20.532437876090572)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-11.423210056245072,20.532437876090572)
goal = (-11.678766492295068,38.4904176308184)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 8 of 123"
path = []
start = (-24.710757732879813,2.672138134909332)
goal = (-21.224335610176396,2.325539736098733)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-21.224335610176396,2.325539736098733)
goal = (-15.758708107655941,3.972667792510741)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.758708107655941,3.972667792510741)
goal = (-14.90784316883078,-2.359781773323215)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-14.90784316883078,-2.359781773323215)
goal = (-6.1145986381243524,5.226554871369132)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.1145986381243524,5.226554871369132)
goal = (5.443953988271609,13.393222608376561)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 9 of 123"
path = []
start = (-27.716677673311814,10.120244052658908)
goal = (-22.741967141424,14.837213152148827)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.741967141424,14.837213152148827)
goal = (-16.313247146800975,15.52107876362189)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-16.313247146800975,15.52107876362189)
goal = (-7.223818172790136,13.59143924283401)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.223818172790136,13.59143924283401)
goal = (1.1523359507474922,23.120928276924904)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 10 of 123"
path = []
start = (-25.580359974551094,-4.8352760418795135)
goal = (-21.11531290398012,-6.448460124690808)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-21.11531290398012,-6.448460124690808)
goal = (-23.768808989308226,-15.525831031780287)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-23.768808989308226,-15.525831031780287)
goal = (-4.5436253580669685,-1.2200230049041387)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.5436253580669685,-1.2200230049041387)
goal = (9.611216169416629,-2.62979538019116)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 11 of 123"
path = []
start = (-32.04243379232117,12.231815511852382)
goal = (-24.913642815577788,18.69274067717946)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.913642815577788,18.69274067717946)
goal = (-25.664420804061205,26.68616129775199)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-25.664420804061205,26.68616129775199)
goal = (-19.89576428867944,29.599951571053122)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-19.89576428867944,29.599951571053122)
goal = (-0.7465373853684056,27.536477916482188)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 12 of 123"
path = []
start = (-32.29484671974041,16.291868354102572)
goal = (-32.35277161148735,26.16357699505701)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-32.35277161148735,26.16357699505701)
goal = (-31.6453971144311,29.1671288277411)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-31.6453971144311,29.1671288277411)
goal = (-23.792097404794823,32.106594773516775)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-23.792097404794823,32.106594773516775)
goal = (-2.62647573517474,31.571006562876253)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 13 of 123"
path = []
start = (-26.068076997549966,-5.338068982200104)
goal = (-25.952862070138977,-9.924212727383585)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-25.952862070138977,-9.924212727383585)
goal = (-16.446797177694425,-12.019693076703176)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-16.446797177694425,-12.019693076703176)
goal = (-5.564894263433807,3.788839295666307)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-5.564894263433807,3.788839295666307)
goal = (9.659913738053469,-2.0573004542499476)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 14 of 123"
path = []
start = (-31.884050506194573,16.27985865867073)
goal = (-31.475025340494458,27.491584808988595)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-31.475025340494458,27.491584808988595)
goal = (-23.313546190214364,27.57797297783816)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-23.313546190214364,27.57797297783816)
goal = (-24.84138307800903,32.84422353712429)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-24.84138307800903,32.84422353712429)
goal = (-7.893408916215076,39.11785932978243)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 15 of 123"
path = []
start = (-21.224335610176396,2.325539736098733)
goal = (-14.276964744594576,5.329074366144297)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-14.276964744594576,5.329074366144297)
goal = (-14.204286575459882,7.704393303677634)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-14.204286575459882,7.704393303677634)
goal = (-6.877393836826592,14.206326347599248)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-6.877393836826592,14.206326347599248)
goal = (7.207851254902003,12.395598914982394)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 16 of 123"
path = []
start = (-21.065921125453002,10.719647786723655)
goal = (-16.05540415968757,10.985117221913406)
print "     Node 1 and 2 of 5"
path += rrtpath(obstacleList,start,goal,rand)
start = (-16.05540415968757,10.985117221913406)
goal = (-14.20075107366996,13.075932607701368)
print "     Node 2 and 3 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-14.20075107366996,13.075932607701368)
goal = (-7.13777167653701,16.47434033035113)
print "     Node 3 and 4 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.13777167653701,16.47434033035113)
goal = (1.7159234898477749,22.433502706231554)
print "     Node 4 and 5 of 5"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 17 of 123"
path = []
start = (-32.537163268807,-8.34831602717992)
goal = (-32.771448700625626,-15.709087386558751)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-32.771448700625626,-15.709087386558751)
goal = (-4.368473931441255,-14.611003210961568)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-4.368473931441255,-14.611003210961568)
goal = (9.81613386989251,2.4912077289340786)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 18 of 123"
path = []
start = (-26.295457533222454,-7.890373351762181)
goal = (-17.42751353580522,-14.827071968461045)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-17.42751353580522,-14.827071968461045)
goal = (-3.170302091910596,-15.318093206036494)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-3.170302091910596,-15.318093206036494)
goal = (10.065613408185136,3.7547230338402997)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 19 of 123"
path = []
start = (-27.38720035159396,16.430575668440138)
goal = (-16.506194684606367,19.660675913438045)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-16.506194684606367,19.660675913438045)
goal = (-17.909607304926695,28.818324995839465)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-17.909607304926695,28.818324995839465)
goal = (-0.25627708201279376,28.586284551336643)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 20 of 123"
path = []
start = (-32.14496274020945,18.572628960663888)
goal = (-22.635789787798636,27.098073876900035)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-22.635789787798636,27.098073876900035)
goal = (-26.14756311494001,34.47351222462349)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-26.14756311494001,34.47351222462349)
goal = (-3.7636075615154994,36.0666576965224)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 21 of 123"
path = []
start = (-27.475049333524936,-8.907567624513733)
goal = (-17.394870294610907,-15.698685285208816)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-17.394870294610907,-15.698685285208816)
goal = (-1.9862156844086876,-6.6105650782627645)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.9862156844086876,-6.6105650782627645)
goal = (14.025443487679311,-12.399068496128656)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 22 of 123"
path = []
start = (-22.09338098940799,12.726563731544367)
goal = (-16.03414216291479,17.144766751329502)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-16.03414216291479,17.144766751329502)
goal = (-10.047622863908092,22.418377162721473)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-10.047622863908092,22.418377162721473)
goal = (6.3430412372356955,15.5028143902898)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 23 of 123"
path = []
start = (-15.758708107655941,3.972667792510741)
goal = (-13.70227187868933,9.662429512356372)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-13.70227187868933,9.662429512356372)
goal = (-2.033028887796867,4.180636074490021)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.033028887796867,4.180636074490021)
goal = (10.003092637010738,11.12314680931474)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 24 of 123"
path = []
start = (-22.741967141424,14.837213152148827)
goal = (-14.207416406257106,15.755452885838974)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-14.207416406257106,15.755452885838974)
goal = (-15.502478113845473,26.94269566848762)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-15.502478113845473,26.94269566848762)
goal = (-1.0150887723927795,32.93307230954761)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 25 of 123"
path = []
start = (-21.11531290398012,-6.448460124690808)
goal = (-7.933259435621849,-3.3728777860445973)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.933259435621849,-3.3728777860445973)
goal = (-1.050854910553472,-7.681656930975013)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-1.050854910553472,-7.681656930975013)
goal = (13.650183095647378,5.168363355587118)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 26 of 123"
path = []
start = (-24.913642815577788,18.69274067717946)
goal = (-16.16194688653524,23.01307847326746)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-16.16194688653524,23.01307847326746)
goal = (-16.94265303913706,28.497887245466284)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-16.94265303913706,28.497887245466284)
goal = (2.272095206857273,30.13853272531298)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 27 of 123"
path = []
start = (-32.35277161148735,26.16357699505701)
goal = (-28.79035579817255,31.886720049767877)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-28.79035579817255,31.886720049767877)
goal = (-25.561999627379826,34.89251652731173)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-25.561999627379826,34.89251652731173)
goal = (-1.119975361084915,39.465867266657426)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 28 of 123"
path = []
start = (-25.952862070138977,-9.924212727383585)
goal = (-7.724224221512188,-14.099871273217484)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.724224221512188,-14.099871273217484)
goal = (-2.2257822853378997,-15.6476712573884)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-2.2257822853378997,-15.6476712573884)
goal = (15.48253461709347,-9.510584301540202)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 29 of 123"
path = []
start = (-31.475025340494458,27.491584808988595)
goal = (-30.28437062815133,33.86829354348893)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-30.28437062815133,33.86829354348893)
goal = (-26.450301851288913,36.18810658689213)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-26.450301851288913,36.18810658689213)
goal = (0.9524027775387154,33.650753625507974)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 30 of 123"
path = []
start = (-14.276964744594576,5.329074366144297)
goal = (-9.4725991184986,2.0833335826967705)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-9.4725991184986,2.0833335826967705)
goal = (0.07744921077990341,6.028832869662828)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (0.07744921077990341,6.028832869662828)
goal = (9.732100630599128,14.280664174192335)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 31 of 123"
path = []
start = (-16.05540415968757,10.985117221913406)
goal = (-13.887508759356837,15.902991002996028)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-13.887508759356837,15.902991002996028)
goal = (-7.598191974979837,19.71630425943533)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (-7.598191974979837,19.71630425943533)
goal = (4.9270884975011455,24.947993886762095)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 32 of 123"
path = []
start = (-21.383816798168894,-7.750982107189488)
goal = (-7.168507411821711,-15.318203463083462)
print "     Node 1 and 2 of 4"
path += rrtpath(obstacleList,start,goal,rand)
start = (-7.168507411821711,-15.318203463083462)
goal = (1.1511174672763573,-15.609905010532861)
print "     Node 2 and 3 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (1.1511174672763573,-15.609905010532861)
goal = (15.5465811333669,-10.455909304217649)
print "     Node 3 and 4 of 4"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 33 of 123"
path = []
start = (-30.41896358047055,-12.936156154951052)
goal = (2.189304888783468,-14.900488278842445)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.189304888783468,-14.900488278842445)
goal = (15.68088820891861,-7.446477870452391)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 34 of 123"
path = []
start = (-19.90526977142031,-9.930829464805626)
goal = (1.5146968764966715,-0.8939203260562678)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.5146968764966715,-0.8939203260562678)
goal = (17.095704269304797,-14.75872532254501)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 35 of 123"
path = []
start = (-20.551621391281913,16.44503764201175)
goal = (-15.807740424228207,29.558523539527844)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-15.807740424228207,29.558523539527844)
goal = (6.4772410716515765,23.23364055586067)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 36 of 123"
path = []
start = (-19.596560795831053,18.7158865838008)
goal = (-13.643705251672145,29.372668437128713)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-13.643705251672145,29.372668437128713)
goal = (0.8051017157069253,34.40561588333027)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 37 of 123"
path = []
start = (-23.700426217709886,-14.345322367522577)
goal = (5.510433629391997,-13.267527733368402)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.510433629391997,-13.267527733368402)
goal = (17.470759703490913,-14.659861547115966)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 38 of 123"
path = []
start = (-23.425035498688054,23.666556621557493)
goal = (-21.708869996214084,32.9865159812304)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-21.708869996214084,32.9865159812304)
goal = (0.5562549840858182,37.34822956743767)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 39 of 123"
path = []
start = (-14.90784316883078,-2.359781773323215)
goal = (3.3494616981500087,0.8234997595208142)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.3494616981500087,0.8234997595208142)
goal = (15.145288572262757,3.5601688056774883)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 40 of 123"
path = []
start = (-16.313247146800975,15.52107876362189)
goal = (-4.07996484900206,15.607757942970704)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-4.07996484900206,15.607757942970704)
goal = (9.876371178787643,24.25487348303663)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 41 of 123"
path = []
start = (-23.768808989308226,-15.525831031780287)
goal = (5.917402761191866,-14.38642723575742)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (5.917402761191866,-14.38642723575742)
goal = (17.429538084606946,-4.636877716675826)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 42 of 123"
path = []
start = (-25.664420804061205,26.68616129775199)
goal = (-19.55375755529733,32.4995570430096)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-19.55375755529733,32.4995570430096)
goal = (2.0373990713772017,37.60361949322498)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 43 of 123"
path = []
start = (-31.6453971144311,29.1671288277411)
goal = (-24.915661631765058,36.127672376392596)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-24.915661631765058,36.127672376392596)
goal = (3.834646608957101,38.94238324768145)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 44 of 123"
path = []
start = (-16.446797177694425,-12.019693076703176)
goal = (8.295453038644247,-14.36546527413362)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.295453038644247,-14.36546527413362)
goal = (19.5091301136124,-15.051644755599323)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 45 of 123"
path = []
start = (-23.313546190214364,27.57797297783816)
goal = (-19.561598894129197,32.853817112333815)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-19.561598894129197,32.853817112333815)
goal = (8.348423980042618,32.914816213401124)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 46 of 123"
path = []
start = (-14.204286575459882,7.704393303677634)
goal = (-0.9216477735592079,12.72164439722567)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-0.9216477735592079,12.72164439722567)
goal = (12.41535855962799,15.87870794496341)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 47 of 123"
path = []
start = (-14.20075107366996,13.075932607701368)
goal = (-6.395376220499131,23.559709871762106)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-6.395376220499131,23.559709871762106)
goal = (11.193112825880277,20.79183921171882)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 48 of 123"
path = []
start = (-32.771448700625626,-15.709087386558751)
goal = (3.663045068148598,3.6629113006858773)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (3.663045068148598,3.6629113006858773)
goal = (20.573675865649946,-8.250628711678292)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 49 of 123"
path = []
start = (-17.42751353580522,-14.827071968461045)
goal = (8.535315029813468,-13.62268441477411)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.535315029813468,-13.62268441477411)
goal = (18.92621093762594,4.50701397642845)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 50 of 123"
path = []
start = (-16.506194684606367,19.660675913438045)
goal = (-13.770621469102228,30.30691031608738)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-13.770621469102228,30.30691031608738)
goal = (11.300863069797792,21.257984145162883)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 51 of 123"
path = []
start = (-22.635789787798636,27.098073876900035)
goal = (-19.888386423185384,34.66052517178984)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-19.888386423185384,34.66052517178984)
goal = (11.272021779145447,30.31021020325839)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 52 of 123"
path = []
start = (-17.394870294610907,-15.698685285208816)
goal = (6.930624697859308,-1.82416853083539)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (6.930624697859308,-1.82416853083539)
goal = (18.05906382760108,6.142592622904189)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 53 of 123"
path = []
start = (-16.03414216291479,17.144766751329502)
goal = (-5.574041025052814,26.62648409710425)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-5.574041025052814,26.62648409710425)
goal = (12.460575369589286,19.071205826258616)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 54 of 123"
path = []
start = (-13.70227187868933,9.662429512356372)
goal = (1.4759796499335778,10.716526859414905)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.4759796499335778,10.716526859414905)
goal = (15.624623341502584,11.29593791580989)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 55 of 123"
path = []
start = (-14.207416406257106,15.755452885838974)
goal = (-3.294983732057144,26.201838324595084)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-3.294983732057144,26.201838324595084)
goal = (14.800532769700276,20.052172394643815)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 56 of 123"
path = []
start = (-7.933259435621849,-3.3728777860445973)
goal = (7.781454354577242,-1.4704947881961594)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (7.781454354577242,-1.4704947881961594)
goal = (19.87830543255093,4.950091729733728)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 57 of 123"
path = []
start = (-16.16194688653524,23.01307847326746)
goal = (-16.982532477094747,32.83889947063827)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-16.982532477094747,32.83889947063827)
goal = (15.187261637650451,29.633292632310646)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 58 of 123"
path = []
start = (-28.79035579817255,31.886720049767877)
goal = (-25.195048834136262,39.033238493019795)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-25.195048834136262,39.033238493019795)
goal = (14.150172421221441,38.48883249041745)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 59 of 123"
path = []
start = (-7.724224221512188,-14.099871273217484)
goal = (9.11427987842464,-4.67166613468701)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (9.11427987842464,-4.67166613468701)
goal = (24.833129986545423,-11.48953840932235)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 60 of 123"
path = []
start = (-30.28437062815133,33.86829354348893)
goal = (-12.795715668385103,36.57094379896172)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (-12.795715668385103,36.57094379896172)
goal = (15.958021776014391,31.64173236706738)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 61 of 123"
path = []
start = (-9.4725991184986,2.0833335826967705)
goal = (2.572111923411704,9.734908612038488)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (2.572111923411704,9.734908612038488)
goal = (20.76920047373997,5.955586071967232)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 62 of 123"
path = []
start = (-13.887508759356837,15.902991002996028)
goal = (1.1447565482999167,15.088956920641277)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.1447565482999167,15.088956920641277)
goal = (16.48136127814613,21.316216434921333)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 63 of 123"
path = []
start = (-7.168507411821711,-15.318203463083462)
goal = (8.551219214733258,-2.7520171447578097)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (8.551219214733258,-2.7520171447578097)
goal = (26.696833486213237,-7.585077331907154)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 64 of 123"
path = []
start = (-12.747220584281703,16.236479365786213)
goal = (1.198642299849169,18.734105702679212)
print "     Node 1 and 2 of 3"
path += rrtpath(obstacleList,start,goal,rand)
start = (1.198642299849169,18.734105702679212)
goal = (17.387480861740116,17.40299558759799)
print "     Node 2 and 3 of 3"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 65 of 123"
path = []
start = (-7.733847698638289,3.4592484868774456)
goal = (18.20261945720297,15.912578965051484)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 66 of 123"
path = []
start = (-7.6539844940148924,1.3045413582476506)
goal = (22.340711644590876,6.431385192753105)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 67 of 123"
path = []
start = (-22.699097255131502,28.44711473712447)
goal = (18.094411626141998,27.150024207613843)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 68 of 123"
path = []
start = (-17.99647748132761,26.11527412348841)
goal = (18.56826450667716,26.038668178588853)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 69 of 123"
path = []
start = (-5.392243533519167,2.9338411960229323)
goal = (19.25017140934692,17.4619000247154)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 70 of 123"
path = []
start = (-11.423210056245072,20.532437876090572)
goal = (19.040106512791688,27.013898743137023)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 71 of 123"
path = []
start = (-6.1145986381243524,5.226554871369132)
goal = (20.454473194401807,15.383137888367791)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 72 of 123"
path = []
start = (-7.223818172790136,13.59143924283401)
goal = (22.118245507841984,13.169039570967833)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 73 of 123"
path = []
start = (-4.5436253580669685,-1.2200230049041387)
goal = (24.530432468632924,12.046388720076187)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 74 of 123"
path = []
start = (-19.89576428867944,29.599951571053122)
goal = (19.807704066767087,37.411506880044826)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 75 of 123"
path = []
start = (-23.792097404794823,32.106594773516775)
goal = (23.975245994511717,38.40733614027644)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 76 of 123"
path = []
start = (-5.564894263433807,3.788839295666307)
goal = (25.454538250773567,10.822287181819906)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 77 of 123"
path = []
start = (-24.84138307800903,32.84422353712429)
goal = (25.783148333837417,32.99226081332773)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 78 of 123"
path = []
start = (-6.877393836826592,14.206326347599248)
goal = (26.096267811011856,14.72293409600108)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 79 of 123"
path = []
start = (-7.13777167653701,16.47434033035113)
goal = (26.891019366268424,24.948311915341204)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 80 of 123"
path = []
start = (-4.368473931441255,-14.611003210961568)
goal = (28.346901079452877,-10.170382916221943)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 81 of 123"
path = []
start = (-3.170302091910596,-15.318093206036494)
goal = (29.32735876298645,-14.431798178188362)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 82 of 123"
path = []
start = (-17.909607304926695,28.818324995839465)
goal = (26.49073256744311,31.678935441759233)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 83 of 123"
path = []
start = (-26.14756311494001,34.47351222462349)
goal = (30.14106571084178,37.28746773547874)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 84 of 123"
path = []
start = (-1.9862156844086876,-6.6105650782627645)
goal = (29.532195651780818,-10.532421153900549)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 85 of 123"
path = []
start = (-10.047622863908092,22.418377162721473)
goal = (28.51826377568755,12.205314186800141)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 86 of 123"
path = []
start = (-2.033028887796867,4.180636074490021)
goal = (31.718260379438753,-3.2631888499739414)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 87 of 123"
path = []
start = (-15.502478113845473,26.94269566848762)
goal = (31.46401947277338,25.882667108661312)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 88 of 123"
path = []
start = (-1.050854910553472,-7.681656930975013)
goal = (29.80658950529812,-11.734723602223823)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 89 of 123"
path = []
start = (-16.94265303913706,28.497887245466284)
goal = (30.950679080042164,37.444822901319185)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 90 of 123"
path = []
start = (-25.561999627379826,34.89251652731173)
goal = (33.05153447881312,31.581934344655405)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 91 of 123"
path = []
start = (-2.2257822853378997,-15.6476712573884)
goal = (31.075523129813206,-6.312176645327082)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 92 of 123"
path = []
start = (-26.450301851288913,36.18810658689213)
goal = (31.348840208755824,14.852698501164198)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 93 of 123"
path = []
start = (0.07744921077990341,6.028832869662828)
goal = (32.467905730725654,8.224286206273224)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 94 of 123"
path = []
start = (-7.598191974979837,19.71630425943533)
goal = (35.212479662441886,27.62597363283197)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 95 of 123"
path = []
start = (1.1511174672763573,-15.609905010532861)
goal = (32.78791912875753,-12.031027131205828)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 96 of 123"
path = []
start = (2.189304888783468,-14.900488278842445)
goal = (33.06720845811635,-11.963223082155192)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 97 of 123"
path = []
start = (1.5146968764966715,-0.8939203260562678)
goal = (32.47577145350221,-1.8279134129187913)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 98 of 123"
path = []
start = (-15.807740424228207,29.558523539527844)
goal = (35.67177816705057,24.96859389428992)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 99 of 123"
path = []
start = (-13.643705251672145,29.372668437128713)
goal = (35.96177405594968,35.77542862802488)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 100 of 123"
path = []
start = (5.510433629391997,-13.267527733368402)
goal = (33.17674364134029,-6.920410260668515)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 101 of 123"
path = []
start = (-21.708869996214084,32.9865159812304)
goal = (36.64993710593881,29.077412701587978)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 102 of 123"
path = []
start = (3.3494616981500087,0.8234997595208142)
goal = (34.240837855131495,1.4477185291052486)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 103 of 123"
path = []
start = (-4.07996484900206,15.607757942970704)
goal = (34.69820132529424,1.6024726413968011)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 104 of 123"
path = []
start = (5.917402761191866,-14.38642723575742)
goal = (34.68267004260097,-9.348389248253344)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 105 of 123"
path = []
start = (-19.55375755529733,32.4995570430096)
goal = (37.490196926356724,31.17462875819191)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 106 of 123"
path = []
start = (-24.915661631765058,36.127672376392596)
goal = (37.605059943293476,28.28924919187264)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 107 of 123"
path = []
start = (8.295453038644247,-14.36546527413362)
goal = (35.51858847746242,-13.556801751658682)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 108 of 123"
path = []
start = (-19.561598894129197,32.853817112333815)
goal = (38.199363323425736,33.0195632377082)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 109 of 123"
path = []
start = (-0.9216477735592079,12.72164439722567)
goal = (38.21383862474186,11.339455107247737)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 110 of 123"
path = []
start = (-6.395376220499131,23.559709871762106)
goal = (38.17467594668765,29.20447060296284)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 111 of 123"
path = []
start = (3.663045068148598,3.6629113006858773)
goal = (34.909545777192314,-3.639668072661383)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 112 of 123"
path = []
start = (8.535315029813468,-13.62268441477411)
goal = (35.65615664970826,-12.400894671935319)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 113 of 123"
path = []
start = (-13.770621469102228,30.30691031608738)
goal = (38.05369817895801,39.440032663732296)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 114 of 123"
path = []
start = (-19.888386423185384,34.66052517178984)
goal = (39.49286402747,39.078173823885024)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 115 of 123"
path = []
start = (6.930624697859308,-1.82416853083539)
goal = (34.654787472165104,-5.131903964592329)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 116 of 123"
path = []
start = (-5.574041025052814,26.62648409710425)
goal = (38.46836085076361,18.315285427130608)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 117 of 123"
path = []
start = (1.4759796499335778,10.716526859414905)
goal = (38.42469900809707,10.455453143897774)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 118 of 123"
path = []
start = (-3.294983732057144,26.201838324595084)
goal = (38.90912026034784,2.609508886641702)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 119 of 123"
path = []
start = (7.781454354577242,-1.4704947881961594)
goal = (36.64414017909073,-7.486270779821188)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 120 of 123"
path = []
start = (-16.982532477094747,32.83889947063827)
goal = (39.71300769210246,0.20887202312448494)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 121 of 123"
path = []
start = (-25.195048834136262,39.033238493019795)
goal = (37.37179671802246,-6.164732199335198)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 122 of 123"
path = []
start = (9.11427987842464,-4.67166613468701)
goal = (36.453619530139555,-11.691407750552647)
print "     Node 1 and 2 of 2"
path += rrtpath(obstacleList,start,goal,rand)
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
content += pathStr
print "Path 123 of 123"
path = []
start = (-12.795715668385103,36.57094379896172)
goal = (37.320190983651536,-10.96276603758697)
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
f = open('smo2sol-27.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
