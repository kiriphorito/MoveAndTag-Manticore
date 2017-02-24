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

obstacleList = [[(0.10887588040823118,0.09142047210771191),(1.1039623485842764,-0.007589228694404357),(0.905942946980044,-1.997762165046495),(-3.074402925724136,-1.601723361838031),(-3.1734126265262517,-2.596809830014076),(-1.1832396901741624,-2.794829231618308),(-1.282249390976279,-3.789915699794353),(-13.223287009088821,-2.6017992901689646),(-13.42130641069305,-4.591972226521057),(-12.426219942517006,-4.690981927323172),(-13.0202781473297,-10.66150073637944),(-12.025191679153657,-10.760510437181557),(-11.431133474340962,-4.789991628125285),(-1.4802687925805116,-5.780088636146442),(-1.5792784933826283,-6.7751751043224875),(-0.5841920252065823,-6.874184805124604),(-0.18815322199811743,-2.8938389324204237),(0.8069332461779276,-2.99284863322254),(0.5099041437715794,-5.978108037750675),(1.5049906119476242,-6.077117738552792),(1.6040003127497404,-5.082031270376746),(2.599086780925785,-5.181040971178863),(1.2129509696961562,-19.112251525643497),(-4.75756783936011,-18.518193320830797),(-4.9555872409643476,-20.50836625718289),(1.0149315680919284,-21.102424461995586),(0.8169121664876853,-23.092597398347678),(3.8021715710158395,-23.389626500754026),(4.000190972620064,-21.399453564401938),(39.82330382695769,-24.963802793278123),(40.41736203177039,-18.993283984221858),(4.594249177432759,-15.428934755345662),(5.584346185453922,-5.47807007358521),(7.574519121806013,-5.676089475189441),(7.6735288226081275,-4.681003007013399),(10.658788227136265,-4.978032109419745),(10.757797927938379,-3.9829456412437065),(7.772538523410243,-3.6859165388373536),(7.8715482242123604,-2.6908300706613093),(1.9010294151560894,-2.0967718658486114),(2.0990488167603214,-0.10659892949652063),(3.0941352849363666,-0.2056086302986369),(3.1931449857384826,0.7894778378774103),(10.158750262970798,0.09640993226260114),(10.257759963772914,1.0914964004386465),(8.267587027420824,1.2895158020428774),(8.36659672822294,2.2846022702189224),(14.33711553727921,1.6905440654062291),(14.436125238081328,2.685630533582274),(8.465606429025057,3.2796887383949676),(8.56461612982717,4.274775206571014),(24.485999620643895,2.6906199937371635),(24.684019022248126,4.680792930089255),(8.762635531431403,6.264948142923105),(8.960654933035634,8.255121079275195),(7.965568464859588,8.354130780077309),(7.767549063255357,6.3639578437252196),(6.772462595079312,6.462967544527336),(7.168501398287773,10.443313417231517),(5.178328461935683,10.641332818835748),(4.78228965872722,6.660986946131567),(3.7872031905511765,6.759996646933682),(3.6881934897490614,5.764910178757638),(7.668539362453242,5.368871375549174),(7.47051996084901,3.378698439197083),(5.480347024496919,3.576717840801315),(5.3813373236948046,2.58163137262527),(7.371510260046895,2.3836119710210384),(7.272500559244779,1.388525502844993),(3.2921546865405977,1.7845643060534553),(3.3911643873427133,2.7796507742295002),(2.3960779191666686,2.8786604750316163),(3.1881555255835923,10.839352220439979),(2.193069057407549,10.938361921242095),(1.9950496558033173,8.948188984890006),(0.004876719451224712,9.146208386494235),(0.4999252234618028,14.12164072737446),(-0.4951612447142417,14.220650428176578),(-0.9902097487248201,9.245218087296351),(-1.9852962169008654,9.344227788098467),(-1.2922283112860569,16.309833065330782),(2.688117561418126,15.913794262122323),(2.7871272622202405,16.908880730298367),(-1.1932186104839406,17.304919533506826),(-0.9951992088797089,19.29509246985892),(19.90161662281724,17.21588875301449),(20.000626323619358,18.210975221190534),(27.961318069027723,17.418897614773613),(28.060327769829833,18.413984082949657),(20.09963602442147,19.20606168936658),(20.19864572522359,20.201148157542626),(-0.6981701064733642,22.280351874387055),(-0.4011410040670157,25.26561127891519),(-4.3814868767711985,25.661650082123654),(-5.965642089605046,9.740266591306927),(-6.960728557781089,9.839276292109044),(-1.4161853128626316,65.56411850996759),(-9.376877058270994,66.3561961163845),(-14.921420303189453,10.63135389852597),(-18.901766175893634,11.02739270173443),(-18.208698270278827,17.99299797896674),(-19.20378473845487,18.092007679768855),(-19.89685264406968,11.126402402536549),(-21.887025580421767,11.324421804140776),(-22.28306438363023,7.344075931436599),(1.599010852594854,4.967843112185823),(1.4009914509906234,2.9776701758337323),(0.4059049828145781,3.076679876635848),(0.30689528201246247,2.0815934084598027),(-6.658709995219853,2.774661314074613),(-6.75771969602197,1.7795748458985678),(0.20788558121034684,1.0865069402837573)],[(-24.246837974034875,-28.490829719121777),(-24.743325915820897,-29.358873337646244),(-27.34745677139434,-27.869409512288186),(-27.843944713180363,-28.737453130812685),(-26.97590109465588,-29.2339410725987),(-29.458340803586015,-33.574159165221104),(-28.590297185061505,-34.07064710700715),(-26.107857476131404,-29.730429014384715),(-25.23981385760692,-30.226916956170736),(-25.736301799392958,-31.094960574695207),(-24.868258180868466,-31.591448516481243),(-25.861234064440506,-33.32753575353023),(-24.993190445916074,-33.82402369531623),(-24.000214562343995,-32.087936458267265),(-23.1321709438195,-32.584424400053294),(-22.63568300203349,-31.716380781528816),(-21.76763938350899,-32.212868723314834),(-23.75359115065313,-35.685043197412774),(-26.35772200622651,-34.1955793720547),(-26.85420994801257,-35.063622990579184),(-24.250079092439112,-36.553086815937256),(-24.74656703422513,-37.42113043446172),(-23.01047979717619,-38.41410631803377),(-22.51399185539019,-37.54606269950931),(-21.64594823686566,-38.04255064129532),(-22.63892412043774,-39.778637878344306),(-24.37501135748671,-38.785661994772234),(-24.871499299272728,-39.65370561329672),(-23.135412062223804,-40.64668149686878),(-23.63190000400979,-41.514725115393276),(-22.76385638548528,-42.01121305717929),(-20.777904618341182,-38.53903858308134),(-19.90986099981669,-39.03552652486735),(-22.392300708746866,-43.37574461748979),(-31.072736893991625,-38.41086519962956),(-32.06571277756373,-40.14695243667849),(-23.385276592318977,-45.111831854538735),(-24.378252475890957,-46.847919091587706),(-28.718470568513336,-44.3654793826576),(-29.21495851029926,-45.23352300118211),(-27.478871273250494,-46.22649888475411),(-27.975359215036338,-47.094542503278625),(-31.447533689134232,-45.10859073613457),(-31.944021630920307,-45.97663435465901),(-30.20793439387137,-46.96961023823104),(-30.704422335657462,-47.83765385675554),(-35.04464042827985,-45.35521414782542),(-34.051664544707755,-43.619126910776444),(-34.91970816323219,-43.1226389689905),(-35.912684046804266,-44.85872620603948),(-36.78072766532869,-44.362238264253435),(-37.277215607114755,-45.230281882777916),(-36.4091719885903,-45.726769824563874),(-37.402147872162224,-47.46285706161295),(-45.21454043888265,-42.994465585538705),(-45.711028380668665,-43.86250920406323),(-50.05124647329119,-41.38006949513293),(-50.54773441507729,-42.24811311365735),(-46.20751632245484,-44.73055282258757),(-46.70400426424072,-45.59859644111208),(-38.8916116975204,-50.066987917186374),(-39.38809963930646,-50.935031535710806),(-41.12418687635535,-49.942055652138826),(-41.620674818141424,-50.81009927066326),(-39.88458758109249,-51.80307515423527),(-40.38107552287865,-52.671118772759655),(-39.51303190435404,-53.16760671454575),(-35.54112837006582,-46.2232577663499),(-34.673084751541374,-46.71974570813593),(-36.65903651868551,-50.191920182233844),(-34.92294928163646,-51.184896065805916),(-32.93699751449242,-47.712721591707975),(-31.2009102774434,-48.70569747528002),(-32.19388616101542,-50.441784712329),(-31.325842542490946,-50.93827265411502),(-29.339890775346838,-47.466098180017084),(-28.47184715682239,-47.962586121803085),(-28.968335098608424,-48.830629740327595),(-28.10029148008388,-49.327117682113595),(-26.610827654725956,-46.72298682654018),(-24.874740417676946,-47.715962710112194),(-25.867716301248908,-49.45204994716118),(-24.13162906419995,-50.44502583073324),(-23.635141122414055,-49.576982212208726),(-21.89905388536502,-50.56995809578078),(-22.892029768937146,-52.30604533282974),(-22.023986150412732,-52.8025332746158),(-21.031010266840603,-51.06644603756682),(-20.162966648316164,-51.562933979352835),(-19.169990764744092,-49.8268467423039),(-22.642165238842082,-47.8408949751598),(-18.17377376276776,-40.02850240843944),(-16.437686525718796,-41.021478292011466),(-19.41661417643492,-46.229740003158376),(-18.548570557910452,-46.72622794494441),(-15.569642907194252,-41.51796623379748),(-14.701599288669806,-42.01445417558353),(-18.673502822957982,-48.9588031237794),(-17.805459204433493,-49.45529106556539),(-16.81248332086139,-47.7192038285164),(-15.944439702336968,-48.215691770302485),(-20.412831178411377,-56.028084337022875),(-17.808700322837804,-57.51754816238086),(-13.340308846763563,-49.70515559566057),(-12.472265228239134,-50.201643537446586),(-16.444168762527234,-57.14599248564241),(-14.708081525478326,-58.13896836921445),(-12.722129758334253,-54.66679389511654),(-2.3056063360404835,-60.62464919654888),(-1.3126304524683619,-58.88856195949985),(-11.729153874762188,-52.930706658067635),(-10.736177991190118,-51.19461942101867),(-9.868134372665613,-51.691107362804615),(-9.371646430879542,-50.82306374428009),(-16.31599537907546,-46.85116020999197),(-15.323019495503349,-45.11507297294297),(-6.642583310258633,-50.07995239080324),(-5.649607426686526,-48.34386515375431),(-14.330043611931314,-43.37898573589403),(-13.833555670145318,-42.51094211736956),(-8.62529395899841,-45.48986976808571),(-7.632318075426419,-43.75378253103675),(-21.521015971818098,-35.80997546246032),(-21.0245280300321,-34.941931843935855),(-18.420397174458653,-36.43139566929394),(-17.923909232672553,-35.56335205076942),(-20.528040088246062,-34.073888225411395),(-20.031552146460037,-33.2058446068869),(-15.69133405383761,-35.688284315816986),(-15.194846112051579,-34.82024069729252),(-16.06288973057603,-34.32375275550647),(-15.069913847004027,-32.58766551845756),(-9.861652135857122,-35.566593169173736),(-10.854628019429134,-37.30268040622266),(-9.986584400904718,-37.79916834800872),(-8.993608517332634,-36.063081110959686),(-8.125564898808125,-36.559569052745694),(-7.629076957022125,-35.69152543422122),(-14.573425905217983,-31.71962189993304),(-13.58045002164594,-29.983534662884065),(-8.372188310499041,-32.96246231360016),(-7.875700368713023,-32.09441869507579),(-10.479831224286471,-30.60495486971766),(-9.486855340714438,-28.868867632668607),(-10.35489895923891,-28.37237969088267),(-11.347874842810956,-30.108466927931588),(-13.083962079859937,-29.115491044359608),(-12.090986196287885,-27.379403807310606),(-13.82707343333683,-26.386427923738584),(-13.330585491550817,-25.51838430521413),(-8.12232378040396,-28.497311955930265),(-7.129347896831874,-26.76122471888129),(-12.33760960797881,-23.78229706816507),(-11.841121666192787,-22.91425344964059),(1.1795326116744818,-30.361572576431005),(0.18655672810243829,-32.09765981347989),(-1.5495305089465425,-31.104683929907907),(-2.0460184507325394,-31.972727548432406),(-0.3099312136836119,-32.96570343200437),(-1.3029070972557086,-34.70179066905344),(-6.511168808402523,-31.72286301833727),(-7.0076567501885485,-32.59090663686165),(-1.7993950390416522,-35.56983428757784),(-2.295882980827667,-36.43787790610241),(-1.427839362303235,-36.934365847888465),(1.0546003466269305,-32.59414775526602),(3.658731202200382,-34.08361158062402),(4.155219143986418,-33.215567962099534),(1.5510882884129416,-31.726104136741554),(2.04757623019902,-30.858060518217),(6.387794322821403,-33.34050022714716),(6.884282264607407,-32.47245660862261),(11.224500357229893,-34.954896317552745),(11.720988299015872,-34.086852699028185),(7.380770206393517,-31.604412990098258),(7.8772581481793935,-30.736369371573765),(-10.351657840834768,-20.310122594067117),(-9.85516989904869,-19.44207897554267),(-10.723213517573202,-18.9455910337566),(-11.219701459359246,-19.813634652281053),(-47.67753343738744,1.0388589027317927),(-50.65646108810366,-4.1694028084151675),(-14.198629110075316,-25.021896363428016),(-14.695117051861338,-25.889939981952526),(-15.563160670385818,-25.393452040166505),(-16.05964861217184,-26.26149565869102),(-29.948346508563603,-18.31768859011467),(-30.941322392135604,-20.053775827163616),(-17.052624495743892,-27.997582895739992),(-19.535064204674015,-32.33780098836241),(-20.403107823198514,-31.84131304657641),(-19.41013193962644,-30.105225809527415),(-20.27817555815091,-29.60873786774138),(-21.27115144172298,-31.344825104790367),(-22.13919506024745,-30.848337163004338),(-21.642707118461438,-29.98029354447985),(-22.510750736985923,-29.48380560269384),(-21.02128691162784,-26.87967474712036),(-21.88933053015232,-26.38318680533435),(-22.385818471938343,-27.25123042385883),(-24.121905708987317,-26.25825454028678),(-24.61839365077332,-27.12629815881128),(-22.88230641372437,-28.11927404238332),(-23.378794355510404,-28.98731766090781)],[(45.20081810829839,2.198693603713543),(46.17018887941854,1.9530916735543375),(45.18778115878171,-1.9243914109262543),(46.157151929901886,-2.169993341085459),(47.13955965053867,1.7074897433951215),(48.108930421658826,1.4618878132359123),(48.354532351818044,2.431258584356081),(55.14012774965909,0.7120450732416423),(55.385729679818304,1.681415844361834),(53.446988137577996,2.1726197046802467),(54.42939585821482,6.050102789160792),(68.0005866538969,2.6116757669319846),(49.334839961797165,-71.06050283819941),(54.18169381739828,-72.28851248899544),(50.25206293485121,-87.79844482691769),(58.00702910381231,-89.7632602681915),(61.93665998635927,-74.25332793026902),(62.906030757479854,-74.49892986042826),(56.02917671302153,-101.64131145179253),(59.906659797502286,-102.62371917242933),(60.88906751813935,-98.7462360879487),(91.90893219398414,-106.60549785304322),(92.8913399146206,-102.72801476856276),(61.871475238776185,-94.868753003468),(66.78351384196029,-75.48133758106509),(67.7528846130803,-75.72693951122432),(86.41863130517979,-2.0547609060927727),(95.14296824526107,-4.265178277525488),(96.86218175637549,2.5204171203155603),(56.14860936932918,12.835698187001881),(56.88541515980677,15.74381050036238),(54.94667361756656,16.235014360680704),(51.508246595337695,2.6638235649986415),(48.60013428197725,3.400629355476238),(48.84573621213646,4.370000126596378),(47.8763654410163,4.615602056755581),(48.367569301334704,6.554343598995885),(50.306310843575005,6.063139738677501),(50.55191277373424,7.0325105097976),(48.613171231493894,7.523714370116037),(48.858773161653104,8.493085141236191),(47.88940239053292,8.738687071395411),(47.64380046037377,7.76931630027523),(46.67442968925359,8.01491823043444),(47.16563354957203,9.95365977267471),(46.196262778451874,10.199261702833924),(45.705058918133474,8.260520160593641),(44.73568814701327,8.506122090752836),(48.910920959719704,24.985425199795415),(60.54337021316148,22.03820203788503),(61.03457407347996,23.976943580125287),(63.94268638684039,23.240137789647708),(64.18828831699955,24.20950856076788),(61.280176003639134,24.94631435124549),(61.52577793379831,25.915685122365513),(49.89332868035652,28.862908284276024),(50.63013447083407,31.77102059763644),(49.66076369971399,32.01662252779561),(50.39756949019143,34.92473484115609),(49.428198719071446,35.1703367713153),(48.69139292859387,32.26222445795486),(47.72202215747368,32.50782638811402),(47.47642022731442,31.53845561699389),(23.242150949310854,37.678503870974),(22.014141298514563,32.8316500153731),(46.2484105765184,26.691601761393162),(45.26600285588167,22.814118676912575),(29.756070517959312,26.74374955945985),(27.79125507668565,18.988783390498536),(43.30118741460808,15.059152507951346),(42.56438162413046,12.151040194590916),(36.748156997409545,13.624651775546113),(36.50255506725035,12.655281004425955),(42.31877969397127,11.181669423470737),(41.82757583365285,9.242927881230445),(39.88883429141257,9.73413174154887),(39.64323236125336,8.764760970428723),(47.398198530214565,6.799945529155094),(46.906994669896164,4.8612039869147825),(45.937623898775996,5.1068059170739915),(45.6920219686168,4.137435145953841),(41.81453888413622,5.119842866590661),(41.568936953976994,4.150472095470501),(45.4464200384576,3.168064374833683)],[(-34.470071299470696,16.54613794841084),(-35.058908409090435,17.354389677969554),(-33.442404949972996,18.532063897209046),(-34.03124205959274,19.340315626767772),(-35.64774551871018,18.16264140752828),(-36.236582628329906,18.970893137087007),(-38.66133781700609,17.20438180822779),(-38.072500707386354,16.39613007866905),(-39.689004166503764,15.218455859429536),(-49.110397920419764,28.15048353236913),(-37.79487370659765,36.39420306704554),(-23.073945966104,16.18790982807754),(-19.0326873183104,19.132095376176387),(-33.75361505880397,39.338388615144254),(-32.137111599686655,40.516062834383746),(-32.725948709306316,41.324314563942494),(-31.10944525018884,42.50198878318193),(-31.69828235980868,43.31024051274066),(-33.31478581892613,42.1325662935012),(-33.90362292854584,42.94081802305989),(-50.876909249278924,30.575238721045288),(-51.46574635889863,31.38349045060414),(-53.082249818016166,30.205816231364473),(-57.204109585354246,35.863578338275644),(-58.012361314913115,35.274741228655756),(-56.834687095673615,33.65823776953829),(-60.067694013908444,31.302889331059422),(-64.77839089086649,37.76890316752914),(-65.58664262042525,37.18006605790926),(-60.87594574346719,30.71405222143965),(-62.49244920258464,29.536378002200134),(-61.90361209296492,28.728126272641532),(-56.24584998605381,32.84998603997967),(-55.06817576681435,31.233482580862223),(-57.49293095549058,29.466971252002843),(-56.90409384587082,28.65871952244413),(-54.479338657194624,30.425230851303468),(-53.89050154757489,29.6169791217447),(-54.69875327713352,29.028142012125006),(-42.922011084738635,12.863107420950525),(-43.73026281429737,12.274270311330774),(-43.141425704677594,11.466018581772055),(-40.716670516001486,13.232529910631323),(-36.00597363904339,6.7665160741615615),(-34.389470179925986,7.944190293401066),(-39.100167056884025,14.410204129870817),(-37.483663597766586,15.58787834911033),(-36.89482648814685,14.779626619551598),(-36.08657475858812,15.36846372917135),(-33.73122632010911,12.13545681093646),(-32.92297459055042,12.724293920556219),(-34.100648809789924,14.340797379673642),(-30.059390161996323,17.284982927772393),(-30.648227271616072,18.09323465733113),(-34.68948591940967,15.14904910923237),(-35.278323029029416,15.957300838791099)],[(10.627782914793798,-49.8988543995859),(11.159320827873213,-50.74588890122038),(9.465251824604236,-51.8089647273792),(7.339100172286599,-48.420826720841255),(6.492065670652125,-48.95236463392063),(7.555141496810892,-50.64643363718961),(5.861072493541947,-51.709509463348496),(6.392610406621339,-52.556543964982936),(8.08667940989033,-51.49346813882412),(8.618217322969738,-52.34050264045863),(7.7711828213352545,-52.872040553538035),(8.302720734414669,-53.71907505517252),(9.14975523604913,-53.18753714209307),(10.744368975287435,-55.728640646996574),(6.509196467115033,-58.38633021239367),(7.040734380194433,-59.23336471402817),(11.275906888366897,-56.57567514863104),(11.807444801446287,-57.422709650265524),(3.337099785101419,-62.738088781059695),(4.4001756112602575,-64.43215778432868),(12.870520627605195,-59.11677865353448),(13.933596453763963,-60.81084765680346),(12.239527450494982,-61.873923482962304),(12.77106536357442,-62.720957984596794),(14.46513436684336,-61.657882158437936),(14.99667227992279,-62.50491666007247),(-5.3321557593048095,-75.26182657397844),(-4.800617846225377,-76.10886107561296),(-5.6476523478599105,-76.64039898869238),(-6.710728174018566,-74.94632998542353),(-7.557762675653134,-75.47786789850284),(-6.494686849494283,-77.17193690177166),(-8.188755852763244,-78.23501272793068),(-9.783369592001527,-75.69390922302719),(-10.630404093636024,-76.22544713610667),(-9.035790354397978,-78.76655064100991),(-11.576893859301194,-80.36116438024834),(-11.04535594622184,-81.2081988818828),(-4.269079933145939,-76.95589557724749),(-3.737542020066501,-77.80293007888193),(11.509079009354028,-68.23524764345244),(12.040616922433557,-69.08228214508685),(9.499513417530245,-70.67689588432515),(10.031051330609444,-71.52393038595963),(10.87808583224389,-70.9923924728802),(11.941161658402773,-72.68646147614916),(12.788196160037426,-72.15492356306973),(11.725120333878444,-70.46085455980082),(12.572154835513047,-69.92931664672136),(13.103692748592517,-70.77635114835587),(13.950727250226974,-70.24481323527642),(12.35611351098867,-67.70370973037298),(13.203148012623139,-67.17217181729359),(15.860837578020202,-71.40734432546598),(-15.479438982455584,-91.07424710940448),(-16.542514808614435,-89.38017810613539),(-8.072169792269815,-84.06479897534123),(-9.135245618428367,-82.37072997207241),(-11.676349123331942,-83.9653437113106),(-15.397114514887974,-78.03610219986919),(-16.24414901652243,-78.5676401129486),(-12.523383624966357,-84.49688162438997),(-13.37041812660102,-85.02841953746949),(-17.622721431236283,-78.25214352439353),(-10.846445418160403,-73.99984021975813),(-11.909521244319315,-72.30577121648932),(-18.685797257395127,-76.5580745211246),(-19.74887308355403,-74.86400551785552),(-22.28997658845745,-76.45861925709389),(-15.911521631504414,-86.62303327670764),(-17.60559063477342,-87.68610910286644),(-18.668666460932066,-85.99204009959763),(-21.209769965835616,-87.58665383883576),(-18.020542487359048,-92.66886084864277),(-23.94978399880044,-96.38962624019877),(-53.715907131248144,-48.95569414866712),(-59.64514864268929,-52.67645954022305),(-29.879025510241938,-100.11039163175451),(-31.573094513511123,-101.17346745791325),(-29.446942861192838,-104.56160546445153),(-34.529149871000286,-107.75083294292766),(-33.9976119579205,-108.59786744456235),(-28.91540494811376,-105.40863996608584),(-27.852329121955044,-107.10270896935472),(19.58160296957632,-77.3365858369073),(20.644678795735004,-79.03065484017638),(22.338747799003976,-77.96757901401747),(14.897217015892071,-66.10909599113474),(16.59128601916107,-65.04602016497589),(17.65436184531997,-66.74008916824482),(19.348430848588798,-65.67701334208603),(18.285355022430068,-63.982944338817134),(19.132389524064386,-63.45140642573755),(31.889299437970685,-83.78023446496515),(34.43040294287405,-82.18562072572686),(21.67349302896802,-61.85679268649932),(22.520527530602457,-61.325254773419914),(28.898982487555504,-71.48966879303374),(30.593051490824465,-70.42659296687485),(26.872286099268496,-64.4973514554335),(30.260424105806496,-62.371199803115694),(29.19734827964763,-60.677130799846815),(25.809210273109628,-62.80328245216454),(24.214596533871443,-60.26217894726105),(33.531976051850705,-54.41526190338752),(33.0004381387713,-53.568227401753),(37.23561064694367,-50.91053783635592),(36.704072733864294,-50.063503334721396),(35.857038232229826,-50.59504124780089),(33.73088657991208,-47.20690324126298),(32.883852078277684,-47.73844115434232),(35.01000373059533,-51.12657916088025),(34.16296922896085,-51.658117073959744),(30.442203837404936,-45.72887556251833),(29.595169335770443,-46.26041347559776),(33.315934727326336,-52.18965498703921),(32.46890022569187,-52.72119290011854),(31.937362312612482,-51.87415839848412),(16.6907412831918,-61.441840833913616),(15.627665457032936,-59.74777183064464),(29.18021748318467,-51.24316522137395),(27.05406583086698,-47.85502721483602),(13.501513804715252,-56.35963382410671),(10.843824239318108,-52.124461315934276),(11.690858740952606,-51.59292340285486),(12.22239665403203,-52.43995790448935),(13.06943115566651,-51.908419991409936),(14.132506981825376,-53.602488994678886),(14.979541483459844,-53.07095108159949),(13.91646565730101,-51.376882078330524),(14.763500158935507,-50.8453441652511),(14.231962245856105,-49.99830966361662),(15.926031249125057,-48.93523383745781),(18.052182901442713,-52.323371843995716),(19.746251904711695,-51.260296017836865),(19.214713991632273,-50.41326151620239),(21.755817496535748,-48.818647776964106),(21.224279583456287,-47.9716132753296),(18.683176078552805,-49.56622701456792),(17.620100252394014,-47.87215801129895),(20.16120375729747,-46.2775442720607),(19.62966584421806,-45.43050977042622),(18.782631342583592,-45.96204768350566),(17.71955551642473,-44.26797868023668),(19.413624519693684,-43.20490285407781),(22.071314085090815,-47.44007536225024),(22.918348586725273,-46.908537449170794),(22.386810673645847,-46.06150294753635),(24.92791417854935,-44.46688920829816),(26.52252791778753,-47.00799271320142),(27.369562419422017,-46.47645480012204),(25.77494868018381,-43.93535129521869),(26.621983181818237,-43.403813382139205),(27.685059007977063,-45.09788238540818),(28.53209350961162,-44.56634447232869),(27.46901768345273,-42.87227546905981),(29.163086686721766,-41.80919964290105),(28.631548773642415,-40.96216514126659),(25.24341076710428,-43.08831679358423),(20.991107462469074,-36.312040780508354),(20.144072960834606,-36.84357869358779),(24.396376265469925,-43.619854706663666),(21.85527276056645,-45.2144684459019),(21.32373484748704,-44.36743394426742),(23.017803850755957,-43.304358118108546),(22.486265937676553,-42.45732361647409),(20.792196934407578,-43.52039944263289),(20.26065902132818,-42.673364940998404),(21.10769352296263,-42.14182702791897),(20.576155609883262,-41.29479252628457),(17.188017603345365,-43.42094417860225),(16.656479690265904,-42.573909676967716),(15.809445188631472,-43.10544759004716),(17.93559684094908,-46.493585596585056),(17.088562339314596,-47.02512350966448),(14.430872773917537,-42.7899510014921),(20.360114285358936,-39.06918560993616),(19.828576372279564,-38.22215110830167),(16.440438365741606,-40.34830276061938),(12.71967297418571,-34.41906124917794),(11.872638472551193,-34.95059916225735),(15.593403864107128,-40.87984067369881),(13.899334860838144,-41.942916499857574),(13.36779694775875,-41.09588199822308),(12.52076244612426,-41.62741991130251),(16.24152783768012,-47.55666142274392),(15.394493336045642,-48.08819933582332),(14.33141750988683,-46.39413033255436),(13.484383008252314,-46.925668245633744),(14.547458834411149,-48.61973724890274),(13.700424332776656,-49.15127516198211),(13.168886419697259,-48.30424066034767),(12.321851918062766,-48.835778573427106),(9.13262443958629,-43.75357156362015),(8.28558993795181,-44.285109476699574),(11.474817416428296,-49.36731648650649)]]
rand = (-120,107)

content = ""
starttime = datetime.datetime.now()
print "Path 21 of 111"
path = []
start = (-22.391522819322645,-77.04046549591655)
goal = (-11.26585765401034,-80.92416469298728)
print "     Node 1 and 2 of 6"
path += rrtpath(obstacleList,start,goal,rand)
start = (-11.26585765401034,-80.92416469298728)
goal = (7.608419833556212,-89.08072326685475)
print "     Node 2 and 3 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (7.608419833556212,-89.08072326685475)
goal = (10.952382104555241,-91.03511609426907)
print "     Node 3 and 4 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (10.952382104555241,-91.03511609426907)
goal = (19.94952534224403,-91.84261723683866)
print "     Node 4 and 5 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
start = (19.94952534224403,-91.84261723683866)
goal = (69.56419206774986,-103.33062783874577)
print "     Node 5 and 6 of 6"
path += rrtpath(obstacleList,start,goal,rand)[1:]
pathStr = str(path)[1:-1] + ";"
pathStr = pathStr.replace("[", "(")
pathStr = pathStr.replace("]", ")")
f = open('smo2sol-24-path-21.txt', 'a+')
f.write(pathStr)
f.close
