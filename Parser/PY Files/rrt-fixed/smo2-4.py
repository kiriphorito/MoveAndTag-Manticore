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

obstacleList = [[(-0.05704426993442535,0.0443520770557942),(0.6212640289430875,-1.8371095493596623),(2.502725655358544,-1.1588012504821497)],[(-0.9064740078013211,1.5563537277545438),(-2.266583274985843,3.022676605336006),(-2.999744713776576,2.3426219717437435),(-2.319690080184314,1.6094605329530127),(-3.786012957765777,0.24935126576849131),(-3.1059583241735136,-0.4838101730222415),(-2.3727968853827823,0.19624446057002065),(-1.6927422517905226,-0.5369169782207104),(-3.159065129371985,-1.8970262454052325),(-2.4790104957797245,-2.6301876841959646),(-1.7458490569889917,-1.9501330506037022),(-1.065794423396728,-2.683294489394432),(-2.53211730097819,-4.043403756578957),(-1.8520626673859293,-4.776565195369688),(-1.118901228595197,-4.096510561777427),(-0.43884659500293377,-4.8296720005681575),(-1.9051694725843982,-6.189781267752679),(-1.225114838992135,-6.9229427065434095),(0.9743694773800569,-4.882778805766625),(-0.38573978980446577,-3.4164559281851647),(0.34742164898626493,-2.736401294592905),(-1.0126876181982591,-1.2700784170114416),(-0.2795261794075282,-0.5900237834191802),(-1.639635446592052,0.8762990941622821)],[(0.07994469527374685,0.11442905065670439),(0.9848878563433552,1.8979866962721594),(-0.7986697892720992,2.802929857341768)],[(0.41548312390178915,0.3165720753228515),(2.3794211388983237,-0.06151187468226521),(2.7575050889034407,1.9024261403142697)],[(2.9212853184366985,1.1999145575801127),(2.9304269235085063,-0.800064550074941),(3.93041647733604,-0.7954937475390369),(3.9258456748001294,0.20449580628848785),(5.9258247824551855,0.213637411360297),(5.921253979919283,1.2136269651878269),(4.921264426091753,1.2090561626519234),(4.916693623555849,2.209045716479447),(6.916672731210905,2.21818732155125),(6.912101928675,3.2181768753787843),(3.9121332671924205,3.2044644677710705),(3.921274872264225,1.2044853601160173)],[(-0.5004497882398188,3.26435635655471),(1.1890571430437014,4.334667680023231),(0.6539014813094429,5.179421145664993),(-0.1908519843323171,4.644265483930732),(-1.2611633078008424,6.333772415214252),(-2.1059167734426043,5.7986167534799895),(-1.5707611117083404,4.953863287838231),(-2.4155145773501014,4.418707626103973),(-3.4858259008186203,6.108214557387494),(-4.3305793664603875,5.5730588956532285),(-3.795423704726121,4.728305430011468),(-4.640177170367882,4.1931497682772125),(-3.2602680429918602,3.8835519643697145),(-2.7251123812576017,3.038798498727947),(-1.035605449974078,4.109109822196471)],[(3.128864905306475,-1.6433917402151164),(1.1779582749371784,-2.083804409857026),(1.3981646097581306,-3.0592577250416726),(2.3736179249427782,-2.8390513902207175),(2.8140305945846795,-4.789958020590016),(3.789483909769334,-4.569751685769063),(3.569277574948381,-3.594298370584413),(4.544730890133035,-3.374092035763459),(3.34907124012743,-2.618845055399764)],[(1.4031646729685128,2.7577485857396944),(3.178276861296175,1.8363492592527606),(4.099676187783109,3.6114614475804254)],[(-3.613497663466867,1.907875201562182),(-5.371517365084969,2.8614824208734633),(-6.325124584396251,1.1034627192553605)],[(6.594276450680278,-4.596777793792488),(4.80693358198492,-3.699333940549381),(4.3582116553633625,-4.593005374897054),(5.251883089711041,-5.04172730151861),(4.354439236467934,-6.8290701702139796),(5.248110670815616,-7.27779209683553),(5.696832597437172,-6.384120662487848),(6.590504031784852,-6.8328425891094025),(5.6930601785417325,-8.620185457804762),(6.5867316128894196,-9.068907384426321),(7.035453539510975,-8.175235950078637),(7.9291249738586504,-8.623957876700192),(7.484175466132523,-7.281564515730959),(7.932897392754088,-6.387893081383274),(6.145554524058721,-5.490449228140166)],[(-5.53230022144111,-1.1057478417996487),(-6.050967739179587,0.825827682492116),(-7.016755501325465,0.5664939236228823),(-6.757421742456229,-0.39929383852300204),(-8.688997266747997,-0.9179613562614731),(-8.42966350787876,-1.883749118407356),(-7.463875745732875,-1.6244153595381219),(-7.204541986863639,-2.590203121684006),(-9.136117511155403,-3.1088706394224834),(-8.876783752286173,-4.074658401568362),(-5.979420465848518,-3.296657124960651),(-6.498087983586993,-1.3650816006688853)],[(-3.649886963968159,-0.029049669017145133),(-5.623639969803588,-0.35200333745211215),(-5.300686301368623,-2.325756343287544)],[(-2.76779727331947,-10.222606926590974),(-4.7567676000659915,-10.43236163886658),(-4.651890243928191,-11.426846802239846),(-3.6574050805549287,-11.32196944610204),(-3.4476503682793216,-13.310939772848565),(-2.453165204906054,-13.206062416710761),(-2.5580425610438597,-12.211577253337495),(-1.5635573976706052,-12.106699897199698),(-2.6629199171816658,-11.217092089964236)],[(5.957043038765268,-1.1251041510017832),(5.6800121116067475,-3.1058246929757605),(7.66073265358072,-3.382855620134287)],[(-3.146070177724042,-3.5814568540851917),(-5.1460590319063,-3.5747797928509333),(-5.149397562523429,-4.574774219942071),(-4.1494031354323,-4.578112750559195),(-4.156080196666554,-6.578101604741455),(-3.156085769575419,-6.581440135358582),(-3.1527472389583004,-5.581445708267447),(-2.152752811867167,-5.58478423888458),(-3.149408708341172,-4.581451281176319)],[(8.562188493003658,1.1696245258451474),(6.562781487414283,1.1209251829325841),(6.587131158870564,0.12122168013789159),(7.586834661665251,0.145571351594177),(7.63553400457783,-1.853835653995205),(8.635237507372516,-1.8294859825389158),(8.610887835916234,-0.8297824797442286),(9.610591338710925,-0.8054328082879403),(8.586538164459943,0.16992102305045986)],[(-6.782916172193694,5.4779140368818116),(-6.828906843642381,7.477385181494728),(-8.828377988255301,7.431394510046042)],[(10.252901230515604,-3.003544318334004),(8.328772611887683,-2.4578987749846695),(8.055949840213016,-3.4199630842986317),(9.01801414952697,-3.692785855973295),(8.472368606177644,-5.616914474601227),(9.434432915491602,-5.8897372462758915),(9.707255687166267,-4.927672936961926),(10.66931999648022,-5.200495708636601),(10.123674453130887,-7.124624327264522),(11.085738762444848,-7.397447098939194),(11.358561534119515,-6.435382789625229),(12.320625843433472,-6.708205561299903),(11.631384305794183,-5.473318480311271),(11.904207077468858,-4.511254170997309),(9.980078458840937,-3.965608627647965)],[(5.078144240344647,3.9563259003108913),(7.02338944293929,3.4915451974170164),(7.255779794386228,4.464167798714341),(6.283157193088907,4.696558150161278),(6.747937895982782,6.641803352755919),(5.77531529468546,6.874193704202854),(5.542924943238522,5.901571102905533),(4.570302341941199,6.133961454352471),(5.310534591791584,4.928948501608213)],[(2.059772623223469,-9.575605771852345),(2.247467112144429,-11.566778988533578),(4.238640328825664,-11.37908449961262)]]
rand = (-8, 10)

content = ""
starttime = datetime.datetime.now()
print "Path 1 of 1"
path = []
start = (1.7611158486517908,-7.747711347622252)
goal = (4.545065131048604,7.011566783399479)
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
f = open('smo2sol-4.txt', 'w')
f.write(content)
f.close

#plt.axis('scaled')
#plt.grid(True)
#plt.pause(0.01)  # Need for Mac
#plt.show()
