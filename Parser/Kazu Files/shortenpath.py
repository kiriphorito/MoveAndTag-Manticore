
import shapely
from shapely.geometry import Polygon, LineString, Point
import matplotlib.pyplot as plt
from ast import literal_eval
#drayton bo4
import random
import math
import copy


def drawPolygon(points):
    polygon = plt.Polygon(points)
    plt.gca().add_patch(polygon)

def drawPolygons(polygons):
    try:
        for xs in polygons:
            drawPolygon(xs)
    except ValueError:
        print ("no polygons specified")

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

path = [[5, 10], [4.512201320082587, 9.856422365292817], [4.512201320082587, 9.856422365292817], [3.3994930330723685, 2.7316622218621864], [3.3994930330723685, 2.7316622218621864], [2.938889542216369, 1.827169941829606], [2.938889542216369, 1.827169941829606], [0.3054606643014337, 0.9522046957269885], [0.3054606643014337, 0.9522046957269885], [0, 0]]

#def supersmoothie():
#    for counter1 in range(0,len(path)-1):
#        print ("counter1 = ",counter1)
#        print path
#        coord1 = path[counter1]
#        print ("coord 1",coord1,"counter",counter1)
#        for counter in range(len(path)-1,0,-1):
#            coord2 = path[counter]
#            if LineCollisionCheck(coord1,coord2,obstacleList): #if no obstacle
#                print ("no obstacle between",coord1, coord2)
#                if counter1 != counter:
#                    del path[(counter1+1):(counter-1)]

def supersmoothie():
    state = True
    counter1 = 0
    counter2 = len(path)-1
    while state:
        counter2 = len(path)-1
        if counter1 == counter2:
            state = False
            break
        coord1 = path[counter1]
        print ("coord 1",coord1,"counter",counter1)
        for counter in range(counter2,0,-1):
            coord2 = path[counter]
            print ("coord 2",coord2,"counter",counter)
            if LineCollisionCheck(coord1,coord2,obstacleList): #if no obstacle
                print ("no obstacle between",coord1, coord2,(counter1+1),(counter-1))
                del path[(counter1+1):(counter)]
                print ("new path",path)
                break
        counter1 += 1







#                    print ("new path",path)
#            else:
#                print ("obstacle between",coord1, coord2)
#            print coord2



obstacleList = [[(1,2),(1,4),(3,4),(3,2)],[(8,1),(4,1),(4,4),(5,2)]]
plt.axis()

drawPolygons(obstacleList)
plt.plot([x for (x,y) in path], [y for (x,y) in path],'-b')

supersmoothie()
plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')
plt.show()


