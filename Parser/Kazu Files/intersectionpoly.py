import shapely
from shapely.geometry import Polygon, LineString, Point

polygonpoints = [[(1,2),(1,4),(3,4),(3,2)],[(8,1),(4,1),(4,4),(5,2)]]

#def onepolymultline(polygonpoints,multiline):
#    p1 = Polygon(polygonpoints)
#    for line in multiline:
#        p2 = LineString(line)
#        if p2.intersects == True:
#            print "true"
#            return True
#    print "false"
#    return False


def polysOneLine(polygonpoints,line):
    p2 = LineString(line)
    for poly in polygonpoints:
        p1 = Polygon(poly)
        if p2.intersects(p1) == True:
            print "trure"
            return True
    print "false"
    return False

def checkifPointisinbound(polygonpoints,point):
    p2 = Point(point)
    for poly in polygonpoints:
        p1 = Polygon(poly)
        if p1.contains(p2):
            print "true"
            return True
    print "false"
    return False

#
#p1=Polygon(polygonpoints)
#p2=LineString([(-1.0, -1.0), (0.0, 6.0)])

#polysOneLine(polygonpoints, [(-1.0, -1.0), (0.0, 6.0)])

checkifPointisinbound(polygonpoints,(7,7))

p1=Polygon(polygonpoints[0])
#print p1.contains(Point(1,2))
#[[(-1.0, -1.0), (0.0, 6.0), (1.0, 6.0), (2.0, 2.0), (4.0, 2.0), (4.0, 3.0), (4.0, 4.0)]]

#print p2.intersects(p1)

