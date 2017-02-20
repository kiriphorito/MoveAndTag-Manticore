import matplotlib.pyplot as plt



def drawPolygon(points):
    polygon = plt.Polygon(points)
    plt.gca().add_patch(polygon)

def drawPolygons(polygons):
    try:
        for xs in polygons:
            drawPolygon(xs)
    except ValueError:
        print ("no polygons specified")


def drawLine(points):
    line = plt.Polygon(points, closed=None, fill=None, edgecolor='r')
    plt.gca().add_patch(line)

def drawLines(lines):
    for xs in lines:
        drawLine(xs)

def drawRobots(robots):
    for (x,y) in robots:
        plt.plot(x,y,"o")


plt.axes()




#polypoints = [[(1,6),(1,1),(5,1),(5,5),(3,5),(3,3),(4,3),(4,2),(2,2),(2,6),(6,6),(6,0),(0,0),(0,6)]]
#linepoints = [[(-1, -1), (0, 6), (1, 6), (2, 2), (4, 2), (4, 3), (4, 4)]]
#robotpoints = [(-1,-1),(4,4)]

polypoints = [[(1,2),(1,4),(3,4),(3,2)],[(8,1),(4,1),(4,4),(5,2)]]
robotpoints = [(0,1),(2,0),(3,5),(6,2),(9,0)]


#drawLines(linepoints)
drawPolygons(polypoints)
drawRobots(robotpoints)

plt.axis('scaled')
plt.show()




