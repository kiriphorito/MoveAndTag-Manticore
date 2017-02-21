#!/usr/bin/python
# coding: UTF-8
import pyvisgraph as vg
import matplotlib.pyplot as plt
from ast import literal_eval
from ast import literal_eval as make_tuple


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
        plt.plot(x,y,".")


plt.axes()


fileName = '3.txt' #change here per question
checkState = 0
robots = []
polygons = []
with open(fileName,'r') as input: #reading txt file and drawing map
    for line in input:
        if 'Robots' in line:
            checkState = 1
            continue
        if 'Polygons' in line:
            checkState = 2
            continue
        if checkState == 1:
            line = line.strip('\n')
            lenstr = len(line)
            if lenstr != 0:
                robots.append(line)
        if checkState == 2:
            line = line.strip('\n')
            lenstr = len(line)
            if lenstr != 0:
                polygons.append(line)

robotpoints = list(literal_eval(robots[0]))
drawRobots(robotpoints)

polygonpoints = list(literal_eval(polygons[0]))
drawPolygons(polygonpoints)



polys = []
for arraypoly in polygonpoints: #adding poygons to the vg
    temp = []
    for (x,y) in arraypoly:
        temp.append(vg.Point(x,y))
    polys.append(temp)

g = vg.VisGraph()
g.build(polys)


paths = []

for (x,y) in robotpoints: #finding shortest path between any two vertices
    for (a,b) in robotpoints:
        shortest = g.shortest_path(vg.Point(x,y), vg.Point(a,b))
        paths.append(shortest)


print paths

for path in paths: #drawing the visibility graph
    print path
    temppath = []
    for coord in path:
        coord.__str__
        temppath.append(make_tuple(coord.__str__()))
    drawLine(temppath)






plt.axis('scaled')
plt.show()



