#!/usr/bin/python
# coding: UTF-8
import matplotlib.pyplot as plt
from ast import literal_eval


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


fileName = '29.txt'
checkState = 0
robots = []
polygons = []
with open(fileName,'r') as input:
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
print robotpoints
drawRobots(robotpoints)

polygonpoints = list(literal_eval(polygons[0]))
print polygonpoints
drawPolygons(polygonpoints)

plt.axis('scaled')
plt.show()
