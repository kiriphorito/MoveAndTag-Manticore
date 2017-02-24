#!/usr/bin/python
# coding: UTF-8
import matplotlib.pyplot as plt
from ast import literal_eval
import numpy
import sys
import ast


def drawPolygon(points):
    polygon = plt.Polygon(points,color='black')
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

#def drawLines(lines):
#    for xs in lines:
#        drawLine(xs)

def drawRobots(robots):
    for (x,y) in robots:
        plt.plot(x,y,".")

def drawPolygonNoFill(points,color):
    polygon = plt.Polygon(points,color=color,fill=False)
    plt.gca().add_patch(polygon)

def drawPolygonsNoFill(polygons):
    try:
        for xs in polygons:
            drawPolygonNoFill(xs,'black')
    except ValueError:
        print ("no polygons specified")


def samLines(coords):
    for lines in coords:
        print lines
        prex = "f"
        prey = "f"
        randColor = numpy.random.rand(3,1)
        for (x,y) in lines:
            print (x,y)
            if prex == "f":
                prex = x
                prey = y
            else:
                line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None,edgecolor=randColor)
                plt.gca().add_patch(line)
                prex = x
                prey = y
                print("connecting",(x,y),"to",(prex,prey))
#plt.plot((0,1),(1,2))


plt.axes()
if 'all' in sys.argv[2]:
    solutionFileName = 'smo2sol-' + sys.argv[1] + '.txt'
else:
    solutionFileName = 'smo2sol-' + sys.argv[1] + '-path-' + sys.argv[2] + '.txt'
with open(solutionFileName,'r') as input:
    for line in input:
        if 'Time Taken:' in line:
            continue
        solution = line

solution = solution.replace(";","],[")
solution = "[[" + solution + "]]"
print solution

solution = ast.literal_eval(solution)

samLines(solution)

fileName = sys.argv[1] + '.txt' #change here per question
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
print robotpoints
drawRobots(robotpoints)

polygonpoints = list(literal_eval(polygons[0]))
print polygonpoints

if 'nofill' == sys.argv[3]:
    drawPolygonsNoFill(polygonpoints)
else:
    drawPolygons(polygonpoints)

plt.axis('scaled')
plt.show()
