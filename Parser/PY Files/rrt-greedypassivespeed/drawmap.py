#!/usr/bin/python
# coding: UTF-8
import matplotlib.pyplot as plt
from ast import literal_eval
import numpy

#  Change this 
number = 26  

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
    line = plt.Polygon(points, closed=None, fill=None)
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

fileName = str(number)+'.txt'  #change here per question
answerfileName = 'smo2sol-'+ str(number) + '.txt'
checkState = 0
robots = []
polygons = []
paths = []
samPaths = []
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
drawPolygonsNoFill(polygonpoints)

with open(answerfileName, 'r') as input:
    for line in input:
        line = line.strip('\n')
        paths = line.split(";")
        for path in paths:
            path = list(literal_eval(path))
            samPaths.append(path)

print samPaths
samLines(samPaths)
plt.axis('scaled')
plt.show()
