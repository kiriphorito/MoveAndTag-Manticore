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

#def drawLines(lines):
#    for xs in lines:
#        drawLine(xs)

def drawRobots(robots):
    for (x,y) in robots:
        plt.plot(x,y,".")

def samLines(coords):
    for lines in coords:
        print lines
        prex = "f"
        prey = "f"
        for (x,y) in lines:
            print (x,y)
            if prex == "f":
                prex = x
                prey = y
            else:
                line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None)
                plt.gca().add_patch(line)
                prex = x
                prey = y
                print("connecting",(x,y),"to",(prex,prey))
#plt.plot((0,1),(1,2))


plt.axes()

samLines([[(-40.57514036406533, 3.8), (-46.19376547090254, 4.201137536168899), (-45.516422368573636, 11.005054497023648), (-50.30891598378499, 6.327204140107319), (-50.26979980363977, -42.381685465382525), (-7.954625549843027, -14.027870964596765), (-0.6766162121489226, -10.310312774137797), (2.1751389899810256, -5.730368125347936), (4.87230677871107, -5.382711652926183), (11.617667918691506, -15.707012495815102), (74.34600792513247, -34.433155569455884), (73.1228535035278, 9.778470585246874), (52.68939260423591, -4.498466820722119), (52.946466551349, -4.99644686618755)]])


fileName = '6.txt' #change here per question
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
drawPolygons(polygonpoints)

plt.axis('scaled')
plt.show()
