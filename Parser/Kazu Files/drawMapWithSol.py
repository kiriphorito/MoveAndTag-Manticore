#!/usr/bin/python
# coding: UTF-8
import matplotlib.pyplot as plt
from ast import literal_eval
import sys
import ast

maxpath = sys.argv[2]

for i in range(1,maxpath):
    solutionFileName = 'smo2sol-' + sys.argv[1] + '-path-' + i + '.txt'
    with open(solutionFileName,'r') as input:
    for line in input:
        if 'Time Taken:' in line:
            continue
        solution = line

solutionFileName = 'smo2sol-' + sys.argv[1] + '.txt'
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
drawPolygons(polygonpoints)

plt.axis('scaled')
plt.show()
