#!/usr/bin/python
# coding: UTF-8
import matplotlib.pyplot as plt
from ast import literal_eval
import sys
import ast

maxpath = sys.argv[2]
maxpath = int(maxpath)

path = ""
print (maxpath)
for i in range(1,maxpath+1):
    solutionFileName = 'smo2sol-' + sys.argv[1] + '-path-' + str(i) + '.txt'
    print solutionFileName
    with open(solutionFileName,'r') as input:
        for line in input:
            if 'Time Taken:' in line:
                continue
            solution = line
    
        path+= str(solution)
        print path



solname = 'smo2sol-' + sys.argv[1] + '.txt'
f= open(solname,'w')
f.write(str(path))
f.close

