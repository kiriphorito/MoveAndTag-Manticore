import matplotlib.pyplot as plt
from matplotlib import pyplot


plt.axes()
#arrayPoints = [[(0.0, 0.0), (10.0, 10.0), (9.0, 9.0)],[(10.0, 10.0), (8.0, 10.0), (5.0, 0.0)],[ (8.0, 10.0), (0.0, 7.0) ],[(9.0, 9.0), (1.0, 8.0)]]
#
#
#for lines in arrayPoints:
#    prex = "f"
#    prey = "f"
#    for (y,x) in lines:
#        if prex == "f":
#            prex = x
#            prey = y
#            continue
#        print (x,y)
#        print prex,  prey
#        plt.plot((prex,prey),(x,y))
#        prex = x
#        prey = y

coords = [[(0.0, 0.0), (10.0, 10.0), (9.0, 9.0)],[(10.0, 10.0), (8.0, 10.0), (5.0, 0.0)],[(8.0, 10.0), (0.0, 7.0)], [(9.0, 9.0), (1.0, 8.0)]]

#coords = [[(0.0, 1.0), (2.0, 0.0), (3.0, 2.0)],[(3.0, 4.0), (3.0, 5.0),(6.0, 2.0), (8.0, 1.0), (9.0, 0.0)]]

for lines in coords:
    print lines
    prex = "f"
    prey = "f"
    for (x,y) in lines:
        print (x,y)
        if prex == "f":
            prex = x
            prey = y
            continue
        line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None)
        plt.gca().add_patch(line)
#plt.plot((0,1),(1,2))





plt.axis('scaled')
plt.show()
