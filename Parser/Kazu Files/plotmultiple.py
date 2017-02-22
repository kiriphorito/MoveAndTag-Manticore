import matplotlib.pyplot as plt
from matplotlib import pyplot


#plt.axes()
path = [[70.69506359346656, 78.41810448767427], [71.09286714681357, 27.46350310196671], [57.16494228203929, -33.85202051828178], [-2.8369022298963946, -103.90691703176745], [-56.5533440118182, -113.18872565547956], [-84.2787860814455, -111.42966972146854]]
#coords = [[(-1.0, -1.0), (0.0, 6.0), (1.0, 6.0), (2.0, 2.0), (4.0, 2.0), (4.0, 3.0), (4.0, 4.0)]]
#
##coords = [[(0.0, 1.0), (2.0, 0.0), (3.0, 2.0)],[(3.0, 4.0), (3.0, 5.0),(6.0, 2.0), (8.0, 1.0), (9.0, 0.0)]]
#
#for lines in coords:
#    print lines
#    prex = "f"
#    prey = "f"
#    for (x,y) in lines:
#        print (x,y)
#        if prex == "f":
#            prex = x
#            prey = y
#        else:
#            line = plt.Polygon([(prex,prey),(x,y)], closed=None, fill=None)
#            plt.gca().add_patch(line)
#            prex = x
#            prey = y
#            print("connecting",(x,y),"to",(prex,prey))
##plt.plot((0,1),(1,2))
#

pathLength = len(path)
print path
print path.reverse()




#plt.axis('scaled')
#plt.show()
