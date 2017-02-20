import pyvisgraph as vg

def graph(polygons,robots):
    polys = [];
    temp = [];
    shortest = [];
    z = len(robots) - 1
    for x in polygons:
        for a,b in x:
            # print a, "then", b
            temp.append(vg.Point(a,b));
        polys.append(temp);
        temp = [];
    # print polys
    g = vg.VisGraph()
    g.build(polys)
    shortest = g.shortest_path((vg.Point(robots[0][0],robots[0][1])), (vg.Point(robots[1][0],robots[1][1])))
    for i in xrange(1, len(robots) - 1, 1):
            # print i
            # shortest.pop()
            shortest += g.shortest_path((vg.Point(robots[i][0],robots[i][1])), (vg.Point(robots[i+1][0],robots[i+1][1])) )
    print shortest

# graph([[(1,2),(1,4),(3,4),(3,2)],[(8,1),(4,1),(4,4),(5,2)]],[(0,1),(2,0),(3,5),(6,2),(9,0)])
graph([[(0,1), (2,3), (4,1), (4,10), (0,10)],[(4,0), (2,2), (0,0), (0,-10), (4,-10)]],[(-1.5, 1.5), (-1,0), (5,0), (4.5, 3.5), (4.6, -3)])
