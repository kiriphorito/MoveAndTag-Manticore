import pyvisgraph as vg

def graph(polygons,robots):
    polys = [];
    temp = [];
    shortest = [];
    for x in polygons:
        for a,b in x:
            print a, "then", b
            temp.append(vg.Point(a,b));
        polys.append(temp);
    g = vg.VisGraph()
    g.build(polys)
    shortest = g.shortest_path((vg.Point(robots[0][0],robots[0][1])), (vg.Point(robots[1][0],robots[1][1])))
    for i in robots:
        if len(robots) > i:
            print i
            shortest += g.shortest_path(shortest, (vg.Point(robots[i][0],robots[i][1])))
    print shortest

graph([[(1,2),(1,4),(3,4),(3,2)]],[(0,1),(2,0),(3,4)])
