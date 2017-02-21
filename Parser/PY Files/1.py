import pyvisgraph as vg
polys = [[vg.Point(1,6),vg.Point(1,1),vg.Point(5,1),vg.Point(5,5),vg.Point(3,5),vg.Point(3,3),vg.Point(4,3),vg.Point(4,2),vg.Point(2,2),vg.Point(2,6),vg.Point(6,6),vg.Point(6,0),vg.Point(0,0),vg.Point(0,6)]]
g = vg.VisGraph()
g.build(polys)
shortest = g.shortest_path(vg.Point(-1,-1), vg.Point(4,4))
print shortest