import pyvisgraph as vg
polys = [[vg.Point(1,2),vg.Point(1,4),vg.Point(3,4),vg.Point(3,2)],[vg.Point(8,1),vg.Point(4,1),vg.Point(4,4),vg.Point(5,2)]]
g = vg.VisGraph()
g.build(polys)
shortest = g.shortest_path(vg.Point(0,1), vg.Point(2,0))
shortest += g.shortest_path(vg.Point(2,0), vg.Point(3,5))
shortest += g.shortest_path(vg.Point(3,5), vg.Point(6,2))
shortest += g.shortest_path(vg.Point(6,2), vg.Point(9,0))
print shortest