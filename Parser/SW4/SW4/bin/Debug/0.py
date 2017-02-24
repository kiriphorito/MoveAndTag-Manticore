import pyvisgraph as vg
polys = []
g = vg.VisGraph()
g.build(polys)
shortest = g.shortest_path(vg.Point(0,0), vg.Point(10,10))
shortest += g.shortest_path(vg.Point(10,10), vg.Point(8,10))
shortest += g.shortest_path(vg.Point(8,10), vg.Point(9,9))
shortest += g.shortest_path(vg.Point(9,9), vg.Point(5,0))
shortest += g.shortest_path(vg.Point(5,0), vg.Point(0,7))
shortest += g.shortest_path(vg.Point(0,7), vg.Point(1,8))
print shortest