# import Polygon as pn
# import Polygon.Utils as pu
import pyclipper as pc
from math import *


def polysCollide(poly1, poly2):
    # cpoly1 = pn.Polygon(poly1)
    # cpoly2 = pn.Polygon(poly2)
    # return cpoly1.overlaps(cpoly2)
    collide = True
    for cont in pc.MinkowskiDiff(poly1, poly2):
        collide &= pc.Orientation(collide) == pc.PointInPolygon((0, 0), cont)
    return collide
