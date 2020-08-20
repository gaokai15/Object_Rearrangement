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


def circle_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0**2 - r1**2 + d**2) / (2 * d)
        h = math.sqrt(r0**2 - a**2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return ((x3, y3), (x4, y4))
