import Polygon as pn
import Polygon.Utils as pu
from math import *


def polysCollide(poly1, poly2):
    cpoly1 = pn.Polygon(poly1)
    cpoly2 = pn.Polygon(poly2)
    return cpoly1.overlaps(cpoly2)


def mergePolys(polyiter, removeHoles=False):
    pm = pn.Polygon()
    for poly in polyiter:
        pm = pm + pn.Polygon(poly)
    # pm.simplify()
    # pm = pu.prunePoints(pm)
    if removeHoles:
        pm = pu.fillHoles(pm)
    return pu.pointList(pm)


def poly_disk(center, radius, resolution=1):
    poly = []
    for i in range(360, 0, -resolution):
        rad = radians(i)
        x = center[0] + radius * cos(rad)
        y = center[1] + radius * sin(rad)
        poly.append([x, y])
    return poly
