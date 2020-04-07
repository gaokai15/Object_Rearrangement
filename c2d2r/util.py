import numpy as np
import Polygon as pn
import Polygon.Utils as pu
from math import *


def norm(u):
    return sqrt(sum(np.power(u, 2)))


def dist(a, b):
    return norm(np.subtract(a, b))


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


def numBits(v):
    c = ((v & 0xfff) * 0x1001001001001 & 0x84210842108421) % 0x1f
    c += (((v & 0xfff000) >> 12) * 0x1001001001001 & 0x84210842108421) % 0x1f
    return c


MultiplyDeBruijnBitPosition2 = [
    0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8, 31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
]


def pow2log2(b32):
    return MultiplyDeBruijnBitPosition2[((b32 * 0x077CB531) & 0xffffffff) >> 27]
