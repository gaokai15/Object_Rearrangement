import numpy as np
from itertools import islice, cycle
from math import *


def norm(u):
    return sqrt(sum(np.power(u, 2)))


def dist(a, b):
    return norm(np.subtract(a, b))


def dist_to_segment(p, x, y):
    u = np.subtract(y, x)
    v = np.subtract(p, x)

    vtu = np.dot(v, u) / norm(u)
    if 0 < vtu < norm(u):
        vpu = (vtu / norm(u)) * u
        return dist(vpu, v), tuple(vpu + x)
    else:
        dpxpy = [dist(x, p), dist(y, p)]
        return min(dpxpy), int(np.argmin(dpxpy))


# def shoelace(polygon):
#     darea = (polygon[-1][0] * polygon[0][1]) - (polygon[0][0] * polygon[-1][1])
#     for i in range(len(polygon) - 1):
#         darea += (polygon[i][0] * polygon[i + 1][1]) - (polygon[i + 1][0] * polygon[i][1])
#     return abs(darea)


def bbcross(a, b):
    a0x = min(a[0][0], a[1][0])
    a0y = min(a[0][1], a[1][1])
    a1x = max(a[0][0], a[1][0])
    a1y = max(a[0][1], a[1][1])
    b0x = min(b[0][0], b[1][0])
    b0y = min(b[0][1], b[1][1])
    b1x = max(b[0][0], b[1][0])
    b1y = max(b[0][1], b[1][1])
    return a0x <= b1x and a1x >= b0x and a0y <= b1y and a1y >= b0y


def slen(s):
    return dist(*s)


def segcross(x, y):
    b, a = sorted([x, y], key=slen)
    if not bbcross(a, b):
        return False

    u = np.subtract(a[1], a[0])
    b0 = np.subtract(b[0], a[0])
    b1 = np.subtract(b[1], a[0])
    uxb0 = np.sign(np.cross(u, b0))
    uxb1 = np.sign(np.cross(u, b1))

    return uxb0 != uxb1 or uxb0 == 0 or uxb1 == 0
    # return uxb0 != uxb1 and not ((uxb0 == 0) ^ (uxb1 == 0))


# def anyInterior(points, polygon):
#     darea = shoelace(polygon)
#     for point in points:
#         isInterior = True
#         for i in range(len(polygon)):
#             newPoly = list(polygon)
#             newPoly.insert(i, point)
#             if shoelace(newPoly) > darea:
#                 isInterior = False
#                 break
#         if isInterior:
#             return True
#     return False


def polysCollide(poly1, poly2):
    # if anyInterior(poly1, poly2):
    #     return True
    # if anyInterior(poly2, poly1):
    #     return True
    cpoly1 = list(islice(cycle(poly1), len(poly1) + 1))
    cpoly2 = list(islice(cycle(poly2), len(poly2) + 1))
    for i in range(len(poly1)):
        for j in range(len(poly2)):
            if segcross(cpoly1[i:i + 2], cpoly2[j:j + 2]):
                return True
    return False


# def symPolyConvexHull(poly, p1, p2):
#     marea = 0
#     mpoly = []
#     for i in range(len(poly) - 1):
#         for j in range(i + 1, len(poly)):
#             beg = np.add(poly[:i + 1], p1)  # .tolist() if len(poly[:i]) > 0 else []
#             mid = np.add(poly[i:j + 1], p2)  # .tolist() if len(poly[i:j]) > 0 else []
#             end = np.add(poly[j:], p1)  # .tolist() if len(poly[j:]) > 0 else []
#             testPoly = np.concatenate([beg, mid, end])
#             testArea = shoelace(testPoly)
#             # print(testArea, testPoly)
#             if marea < testArea:
#                 marea = testArea
#                 mpoly = testPoly
#     return mpoly
