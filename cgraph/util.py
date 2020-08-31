# import Polygon as pn
# import Polygon.Utils as pu
import colorsys
from math import *
import pyclipper as pc


def getColors(N):
    HSV_tuples = [(x * 1.0 / N, 0.5, 0.5) for x in range(N)]
    RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
    return RGB_tuples


def getColor(x):
    return colorsys.hsv_to_rgb(x, 0.8, 0.8)


def polyINTER(poly1, poly2, tree=False):
    if not (poly1 and poly2):
        return []

    clip = pc.Pyclipper()
    if type(poly1[0][0]) == list:
        clip.AddPaths(poly1, pc.PT_SUBJECT, True)
    else:
        clip.AddPath(poly1, pc.PT_SUBJECT, True)
    if type(poly2[0][0]) == list:
        clip.AddPaths(poly2, pc.PT_CLIP, True)
    else:
        clip.AddPath(poly2, pc.PT_CLIP, True)

    if tree:
        return clip.Execute2(pc.CT_INTERSECTION, pc.PFT_NONZERO, pc.PFT_NONZERO)
    return clip.Execute(pc.CT_INTERSECTION, pc.PFT_NONZERO, pc.PFT_NONZERO)


def polyUNION(poly1, poly2, tree=False):
    if not poly1:
        return poly2
    if not poly2:
        return poly1

    clip = pc.Pyclipper()
    if type(poly1[0][0]) == list:
        clip.AddPaths(poly1, pc.PT_SUBJECT, True)
    else:
        clip.AddPath(poly1, pc.PT_SUBJECT, True)
    if type(poly2[0][0]) == list:
        clip.AddPaths(poly2, pc.PT_CLIP, True)
    else:
        clip.AddPath(poly2, pc.PT_CLIP, True)

    if tree:
        return clip.Execute2(pc.CT_UNION, pc.PFT_NONZERO, pc.PFT_NONZERO)
    return clip.Execute(pc.CT_UNION, pc.PFT_NONZERO, pc.PFT_NONZERO)


def polyDIFF(poly1, poly2, tree=False):
    if not poly1:
        return list(reversed(poly2))
    if not poly2:
        return poly1

    clip = pc.Pyclipper()
    if type(poly1[0][0]) == list:
        clip.AddPaths(poly1, pc.PT_SUBJECT, True)
    else:
        clip.AddPath(poly1, pc.PT_SUBJECT, True)
    if type(poly2[0][0]) == list:
        clip.AddPaths(poly2, pc.PT_CLIP, True)
    else:
        clip.AddPath(poly2, pc.PT_CLIP, True)

    if tree:
        return clip.Execute2(pc.CT_DIFFERENCE, pc.PFT_NONZERO, pc.PFT_NONZERO)
    return clip.Execute(pc.CT_DIFFERENCE, pc.PFT_NONZERO, pc.PFT_NONZERO)


def polyXOR(poly1, poly2, tree=False):
    if not poly1:
        return poly2
    if not poly2:
        return poly1

    clip = pc.Pyclipper()
    if type(poly1[0][0]) == list:
        clip.AddPaths(poly1, pc.PT_SUBJECT, True)
    else:
        clip.AddPath(poly1, pc.PT_SUBJECT, True)
    if type(poly2[0][0]) == list:
        clip.AddPaths(poly2, pc.PT_CLIP, True)
    else:
        clip.AddPath(poly2, pc.PT_CLIP, True)

    if tree:
        return clip.Execute2(pc.CT_XOR, pc.PFT_NONZERO, pc.PFT_NONZERO)
    return clip.Execute(pc.CT_XOR, pc.PFT_NONZERO, pc.PFT_NONZERO)


def polyTOUCH(poly1, poly2):
    c = 0
    for cont in pc.MinkowskiDiff(poly1, poly2):
        if pc.PointInPolygon((0, 0), cont):
            if pc.Orientation(cont):
                c += 1
            else:
                c -= 1
    return c > 0


def o55():
    return 0.55


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
