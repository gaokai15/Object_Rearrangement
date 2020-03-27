import timeout_decorator
import os
# if __name__ == '__main__' and __package__ is None:
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from cgraph.cgraph import genCGraph


@timeout_decorator.timeout(2)

def timeout_genCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile):
    graph, path = genCGraph( numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile)
    return graph, path