import timeout_decorator
import os
# if __name__ == '__main__' and __package__ is None:
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from DG_Space import Experiments


@timeout_decorator.timeout(3600)

def timeout_single_instance(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index):
    EXP = Experiments()
    return EXP.single_instance(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index)