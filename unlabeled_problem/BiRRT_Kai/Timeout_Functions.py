import timeout_decorator
import os
# if __name__ == '__main__' and __package__ is None:
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from BiRRTPlanner import BiRRTPlanner
from BiRRTStarPlanner import BiRRTStarPlanner
from BiDirDPPlanner import BiDirDPPlanner
from DPBruteForce import Non_Monotone_Solver_General
from DensePathGenerator import DensePathGenerator

TimeLimit = 1000

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner(*args, **kwargs):
    return BiDirDPPlanner(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiRRTPlanner(*args, **kwargs):
    return BiRRTPlanner(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiRRTStarPlanner(*args, **kwargs):
    return BiRRTStarPlanner(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_Non_Monotone_Solver_General(*args, **kwargs):
    return Non_Monotone_Solver_General(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_DensePathGenerator(*args, **kwargs):
    return DensePathGenerator(*args, **kwargs)

