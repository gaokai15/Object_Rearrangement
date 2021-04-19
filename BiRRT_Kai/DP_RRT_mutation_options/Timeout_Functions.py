import timeout_decorator
import os
# if __name__ == '__main__' and __package__ is None:
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from BiDirDPPlanner1 import  BiDirDPPlanner_Leaf_Root
from DPBruteForce import Non_Monotone_Solver_General, Generalized_Brute_force
from DensePathGenerator import DensePathGenerator
from FastHeuristicDPPlanner import FastHeuristicDPPlanner
from BiDir_fmRS_Planner import BiDir_fmRS_Planner
from BiDir_mRS_Planner import BiDir_mRS_Planner

TimeLimit = 500000
@timeout_decorator.timeout(TimeLimit)
def timeout_BiDir_mRS_Planner(*args, **kwargs):
    return BiDir_mRS_Planner(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDir_fmRS_Planner(*args, **kwargs):
    return BiDir_fmRS_Planner(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_Fast_heuristic(*args, **kwargs):
    return FastHeuristicDPPlanner(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Leaf_Root(*args, **kwargs):
    return BiDirDPPlanner_Leaf_Root(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_Non_Monotone_Solver_General(*args, **kwargs):
    return Non_Monotone_Solver_General(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_Generalized_Brute_force(*args, **kwargs):
    return Generalized_Brute_force(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_DensePathGenerator(*args, **kwargs):
    return DensePathGenerator(*args, **kwargs)

