import timeout_decorator
import os
# if __name__ == '__main__' and __package__ is None:
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from BiRRTPlanner import BiRRTPlanner
from BiRRTStarPlanner import BiRRTStarPlanner
from BiDirDPPlanner import BiDirDPPlanner, BiDirDPPlanner_A_star_furthest, BiDirDPPlanner_A_star_nearest, BiDirDPPlanner_suboptimal_furthest, BiDirDPPlanner_Random_Range, BiDirDPPlanner_Random_Nearest, BiDirDPPlanner_Leaf_Root, BiDirDPPlanner_Leaf_Root_Improved_Mutation, BiDirDPPlanner_Leaf_Large_Range, BiDirDPPlanner_Leaf_Small_Range, BiDirDPPlanner_Leaf_Nearest
from DPBruteForce import Non_Monotone_Solver_General, Generalized_Brute_force
from DensePathGenerator import DensePathGenerator

TimeLimit = 1000

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Leaf_Nearest(*args, **kwargs):
    return BiDirDPPlanner_Leaf_Nearest(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Leaf_Large_Range(*args, **kwargs):
    return BiDirDPPlanner_Leaf_Large_Range(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Leaf_Small_Range(*args, **kwargs):
    return BiDirDPPlanner_Leaf_Small_Range(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Leaf_Root_Improved_Mutation(*args, **kwargs):
    return BiDirDPPlanner_Leaf_Root_Improved_Mutation(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Leaf_Root(*args, **kwargs):
    return BiDirDPPlanner_Leaf_Root(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Random_Nearest(*args, **kwargs):
    return BiDirDPPlanner_Random_Nearest(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_Random_Range(*args, **kwargs):
    return BiDirDPPlanner_Random_Range(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_suboptimal_furthest(*args, **kwargs):
    return BiDirDPPlanner_suboptimal_furthest(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_A_star_nearest(*args, **kwargs):
    return BiDirDPPlanner_A_star_nearest(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_BiDirDPPlanner_A_star_furthest(*args, **kwargs):
    return BiDirDPPlanner_A_star_furthest(*args, **kwargs)

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
def timeout_Generalized_Brute_force(*args, **kwargs):
    return Generalized_Brute_force(*args, **kwargs)

@timeout_decorator.timeout(TimeLimit)
def timeout_DensePathGenerator(*args, **kwargs):
    return DensePathGenerator(*args, **kwargs)

