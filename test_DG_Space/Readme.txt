This file written on March 28th is an explanation of functions in the class Experiments. You can switch among experiment functions by modifying line 897 in DP_Space.py.

single_instance: In this function, we generate a instance based on the parameters. And print out the DGs and the optimal solution to this instance.
    It is likely to have a failure on generating an instance, then we need to run it again.
    usage: python DG_Space.py 5 80 1000 1000 True n n

multi_instances: In this function, I collect data from 20 successfully generated instances. I set a timeout of 2 secs for genCGraph, so if it fail to generate an instance, it will print the failure information and try to generate again.
    Due to the timeout setting, I don't recommend you to show instance figures in this case.
    usage: python DG_Space.py 5 80 1000 1000 n n n

instance_generation_difficulty: In this function, I count the number of successful instance generation in 10 trials.
    Also due to the timeout setting, I don't recommend you to show instance figures in this case.
    usage: python DG_Space.py 5 80 1000 1000 n n n