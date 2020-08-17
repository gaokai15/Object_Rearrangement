from __future__ import division

import copy
from util import *
import Polygon as pn
from random import uniform, random


class InstanceGenerator(object):
    def __init__(self, numObjs, HEIGHT, WIDTH, polygon):
        if numObjs == 0:
            self.polygon = None
            self.points = None
            self.objects = None
            self.minkowski_objs = []
            self.buffer_points = None
            self.buffers = None
            self.minkowski_buffers = []
            return
        self.polygon = polygon
        self.points, self.objects, self.minkowski_objs = self.generateInstance(numObjs, HEIGHT, WIDTH)
        self.buffer_points = []  ### center of the buffers
        self.buffers = []  ### polygons of the buffers
        self.minkowski_buffers = []  ### minkowski sum of the buffers

        ### need to check if the instance has been successfully generated
        if (self.points == False): return

    def copy(self):
        new_instance = InstanceGenerator(0, 0, 0, 0)
        new_instance.polygon = copy.copy(self.polygon)
        new_instance.points = copy.copy(self.points)
        # print(new_instance.points)
        new_instance.objects = copy.copy(self.objects)
        new_instance.buffer_points = copy.copy(self.buffer_points)
        # print(new_instance.buffer_points)
        new_instance.buffers = copy.copy(self.buffers)

        for point in self.points:
            mink_obj = 2 * self.polygon + point  ### grown_shape object
            new_instance.minkowski_objs.append(pn.Polygon(mink_obj))

        for point in self.buffer_points:
            mink_obj = 2 * self.polygon + point  ### grown_shape object
            new_instance.minkowski_buffers.append(pn.Polygon(mink_obj))

        return new_instance

    def generateInstance(self, numObjs, HEIGHT, WIDTH):
        ### this function tries to generate a instance with the allowed amount of trials
        points, objects, minkowski_objs = self.genSingleInstance(numObjs, HEIGHT, WIDTH)
        timeout = 20  ### allow 20 trials to generate a single instance
        while (points == False) and (timeout > 0):
            timeout -= 1
            points, objects, minkowski_objs = self.genSingleInstance(numObjs, HEIGHT, WIDTH)
        ### reach here either (1) success generation (points != False) or (2) timeout
        return points, objects, minkowski_objs

    def genSingleInstance(self, numObjs, HEIGHT, WIDTH):
        ### this function makes a single attempt of generating instances
        points = []  ### center of the objects
        objects = []  ### polygons of the objects
        minkowski_objs = []  ### minkowski sum of the objects

        for i in range(numObjs):
            ### need to generate both start and goal
            for j in range(2):
                isfree = False
                timeout = 1000
                while not isfree and timeout > 0:
                    timeout -= 1
                    ### generate the center of an object with uniform distribution
                    point = (
                        uniform(0 - min(self.polygon[:, 0]), WIDTH - max(self.polygon[:, 0])),
                        uniform(0 - min(self.polygon[:, 1]), HEIGHT - max(self.polygon[:, 1]))
                    )
                    ### For dense case,
                    ### start only checks with starts
                    ### goal only checks with goals
                    isfree = isCollisionFree(self.polygon, point, objects[j % 2::2])

                if timeout <= 0:
                    # print "FAIL TO GENERATE THE INITIAL INSTANCE AT (" + str(i) + "," + str(j) + ")"
                    return False, False, False

                ### Congrats the object's goal/start is accepted
                points.append(point)
                objects.append(pn.Polygon(self.polygon + point))
                mink_obj = 2 * self.polygon + point  ### grown_shape object
                minkowski_objs.append(pn.Polygon(mink_obj))

        return points, objects, minkowski_objs

    def genBuffers(self, HEIGHT, WIDTH, numObjs):
        ### This function generate buffers for the instance already generated
        ### The idea is that we randomly generate a buffer in the space, hoping it will overlap with nothing
        ### if it could not achieve after several trials, we increment the number of object poses it can overlap
        ### we keep incrementing until we find enough buffers

        ### initialization
        self.buffer_points = []  ### center of the buffers
        self.buffers = []  ### polygons of the buffers
        self.minkowski_buffers = []  ### minkowski sum of the buffers

        numBuffers = 0  ### we can decide the number of buffers based on numObjs later
        maximumOverlap = numObjs

        for i in range(numBuffers):
            numOverlapAllowed = 0
            isValid = False

            while numOverlapAllowed <= maximumOverlap:
                timeout = 500
                while not isValid and timeout > 0:
                    timeout -= 1
                    ### generate the center of an object with uniform distribution
                    point = (
                        uniform(0 - min(self.polygon[:, 0]), WIDTH - max(self.polygon[:, 0])),
                        uniform(0 - min(self.polygon[:, 1]), HEIGHT - max(self.polygon[:, 1]))
                    )
                    numOverlap = countNumOverlap(self.polygon, point, self.objects, self.buffers, numOverlapAllowed)
                    if numOverlap <= numOverlapAllowed:
                        isValid = True
                ### reach here either (1) isValid == True (2) timeout <= 0
                if isValid == True:
                    ### Otherwise the buffer is accepted
                    self.buffer_points.append(point)
                    self.buffers.append(pn.Polygon(self.polygon + point))
                    mink_obj = 2 * self.polygon + point  ### grown_shape buffer
                    self.minkowski_buffers.append(pn.Polygon(mink_obj))
                    # print "successfully generating buffer " + str(i) + " overlapping with " + str(numOverlap) + " poses"
                    break
                else:
                    ### timeout <= 0
                    ### keep failing generating a buffer allowed to overlap with maximum number of objects
                    ### increase the numOverlapAllowed
                    numOverlapAllowed += 1

        ### reach here if all the buffers have been generated
        return


if __name__ == "__main__":
    print("welcome to Experiment! Please call it from Experiment.py\n")
