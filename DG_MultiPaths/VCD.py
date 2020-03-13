# import ast, math, operator
import matplotlib.pyplot as plt

# import sys
from utils.configuration_space import Roadmap
from collections import defaultdict
from utils.uniform_cost_search import Search
from utils.graph_utils import *


class VerticalCellDecomposition:
    def __init__(self, cspace):
        self.cspace = cspace

        self.polygon_vertices = [item for sublist in cspace.polygons for item in sublist]

        self.polygon_edges = []

        for polygon in cspace.polygons:
            for i in range(len(polygon)):
                self.polygon_edges.append(
                    [polygon[i % len(polygon)], polygon[(i + 1) % len(polygon)]]
                )

        self.decomposition_lines = []
        self.decomposition_lines_midpts = []
        self.decomposition_lines_map = defaultdict(list)
        self.regions = []

        self.roadmap = Roadmap()

    def construct_decomposition_lines(self, vertex, verticalLine, pointList, top):
        for edge in self.polygon_edges:
            if vertex not in edge:  # exclude edges containing the self vertex
                point = line_intersection(verticalLine, edge)
                if point is not None:  # If they intersect...
                    pointList.append(point)

        # Find nearest point of intersection
        nearest_point = pointList[0]
        for i in range(1, len(pointList)):
            if top and pointList[i][1] < nearest_point[1]:
                nearest_point = pointList[i]
            elif pointList[i][1] > nearest_point[1]:
                nearest_point = pointList[i]

        # Check if the nearest point falls on same polygon. If so, ignore the vertical line
        for polygon in self.cspace.polygons:
            if vertex in polygon:
                if not on_polygon(nearest_point[0], nearest_point[1], polygon):
                    self.decomposition_lines.append([vertex, nearest_point])

                    # create vertice_decompLines_map {vertex : pointList}
                    self.decomposition_lines_map[vertex].append([vertex, nearest_point])

    # Iterate over all edges for each vertex
    def vertical_lines(self):
        for vertex in self.polygon_vertices:
            # TopLine
            top = True
            pointList = [(vertex[0], self.cspace.boundary[2][1])]
            verticalLine = [vertex, (vertex[0], self.cspace.boundary[2][1])]
            self.construct_decomposition_lines(vertex, verticalLine, pointList, top)

            # BottomLine
            top = False
            pointList = [
                (vertex[0], self.cspace.boundary[0][1])
            ]  # considering the intersection point on border
            verticalLine = [vertex, (vertex[0], self.cspace.boundary[0][1])]
            self.construct_decomposition_lines(vertex, verticalLine, pointList, top)

        for line in self.decomposition_lines:
            self.decomposition_lines_midpts.append(
                [line[0][0], (line[0][1] + line[1][1]) / 2.0]
            )

    # up = True for up, False for down
    def find_region_line(self, line_type, i, vertex, current_vertex):
        maxY = self.cspace.boundary[2][1]
        minY = self.cspace.boundary[0][1]

        if line_type == "middle":
            for next_vertex in self.polygon_vertices[i:]:  # next_vertexertex
                for k in range(len(self.decomposition_lines_map[next_vertex])):
                    temp_vertex = self.decomposition_lines_map[next_vertex][k][1]
                    if (temp_vertex[1] not in (
                            maxY,
                            minY)):  # middle line, not intersecting with top/bottom border
                        if ((vertex[1] <= next_vertex[1] <= current_vertex[1])
                                or (vertex[1] <= temp_vertex[1] <= current_vertex[1])
                                or (next_vertex[1] <= vertex[1] <= temp_vertex[1])
                                or (next_vertex[1] <= current_vertex[1] <= temp_vertex[1])
                                or (vertex[1] >= next_vertex[1] >= current_vertex[1])
                                or (vertex[1] >= temp_vertex[1] >= current_vertex[1])
                                or (next_vertex[1] >= vertex[1] >= temp_vertex[1])
                                or (next_vertex[1] >= current_vertex[1] >= temp_vertex[1])):
                            return [next_vertex, temp_vertex]

        # if Up, check with 'up Yval', else down
        elif line_type == "up":
            yVal = maxY

        elif line_type == "down":
            yVal = minY

        if line_type == "up" or line_type == "down":
            for next_vertex in self.polygon_vertices[i:]:
                # iterate over its decomposition lines
                for j in range(len(self.decomposition_lines_map[next_vertex])):
                    current_vertex = self.decomposition_lines_map[next_vertex][j][1]
                    if (current_vertex[1] == yVal):
                        return [next_vertex, current_vertex]

        return []

    def region_disection(self):
        # LEN = 1
        # for k in range(LEN):
        maxY = self.cspace.boundary[2][1]
        minY = self.cspace.boundary[0][1]

        # diff = (maxY - minY) / LEN
        # minY += k * diff
        # maxY -= (LEN - 1 - k) * diff

        self.polygon_vertices = sorted(self.polygon_vertices, key=lambda x: x[0])

        firstCell = [
            self.cspace.boundary[0], self.cspace.boundary[3],
            self.decomposition_lines_map[self.polygon_vertices[0]][0][1],
            self.decomposition_lines_map[self.polygon_vertices[0]][1][1]
        ]
        lastCell = [
            self.cspace.boundary[2], self.cspace.boundary[1],
            self.decomposition_lines_map[self.polygon_vertices[-1]][0][1],
            self.decomposition_lines_map[self.polygon_vertices[-1]][1][1]
        ]

        self.regions.append(firstCell)
        self.regions.append(lastCell)

        # Iterate over vertices
        for i, vertex in enumerate(self.polygon_vertices[:-1]):
            # iterate over its decomposition lines
            for j in range(len(self.decomposition_lines_map[vertex])):
                # check if both vertices are having both up and down
                current_vertex = self.decomposition_lines_map[vertex][j][1]
                next_vertex = self.polygon_vertices[i + 1]

                if (len(self.decomposition_lines_map[vertex]) == 2) and (len(
                        self.decomposition_lines_map[next_vertex]) == 2):
                    ov = [
                        self.decomposition_lines_map[vertex][0][1],
                        self.decomposition_lines_map[vertex][1][1],
                        self.decomposition_lines_map[next_vertex][0][1],
                        self.decomposition_lines_map[next_vertex][1][1]
                    ]

                    if ((ov[0][1] in (maxY, minY)) and (ov[1][1] in (maxY, minY))
                            and (ov[2][1] in (maxY, minY)) and (ov[3][1] in (maxY, minY))):
                        self.regions.append(ov)
                        continue

                if (current_vertex[1] == maxY):  # upper line
                    self.regions.append(
                        self.decomposition_lines_map[vertex][j] +
                        self.find_region_line("up", i + 1, vertex, current_vertex)
                    )

                elif (current_vertex[1] == minY):  # lower line
                    self.regions.append(
                        self.decomposition_lines_map[vertex][j] +
                        self.find_region_line("down", i + 1, vertex, current_vertex)
                    )

                else:  # middle
                    lst = self.find_region_line("middle", i + 1, vertex, current_vertex)
                    if len(lst) > 0:
                        self.regions.append(self.decomposition_lines_map[vertex][j] + lst)
                        # TODO, do not remove the comment below
                        # print(self.decomposition_lines_map[vertex][j]+lst)

        self.roadmap.vertices_dict[0] = list(self.cspace.start_state)
        self.roadmap.vertices_dict[1] = list(self.cspace.goal_state)

        for i, region in enumerate(self.regions):
            c_x = 0
            c_y = 0

            for point in region:
                c_x += point[0] / float(len(region))
                c_y += point[1] / float(len(region))

            self.roadmap.vertices_dict[i + 2] = [c_x, c_y]

    def construct_graph(self, collides=None):
        self.vertical_lines()
        self.region_disection()

        max_key = list(self.roadmap.vertices_dict.keys())[-1]

        for i, point in enumerate([self.cspace.start_state, self.cspace.goal_state]):
            for j, centroid in enumerate(list(self.roadmap.vertices_dict.values())[2:]):
                skip = False
                graph_line = [centroid, point]
                # check intersection with any of (edges, decomposition lines)

                for line in self.polygon_edges + self.decomposition_lines:
                    if (line_intersection(graph_line, line) is not None):
                        skip = True
                        break

                if not skip and collides is not None:
                    skip = not collides(point) and not collides(centroid)

                if skip:
                    continue

                self.roadmap.adjacency_dict[i].append(j + 2)
                self.roadmap.adjacency_dict[j + 2].append(i)
                self.roadmap.edge_weights[i].append(distance(point, centroid))
                self.roadmap.edge_weights[j + 2].append(distance(point, centroid))
                break

        for i in range(len(self.decomposition_lines_midpts)):
            self.roadmap.vertices_dict[max_key + i + 1] = self.decomposition_lines_midpts[i]

        for i, centroid in enumerate(list(self.roadmap.vertices_dict.values())[2:max_key + 1]):
            for j, point in enumerate(list(self.roadmap.vertices_dict.values())[max_key + 1:]):
                skip = False
                graph_line = [centroid, point]

                for edge in self.polygon_edges:
                    if line_intersection(graph_line, edge) is not None:
                        skip = True
                        break

                for decomposition_line in self.decomposition_lines:
                    insersection_point = line_intersection(graph_line, decomposition_line)
                    if insersection_point is not None and distance(insersection_point,
                                                                   point) != 0:
                        skip = True
                        break

                if not skip and collides is not None:
                    skip = not collides(point) and not collides(centroid)

                if skip:
                    continue

                self.roadmap.adjacency_dict[i + 2].append(max_key + 1 + j)
                self.roadmap.edge_weights[i + 2].append(distance(point, centroid))
                self.roadmap.adjacency_dict[max_key + j + 1].append(i + 2)
                self.roadmap.edge_weights[max_key + j + 1].append(distance(point, centroid))

    def plot_vcd(self):
        self.cspace.plot_config_space(showPlot=False)

        for point in self.roadmap.vertices_dict.values():
            plt.plot(point[0], point[1], marker='o', color='black')

        for key in self.roadmap.adjacency_dict.keys():
            for value in self.roadmap.adjacency_dict[key]:
                plt.plot(
                    [self.roadmap.vertices_dict[key][0], self.roadmap.vertices_dict[value][0]],
                    [self.roadmap.vertices_dict[key][1], self.roadmap.vertices_dict[value][1]],
                    color='y'
                )

        # plot decomposition lines
        for line in self.decomposition_lines:
            x = [line[0][0], line[1][0]]
            y = [line[0][1], line[1][1]]
            plt.plot(x, y, 'b')

    def search(self, showPlot=False):
        ucs = Search(self.roadmap)
        searchResult = ucs.perform_search()

        if searchResult is None:
            # print("Path could not be found!")
            return None, []

        final_path, final_path_idx, path_cost = searchResult

        self.plot_vcd()

        for i in range(1, len(final_path)):
            plt.plot(
                [elem[0] for elem in final_path[i - 1:i + 1]],
                [elem[1] for elem in final_path[i - 1:i + 1]],
                color='brown'
            )

        if showPlot:
            plt.show()

        return final_path, final_path_idx
