from LatLon import *
from sys import maxsize
from math import sqrt
from numpy import ones, vstack
from numpy.linalg import lstsq
# Library for latitude/longitude calculations

def get_closest(Node_set, curr_latlon):
    min_dist = maxsize
    min_node = None

    for node in Node_set:
        distance_to_node = curr_latlon.distance(node.latlon)
        if distance_to_node < min_dist:
            min_dist = distance_to_node
            min_node = node

    return min_node

def get_closest_road(endpoint_node, curr_latitude, curr_longitude):
    standard_coord = LatLon(39.947604, -75.190101)
    currX = LatLon(39.947604, curr_longitude)
    currY = LatLon(curr_latitude, -75.190101)
    x_curr = standard_coord.distance(currX)
    y_curr = standard_coord.distance(currY)

    mindist = maxsize
    min_road = None
    for road in endpoint_node.neighborEdges:
        distance_to_road = getDistanceToLine(road, x_curr, y_curr)

        if distance_to_road < mindist:
            mindist = distance_to_road
            min_road = road
    return min_road

def getDistanceToLine(road, x_curr, y_curr):
    return abs(-1 * road.m * x_curr + y_curr - road.b)/sqrt(road.m**2 + 1)


class Node:

    def __init__(self, name, latitude, longitude):
        self.name = name
        self.neighborEdges = []
        self.latitude = latitude
        self.longitude = longitude
        self.latlon = LatLon(latitude, longitude)

    def add_neighbor(self, node_end, speed_limit):
        self.neighborEdges.append(self.Edge(self, node_end, speed_limit))


    class Edge:

        def __init__(self, node_start, node_end, speed_limit):
            self.nodeStart = node_start
            self.nodeEnd = node_end
            self.heading = self.calculate_heading(node_start, node_end)
            self.distance = self.calculate_distance(node_start, node_end)
            self.speedLimit = speed_limit
            self.m, self.b = self.set_equation_of_line(node_start, node_end)

        def calculate_heading(self, node_start, node_end):
            start_latlon = node_start.latlon
            end_latlon = node_end.latlon
            return start_latlon.heading_initial(end_latlon)

        def calculate_distance(self, node_start, node_end):
            start_latlon = node_start.latlon
            end_latlon = node_end.latlon
            return start_latlon.distance(end_latlon) * 1000

        def set_equation_of_line(self, node_start, node_end):
            # # setup your projections
            # crs_wgs = proj.Proj(init='epsg:4326')  # assuming you're using WGS84 geographic
            # crs_bng = proj.Proj(init='epsg:26986')  # use a locally appropriate projected CRS
            #
            # # then cast your geographic coordinate pair to the projected system
            # x1, y1 = proj.transform(crs_wgs, crs_bng, node_start.longitude, node_start.latitude)
            # x2, y2 = proj.transform(crs_wgs, crs_bng, node_end.longitude, node_end.latitude)
            #39.947604
            #-75.190101
            standard_coord = LatLon(39.947604, -75.190101)
            point1X = LatLon(39.947604, node_start.longitude)
            point1Y = LatLon(node_start.latitude, -75.190101)
            x1 = standard_coord.distance(point1X)
            y1 = standard_coord.distance(point1Y)


            point2X = LatLon(39.947604, node_end.longitude)
            point2Y = LatLon(node_end.latitude, -75.190101)
            x2 = standard_coord.distance(point2X)
            y2 = standard_coord.distance(point2Y)

            print("X Distance to node {} is {}".format(node_start.name, x1))
            print("Y Distance to node {} is {}".format(node_start.name, y1))

            print("X Distance to node {} is {}".format(node_end.name, x2))
            print("Y Distance to node {} is {}".format(node_end.name, y2))

            # if x2-x1 != 0:
            #     slope = (y2 - y1)/(x2 - x1)
            #     print("Slope is {}".format(slope))
            points = [(x1, y1), (x2, y2)]
            x_coords, y_coords = zip(*points)
            A = vstack([x_coords, ones(len(x_coords))]).T
            m, b = lstsq(A, y_coords)[0]
            print("Node Start {} Node End {}".format(node_start.name, node_end.name))
            print("Line Solution is y = {m}x + {b}".format(m=m, b=b))
            return m, b



if __name__ == '__main__':
    # node = Node("a", 39.950407, -75.192621)
    # nodeB = Node("b", 39.950481, -75.193479)
    # node.add_neighbor(nodeB, 15)
    # print(node.neighborEdges[0].distance)
    # print(node.neighborEdges[0].heading)

    nodesText = open('mapPennParkNodes.txt', 'r')

    Nodes = {}

    for line in nodesText.readlines():
        line_data = line.split()
        node_name = line_data[0]
        lat = line_data[1]
        lon = line_data[2]
        Nodes[node_name] = Node(node_name, lat, lon)

    nodesText.close()

    roads = open('mapPennParkEdges.txt', 'r')

    for line in roads.readlines():
        line_data = line.split()
        node_start = line_data[0]
        node_end = line_data[1]
        speed_forward = line_data[2]
        speed_backward = line_data[3]
        Nodes[node_start].add_neighbor(Nodes[node_end], speed_forward)
        Nodes[node_end].add_neighbor(Nodes[node_start], speed_backward)

    roads.close()

    my_latlon = LatLon(39.950096, -75.185831)
    closest_node = get_closest(Nodes.itervalues(), my_latlon)
    print(closest_node.name)

    closest_road = get_closest_road(closest_node, 39.950096, -75.185831)
    print("Road start {} Road End {}".format(closest_road.nodeStart.name, closest_road.nodeEnd.name))












