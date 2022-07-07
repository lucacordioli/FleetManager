"""!@package mainPackage
@file mainPackage/utilities.py
@brief This file contains some minors functions used in tha package.
"""
import math


def insert_point(event, newPoint):
    """!@brief Callback called when user clicks on the map. Update the newPoint dictionary.
    @param event: event object
    @param newPoint: dictionary containing the new point
    """
    x = event.xdata
    y = event.ydata
    newPoint['x'] = x
    newPoint['y'] = y


def translate_point(x, y, rows, cols):
    """!brief Translate a point from the map to the Gazebo simulation.
    @param x: x coordinate of the point of the map
    @param y: y coordinate of the point of the map
    @param rows: number of rows of the map
    @param cols: number of columns of the map
    @return: x, y coordinates of the point in the Gazebo simulation
    """
    realWidth = 48.4989
    realHeight = 209.316
    x = x * realWidth / cols
    y = realHeight - (y * realHeight / rows)
    return x, y


def distance(p1, p2):
    """!@brief Calculate the distance between two points.
    @param p1: first point
    @param p2: second point
    @return: euclidean distance between p1 and p2
    """
    return math.sqrt((p1['x'] - p2['x']) ** 2 + (p1['y'] - p2['y']) ** 2)


def find_nearest_point(roads, point):
    """!@brief Find the two nearest points in graph belonging to the same road.
    @param roads: list of roads
    @param point: point to find the nearest points
    @return theta: angle of the nearests points
    @return startPoint: nearest starting point in the road
    @return endPoint: nearest arrival point in the road
    """
    minDist = float('inf')
    nearestPoint = None
    nearestPointIndex = 0
    direction = None
    verse = None
    road = None
    for r in roads:
        for p in roads[r]['rightRoad']:
            dist = distance(roads[r]['rightRoad'][p], point)
            if dist < minDist:
                minDist = dist
                nearestPoint = roads[r]['rightRoad'][p]
                nearestPointIndex = p
                road = r

        # horizontal or vertical road
        if roads[road]['rightRoad'][0]['x'] == roads[road]['rightRoad'][1]['x']:
            direction = 'vertical'
        else:
            direction = 'horizontal'

    if direction == 'horizontal':
        if roads[road]['rightRoad'][0]['y'] < roads[road]['leftRoad'][0]['y']:
            verse = 'right'
            if nearestPoint['x'] > point['x']:
                endPoint = roads[road]['rightRoad'][nearestPointIndex]
                startPoint = roads[road]['rightRoad'][nearestPointIndex - 1]
            else:
                endPoint = roads[road]['rightRoad'][nearestPointIndex + 1]
                startPoint = roads[road]['rightRoad'][nearestPointIndex]
        else:
            verse = 'left'
            if nearestPoint['x'] > point['x']:
                endPoint = roads[road]['rightRoad'][nearestPointIndex + 1]
                startPoint = roads[road]['rightRoad'][nearestPointIndex]
            else:
                endPoint = roads[road]['rightRoad'][nearestPointIndex]
                startPoint = roads[road]['rightRoad'][nearestPointIndex - 1]
    else:  # vertical road
        if roads[road]['rightRoad'][0]['x'] > roads[road]['leftRoad'][0]['x']:
            verse = 'up'
            if nearestPoint['y'] < point['y']:
                endPoint = roads[road]['rightRoad'][nearestPointIndex + 1]
                startPoint = roads[road]['rightRoad'][nearestPointIndex]
            else:
                endPoint = roads[road]['rightRoad'][nearestPointIndex]
                startPoint = roads[road]['rightRoad'][nearestPointIndex - 1]
        else:
            verse = 'down'
            if nearestPoint['y'] < point['y']:
                endPoint = roads[road]['rightRoad'][nearestPointIndex]
                startPoint = roads[road]['rightRoad'][nearestPointIndex - 1]
            else:
                endPoint = roads[road]['rightRoad'][nearestPointIndex + 1]
                startPoint = roads[road]['rightRoad'][nearestPointIndex]

    theta = nearestPoint['th']

    return theta, startPoint, endPoint


def find_in_graph(point, graph):
    """!@brief Find the index of a point in a list of points.
    @param point: point to find the index
    @param graph: list of points
    """
    for i in graph:
        if graph[i]['x'] == point['x'] and graph[i]['y'] == point['y']:
            return i
    return -1
