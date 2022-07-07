"""!@package graph_generator_package
@file graph_generator_node.py
@brief Generate a graph from map and user input
"""

import numpy as np
from matplotlib import pyplot as plt
from .mainPackage.fileMaganer import *
from .mainPackage.utilities import *
from .mainPackage.trackRoad import *


def initMap(r, newPoint):
    """! @brief Initialize and show the map.
    @param r Raster of map.
    @param newPoint The new point to change when the map is clicked.
    @return rows Number of rows in the map.
    @return cols Number of columns in the map.
    """
    rows = len(r)
    cols = len(r[0])
    print("rows: ", rows, ", cols: ", cols)

    # show image in window
    mapToShow = np.zeros((len(r), len(r[0]), 3), np.uint8)  # create a black image
    mapToShow.fill(255)  # fill the image with white
    for y in range(len(r) - 1):
        for x in range(len(r[y]) - 1):
            if r[y][x] == 0:
                mapToShow[y][x] = [0, 0, 0]
    plt.imshow(mapToShow)
    plt.connect('button_press_event', lambda event: insert_point(event, newPoint))
    plt.show(block=False)

    return rows, cols


def inputBases(idGraph, newPoint, rows, cols):
    """! @brief Ask user to input the robot's bases.
    @param idGraph Current id of the point of the graph.
    @param newPoint The new point to change when the map is clicked.
    @param rows Number of rows in the map.
    @param cols Number of columns in the map.
    @return bases List of bases.
    @return idGraph Current id of the point of the graph added with bases.
    """
    print("Let's talk about bot's bases!")
    bases = {}
    char = 'general'
    while char != 'n':
        char = input("Press Enter to save position of base's robot! 'n' to next: ")
        if char != 'n':
            x, y = translate_point(newPoint['x'], newPoint['y'], rows, cols)
            bases[idGraph] = {'x': x, 'y': y}
            plt.plot(newPoint['x'], newPoint['y'], 'bo')
            print(bases[idGraph])
            idGraph += 1
    return bases, idGraph


def inputOffices(idGraph, newPoint, rows, cols):
    """! @brief Input the offices.
    @param idGraph Current id of the point of the graph.
    @param newPoint The new point to change when the map is clicked.
    @param rows Number of rows in the map.
    @param cols Number of columns in the map.
    @return offices List of offices associated with name.
    @return idGraph Current id of the point of the graph added with offices.
    """
    offices = {}
    char = 'general'
    while char != 'n':
        char = input("Click on the map and insert office name: ")
        if char != 'n':
            x, y = translate_point(int(newPoint['x']), int(newPoint['y']), rows, cols)
            offices[idGraph] = {'name': char, 'x': x, 'y': y}
            plt.plot(newPoint['x'], newPoint['y'], 'ro')
            print(offices[idGraph])
            idGraph += 1
    return offices, idGraph


def inputCorridors(newPoint, r):
    """! @brief Input some points in the corridors and create complete roads.
    @param newPoint The new point to change when the map is clicked.
    @param r Raster of map.
    @return corridors List of roads.
    """
    roads = {}
    nRoads = 0
    startPoint = {'x': 0, 'y': 0}
    endPoint = {'x': 0, 'y': 0}
    centerPoint = {'x': 0, 'y': 0}
    print("Let's talk about corridors! Please insert in clockwise order!")
    char = 'y'
    while char == 'y':
        input("Press Enter to save a point in the middle of corridor!")
        centerPoint['x'] = newPoint['x']
        centerPoint['y'] = newPoint['y']
        input("Press Enter to save start position of corridor!")
        startPoint['x'] = newPoint['x']
        startPoint['y'] = newPoint['y']
        input("Press Enter to save end position of corridor!")
        endPoint['x'] = newPoint['x']
        endPoint['y'] = newPoint['y']
        rightRoad, leftRoad, rightRoadLinks, leftRoadLinks = track_road(centerPoint, startPoint, endPoint, r)
        for point in rightRoad:
            plt.plot(rightRoad[point]['x'], rightRoad[point]['y'], 'ro')
            plt.plot(leftRoad[point]['x'], leftRoad[point]['y'], 'bo')
        roads[nRoads] = {'rightRoad': rightRoad, 'leftRoad': leftRoad, 'rightRoadLinks': rightRoadLinks,
                         'leftRoadLinks': leftRoadLinks}
        nRoads += 1
        char = input("Type y to add new corridor or Enter to finish: ")
    return roads


def addCorridorsToGraph(roads, startId, graphPoints, graphLinks, extremePoints, rows, cols):
    """! @brief Add corridors to graph.
    @param roads List of corridors.
    @param startId Start id for corridors points.
    @param graphPoints List of points of the graph.
    @param graphLinks List of links of the graph.
    @param extremePoints List of extreme points of the graph.
    @param rows Number of rows in the map.
    @param cols Number of columns in the map.
    """
    currentId = startId
    for iRoad in roads:
        extremePoints[iRoad] = {'rightRoad': {'start': currentId, 'end': 0}, 'leftRoad': {'start': 0, 'end': 0}}

        # add right road's links
        startRightRoadId = currentId
        for link in roads[iRoad]['rightRoadLinks']:
            graphLinks.append((link[0] + startRightRoadId, link[1] + startRightRoadId))

        # add right road's points
        for point in roads[iRoad]['rightRoad']:
            x, y = translate_point(roads[iRoad]['rightRoad'][point]['x'], roads[iRoad]['rightRoad'][point]['y'], rows,
                                   cols)
            graphPoints[currentId] = {'x': x, 'y': y, 'th': roads[iRoad]['rightRoad'][point]['th']}
            # added translated point to road
            roads[iRoad]['rightRoad'][point]['x'] = x
            roads[iRoad]['rightRoad'][point]['y'] = y
            currentId += 1

        extremePoints[iRoad]['rightRoad']['end'] = currentId - 1

        extremePoints[iRoad]['leftRoad']['end'] = currentId

        # add left road's links
        startLeftRoadId = currentId
        for link in roads[iRoad]['leftRoadLinks']:
            graphLinks.append((link[0] + startLeftRoadId, link[1] + startLeftRoadId))

        i = 0
        deltaId = startLeftRoadId - startRightRoadId
        # add left road's points
        for point in roads[iRoad]['leftRoad']:
            x, y = translate_point(roads[iRoad]['leftRoad'][point]['x'], roads[iRoad]['leftRoad'][point]['y'], rows,
                                   cols)
            graphPoints[currentId] = {'x': x, 'y': y, 'th': roads[iRoad]['leftRoad'][point]['th']}
            # added translated point to road
            roads[iRoad]['leftRoad'][point]['x'] = x
            roads[iRoad]['leftRoad'][point]['y'] = y
            if i % 2 == 0:
                graphLinks.append((currentId, currentId - deltaId))
            else:
                graphLinks.append((currentId - deltaId, currentId))
            currentId += 1
            i += 1

        extremePoints[iRoad]['leftRoad']['start'] = currentId - 1
    return


def addBasesToGraph(bases, roads, startId, graphPoints, graphLinks):
    """! @brief Add bases to graph.
    @param bases List of bases.
    @param roads Dictionary of roads.
    @param startId Start id for graph.
    @param graphPoints List of points of the graph.
    @param graphLinks List of links of the graph.
    @return idGraph Current id of the graph with bases added.
    """
    # assuming that no offices and bases are near each other
    idGraph = startId
    for base in bases:
        theta, startPoint, endPoint = find_nearest_point(roads, bases[base])
        bases[base] = {'x': bases[base]['x'], 'y': bases[base]['y'], 'th': theta}
        graphPoints[idGraph] = {'x': bases[base]['x'], 'y': bases[base]['y'], 'th': theta}
        startPointIndex = find_in_graph(startPoint, graphPoints)
        endPointIndex = find_in_graph(endPoint, graphPoints)
        graphLinks.append((idGraph, endPointIndex))
        graphLinks.append((startPointIndex, idGraph))
        idGraph += 1

    return idGraph


def addOfficesToGraph(offices, roads, startId, graphPoints, graphLinks):
    """! @brief Add offices to graph.
    @param offices List of offices.
    @param roads Dictionary of roads.
    @param startId Start id for graph.
    @param graphPoints List of points of the graph.
    @param graphLinks List of links of the graph.
    @return idGraph Current id of the graph with offices added.
    """
    idGraph = startId
    for office in offices:
        theta, startPoint, endPoint = find_nearest_point(roads, offices[office])
        offices[office] = {'name': offices[office]['name'], 'x': offices[office]['x'], 'y': offices[office]['y'],
                           'th': theta}
        graphPoints[idGraph] = {'x': offices[office]['x'], 'y': offices[office]['y'], 'th': theta}
        startPointIndex = find_in_graph(startPoint, graphPoints)
        endPointIndex = find_in_graph(endPoint, graphPoints)
        graphLinks.append((idGraph, endPointIndex))
        graphLinks.append((startPointIndex, idGraph))
        idGraph += 1

    return idGraph


def connectCorridors(extremePoints, graphLinks):
    """! @brief Connect corridors.
    @param extremePoints Dictionary of extreme points of the graph.
    @param graphLinks List of links of the graph.
    """
    print('Let\'s connect corridors!')
    print('Do you remember corridors\' id?')
    for roadId in extremePoints:
        char = 'y'
        while char == 'y':
            char = input("Where is connected the end of the road " + str(roadId) + "? ")
            print('Extreme point', extremePoints)
            print('Road', roadId)
            try:
                nextRoadId = int(char)
                graphLinks.append(
                    (extremePoints[roadId]['rightRoad']['end'], extremePoints[nextRoadId]['rightRoad']['start']))
                graphLinks.append(
                    (extremePoints[nextRoadId]['leftRoad']['end'], extremePoints[roadId]['leftRoad']['start']))
                print('Connected successfully!\n')
                char = input('Type "y" to add new connection with this road or Enter to continue: ')
            except ValueError:
                print('Wrong input!')
    return


def main():
    """! @brief Main function.
    Call all functions to create graph with a great user experience.
    """
    # point used by callback function
    newPoint = {'x': 0, 'y': 0}

    # read map from file
    r = read_pgm('/Users/lucacordioli/Documents/Lavori/TESI/LogicMove/src/graph_generator_package/maps/povo/mappa.pgm')

    rows, cols = initMap(r, newPoint)

    # input data
    idGraph = 0
    print('Type n for next step!')
    bases, idGraph = inputBases(idGraph, newPoint, rows, cols)
    print('Bases: ', bases, '\n')
    offices, idGraph = inputOffices(idGraph, newPoint, rows, cols)
    print('Offices: ', offices, '\n')
    roads = inputCorridors(newPoint, r)
    print('Roads:', roads, '\n')

    print(idGraph)

    # save offices in file used by telegram bot
    saveOffices(offices)

    # create graph
    graphPoints = {}
    graphLinks = []
    extremePoints = {}

    addCorridorsToGraph(roads, idGraph, graphPoints, graphLinks, extremePoints, rows, cols)
    currentIdGraph = addBasesToGraph(bases, roads, 0, graphPoints, graphLinks)
    addOfficesToGraph(offices, roads, currentIdGraph, graphPoints, graphLinks)
    connectCorridors(extremePoints, graphLinks)
    print('Roads in main: ', roads, '\n')

    # save graph in file
    saveGraph(graphPoints, graphLinks)
    print('Graph saved!')


if __name__ == '__main__':
    main()
