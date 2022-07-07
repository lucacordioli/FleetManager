"""!@package mainPackage
@file mainPackage/fileMaganer.py
@brief This file contains the functions to manage the files used by the program.
"""

import json


def read_pgm(pgmf):
    """!@brief Read a pgm file.
    @param pgmf: The path of the pgm file.
    @return: A raster of the map as list of lists.
    """
    with open(pgmf, 'rb') as f:
        header = f.readline()
        assert header == b'P5\n'
        while True:
            l = f.readline()
            if not l[0] == 35:  # skip any header comment lines
                break
        (width, height) = [int(i) for i in l.split()]

        depth = int(f.readline())
        assert depth <= 255

        raster = []
        for y in range(height):
            row = []
            for x in range(width):
                row.append(ord(f.read(1)))
            raster.append(row)
    return raster


def saveOffices(offices):
    """!@brief Save the offices in a json file with name and id.
    @param offices: The offices to save.
    """
    with open(
            '/Users/lucacordioli/Documents/Lavori/TESI/LogicMove/src/telegram_bot_package/telegram_bot_package/data'
            '/offices.json',
            'w') as o:
        officeToWrite = {}
        for officeId in offices:
            officeToWrite[offices[officeId]['name']] = {'id': officeId}
        o.write(str(json.dumps(officeToWrite)))
    return


def saveGraph(graphPoint, graphLinks):
    """!@brief Save the graph in a txt file.
    Graph format: first line with number of points and links and then the points and links.
    @param graphPoint: The graph points.
    @param graphLinks: The graph links.
    """
    with open('/Users/lucacordioli/Documents/Lavori/TESI/LogicMove/src/fleet_manager_package/maps/povoGraph.txt',
              'w') as f:
        pointLen = len(graphPoint)
        linksLen = len(graphLinks)
        print(pointLen, linksLen)
        f.write(str(pointLen) + ' ' + str(linksLen) + '\n')
        for i in range(0, len(graphPoint)):
            f.write(str(round(graphPoint[i]['x'], 2)) + ' ' + str(round(graphPoint[i]['y'], 2)) + ' ' + str(
                graphPoint[i]['th']) + '\n')

        for link in graphLinks:
            f.write(str(link[0]) + ' ' + str(link[1]) + '\n')
    return
