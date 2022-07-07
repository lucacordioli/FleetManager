"""!@package mainPackage
@file mainPackage/trackRoad.py
@brief This file a function for tracking roads.
"""

pi = 3.14


def track_road(centerPoint, startPoint, endPoint, r):
    """!@brief This function tracks a road with right and left lanes.
    The road must be a straight line.
    @param centerPoint: The center point of the road used to calculate central line.
    @param startPoint: The start point of the road.
    @param endPoint: The end point of the road.
    @return rightRoad: The right lane of the road.
    @return leftRoad: The left lane of the road.
    @return rightRoadLinks: The links of the right lane.
    @return leftRoadLinks: The links of the left lane.
    """

    # road is tracked with coordinates of the map (y down)
    distance = 40  # in pixel

    rightRoad = {}
    rightRoadLinks = []
    leftRoad = {}
    leftRoadLinks = []

    mobilePoint = {'x': centerPoint['x'], 'y': centerPoint['y']}

    if abs(startPoint['x'] - endPoint['x']) > abs(startPoint['y'] - endPoint['y']):
        # x is bigger so direction is horizontal
        # find center of the road
        while r[int(mobilePoint['y'])][int(mobilePoint['x'])] != 0:
            mobilePoint['y'] += 1
        yHigh = mobilePoint['y']

        mobilePoint = {'x': centerPoint['x'], 'y': centerPoint['y']}
        while r[int(mobilePoint['y'])][int(mobilePoint['x'])] != 0:
            mobilePoint['y'] -= 1
        yLow = mobilePoint['y']

        centerPoint['y'] = (yLow + yHigh) / 2

        centerHighRoad = (centerPoint['y'] + yHigh) / 2
        centerLowRoad = (centerPoint['y'] + yLow) / 2

        i = 0
        if startPoint['x'] > endPoint['x']:
            # start point is on the right, direction is left
            mobilePoint = {'x': startPoint['x'], 'y': startPoint['y']}
            while mobilePoint['x'] >= endPoint['x']:
                # road save in pixel reference
                rightRoad[i] = {'x': mobilePoint['x'], 'y': centerLowRoad, 'th': pi}
                leftRoad[i] = {'x': mobilePoint['x'], 'y': centerHighRoad, 'th': 0}
                if i > 0:
                    rightRoadLinks.append((i - 1, i))
                    leftRoadLinks.append((i, i - 1))
                i += 1
                mobilePoint['x'] -= distance
        else:
            # start point is on the left, direction is right
            mobilePoint = {'x': startPoint['x'], 'y': startPoint['y']}
            while mobilePoint['x'] <= endPoint['x']:
                # road save in pixel reference
                rightRoad[i] = {'x': mobilePoint['x'], 'y': centerHighRoad, 'th': 0}
                leftRoad[i] = {'x': mobilePoint['x'], 'y': centerLowRoad, 'th': pi}
                if i > 0:
                    rightRoadLinks.append((i - 1, i))
                    leftRoadLinks.append((i, i - 1))
                i += 1
                mobilePoint['x'] += distance

    else:
        # y is bigger so direction is vertical
        # find center of the road
        while r[int(mobilePoint['y'])][int(mobilePoint['x'])] != 0:
            mobilePoint['x'] += 1
        xRight = mobilePoint['x']

        mobilePoint = {'x': centerPoint['x'], 'y': centerPoint['y']}
        while r[int(mobilePoint['y'])][int(mobilePoint['x'])] != 0:
            mobilePoint['x'] -= 1
        xLeft = mobilePoint['x']

        print("y1: ", xLeft, "y2: ", xRight)
        centerPoint['x'] = (xLeft + xRight) / 2

        # absolute reference
        centerLeftRoad = (centerPoint['x'] + xLeft) / 2
        centerRightRoad = (centerPoint['x'] + xRight) / 2

        i = 0
        if startPoint['y'] < endPoint['y']:
            # start point is on the top, direction is bottom
            mobilePoint = {'x': startPoint['x'], 'y': startPoint['y']}
            while mobilePoint['y'] <= endPoint['y']:
                # road save in pixel reference
                # march reference
                rightRoad[i] = {'x': centerLeftRoad, 'y': mobilePoint['y'], 'th': pi * 3/2}
                leftRoad[i] = {'x': centerRightRoad, 'y': mobilePoint['y'], 'th': pi / 2}
                if i > 0:
                    rightRoadLinks.append((i - 1, i))
                    leftRoadLinks.append((i, i - 1))
                i += 1
                mobilePoint['y'] += distance
        else:
            # start point is on the bottom, direction is top
            mobilePoint = {'x': startPoint['x'], 'y': startPoint['y']}
            while mobilePoint['y'] >= endPoint['y']:
                # road save in pixel reference
                # march reference
                rightRoad[i] = {'x': centerRightRoad, 'y': mobilePoint['y'], 'th': pi / 2}
                leftRoad[i] = {'x': centerLeftRoad, 'y': mobilePoint['y'], 'th': pi * 3/2}
                if i > 0:
                    rightRoadLinks.append((i - 1, i))
                    leftRoadLinks.append((i, i - 1))
                i += 1
                mobilePoint['y'] -= distance

    return rightRoad, leftRoad, rightRoadLinks, leftRoadLinks

