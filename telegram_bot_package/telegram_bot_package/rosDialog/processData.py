"""!@package rosDialog
@file rosDialog/processData.py
@brief Contains functions to process data.
"""
import json
import numpy as np
import telegram_bot_package.data.globaldata as dt


class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        """!@brief Convert numpy array to list.
        @param obj: Object to convert.
        @return: List of object.
        """
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super(NpEncoder, self).default(obj)


def processRequest(request):
    """!@brief Process the request taking data from global variable offices.
    @param request: Request to process.
    @return: Processed request.
    """
    finalRequest = 0
    position = {}
    position['name'] = request['pickup']
    position['id'] = dt.offices[request['pickup']]['id']
    request['pickup'] = position
    position = {}
    position['name'] = request['destination']
    position['id'] = dt.offices[request['destination']]['id']
    request['destination'] = position

    return request
