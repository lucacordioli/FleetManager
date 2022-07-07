"""!@package data
@file data/globaldata.py
@brief Contains global data used in all software.
"""
import pandas as pd

PICKUP, DESTINATION, SLOTS, PRIORITY = range(4)

requests = {}

slotsPerBot = 5


def loadOffices():
    """!@brief Loads offices from json file.
    """
    global offices
    offices = pd.read_json(r'/Users/lucacordioli/Documents/Lavori/TESI/LogicMove/src/telegram_bot_package/telegram_bot_package/data/offices.json')
    return

