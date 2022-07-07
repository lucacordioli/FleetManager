#include "fleet_manager_package/mission.h"

#include <utility>

Mission::Mission(int pickUpID, int destinationID, int slots, ::priority missionPriority, ::status status) : pickUpID(pickUpID),
                                                                                                            destinationID(
                                                                                                             destinationID),
                                                                                                            slots(slots),
                                                                                                            missionPriority(
                                                                                                             missionPriority),
                                                                                                            missionStatus(status) {}

void Mission::print() {
    cout << "Mission -> Pickup: " << pickUpID << ", Destination: " << destinationID << ", Slots: " << slots
         << ", Priority: " << missionPriority << ", Status: " << missionStatus << endl;
}

int Mission::getPickUpId() {
    return pickUpID;
}

int Mission::getDestinationId() {
    return destinationID;
}

int Mission::getSlots() const {
    return slots;
}

priority Mission::getPriority() const {
    return missionPriority;
}

void Mission::changeStatus(::status newStatus) {
    missionStatus = newStatus;
}

int Mission::getNextNode() {
    int node = -1;
    if (!path.empty()) {
        node = path.top();
        path.pop();
    }
    return node;
}

status Mission::getStatus() const {
    return missionStatus;
}
