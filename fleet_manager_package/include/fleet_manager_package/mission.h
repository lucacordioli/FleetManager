#ifndef LOGICMOVECENTRAL_RESQUEST_H
#define LOGICMOVECENTRAL_RESQUEST_H

#include <string>
#include <iostream>
#include <utility>
#include <stack>

using namespace std;

typedef enum {
    LOW,
    MEDIUM,
    HIGH
} priority;

typedef enum {
    NOT_STARTED,
    PICKING,
    DELIVERING,
    FINISHED
} status;

/**
 * @brief Mission class for manage missions and requests.
 */
class Mission {
private:
    int pickUpID;
    int destinationID;
    int slots;
    priority missionPriority;
    status missionStatus;

public:
    stack<int> path;
    /**
     * Create a new Mission object.
     * @brief Constructor.
     * @param pickUpID The id of the node where the robot will pick up the package.
     * @param destinationID The id of the node where the robot will deliver the package.
     * @param slots The number of slots of the package.
     * @param missionPriority The priority of the mission.
     */
    Mission(int pickUpID, int destinationID, int slots, ::priority missionPriority, ::status status);
    /**
     * Print the mission's data, used for test.
     * @brief Print
     */
    void print();
    /**
     * Get the id of the node where the robot will pick up the package.
     * @brief Get pick up id
     * @return The id of the node where the robot will pick up the package.
     */
    int getPickUpId();
    /**
     * Get the id of the node where the robot will deliver the package.
     * @brief Get destination id
     * @return The id of the node where the robot will deliver the package.
     */
    int getDestinationId();
    /**
     * Get the number of slots of the package.
     * @brief Get slots
     * @return The number of slots of the package.
     */
    [[nodiscard]] int getSlots() const;
    /**
     * Get the priority of the mission.
     * @brief Get priority
     * @return The priority of the mission.
     */
    [[nodiscard]] priority getPriority() const;
    /**
     * Get the status of the mission.
     * @brief Get status
     * @return The status of the mission.
     */
    [[nodiscard]] status getStatus() const;
    /**
     * Return the next node where the robot has to go to proceed with the mission.
     * @brief Get next node
     * @return The next node where the robot has to go to proceed with the mission.
     */
    int getNextNode();
    /**
     * Change the status of the mission.
     * @brief Change status
     * @param newStatus The new status of the mission.
     */
    void changeStatus(::status newStatus);
};

#endif //LOGICMOVECENTRAL_RESQUEST_H
