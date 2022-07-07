#ifndef LOGICMOVECENTRAL_BOT_H
#define LOGICMOVECENTRAL_BOT_H

#include <iostream>
#include "mission.h"
#define MIN_BATTERY 30

using namespace std;

/**
 * @brief Bot class for manage robots and store data.
 */
class Bot {
private:
    int id;
    int baseNode;
    int currentNode;
    int totalSlots;
    int freeSlots;
    Mission* currentMission;
    float battery;
    bool inMovement;

public:
    /**
     * Create a new Bot object.
     * @brief Constructor.
     * @param id The id of the bot.
     * @param baseNode The id of the robot's base node.
     * @param currentNode The id of the robot's current node.
     * @param totalSlots The total number of slots of the robot.
     * @param battery The battery of the robot.
     */
    Bot(int id, int baseNode, int currentNode, int totalSlots, float battery);
    /**
     * Print the bot's data, used for test.
     * @brief Print
     */
    void print();
    /**
     * Get the id of the bot.
     * @brief Get id
     * @return The id of the bot.
     */
    [[nodiscard]] int getId() const;
    /**
     * Get the id of the robot's base node.
     * @brief Get base node
     * @return The id of the robot's base node.
     */
    [[nodiscard]] int getBaseNode() const;
    /**
     * Get the id of the robot's current node.
     * @brief Get current node
     * @return The id of the robot's current node.
     */
    [[nodiscard]] int getCurrentNode() const;
    /**
     * Check if the robot have power.
     * @brief Have power
     * @return True if the robot have power (major than a constant), false otherwise.
     */
    [[nodiscard]] bool havePower() const;
    /**
     * Check if the robot have slots.
     * @brief Have slots
     * @param necessarySlots The number of slots that the robot need.
     * @return True if the robot have free slots, false otherwise.
     */
    [[nodiscard]] bool haveSlots(int necessarySlots) const;
    /**
     * Get the number of free slots of the robot.
     * @brief Get free slots
     * @return The number of free slots of the robot.
     */
    [[nodiscard]] int getFreeSlots() const;
    /**
     * Check if the robot is in movement.
     * @brief Is in movement
     * @return True if the robot is in movement, false otherwise.
     */
    [[nodiscard]] bool isInMovement() const;
    /**
     * Set the current node of the robot.
     * @brief Set current node
     * @param currentId The id of the current node.
     */
    void setCurrentNode(int currentId);
    /**
     * Occupy the slots of the robot.
     * @brief Occupy slots
     * @param necessarySlots The number of slots that the robot need.
     */
    void occupySlots(int necessarySlots);
    /**
     * Clean the slots of the robot.
     * @brief Clean slots
     * @param cleanSlots The number of slots of the previous package.
     */
    void cleanSlots(int cleanSlots);
    /**
     * Check if the robot has a mission in progress.
     * @brief Have mission
     * @return True if the robot has a mission in progress, false otherwise.
     */
    bool haveMission();
    /**
     * Assign a mission to the robot.
     * @brief Assign mission
     * @param mission The pointer of mission to assign.
     */
    void assignMission(Mission* mission);
    /**
     * Get the current mission of the robot.
     * @brief Get current mission
     * @return The pointer of the current mission of the robot.
     */
    Mission* getMission();
    /**
     * Set the movement of the robot.
     * @brief Set in movement
     */
    void setInMovement(bool inM);

};

#endif //LOGICMOVECENTRAL_BOT_H
