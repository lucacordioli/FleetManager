#include "fleet_manager_package/bot.h"
#include "rclcpp/rclcpp.hpp"

Bot::Bot(int id, int baseNode, int currentNode, int totalSlots, float battery) : id(id), baseNode(baseNode), currentNode(currentNode),
                                                                   totalSlots(totalSlots), freeSlots(totalSlots),
                                                                   battery(battery) {
    currentMission = nullptr;
    inMovement = false;
}

void Bot::print() {
    cout << "Bot -> Id: " << id << ", Current node: " << currentNode << ", Total slots: " << totalSlots << ", Free slots: "
         << freeSlots << ", Battery: " << battery << endl;
    currentMission->print();

}

int Bot::getId() const {
    return id;
}

int Bot::getBaseNode() const {
    return baseNode;
}


int Bot::getCurrentNode() const {
    return currentNode;
}

bool Bot::havePower() const {
    if(battery>MIN_BATTERY){
        return true;
    }
    return false;
}

bool Bot::haveSlots(int necessarySlots) const {
    if(freeSlots>=necessarySlots){
        return true;
    }
    return false;
}

int Bot::getFreeSlots() const {
    return freeSlots;
}

bool Bot::isInMovement() const {
    return inMovement;
}


void Bot::setCurrentNode(int currentId) {
    currentNode = currentId;
}

void Bot::occupySlots(int necessarySlots) {
    freeSlots = freeSlots - necessarySlots;
}

void Bot::cleanSlots(int cleanSlots) {
    freeSlots = freeSlots + cleanSlots;
}

bool Bot::haveMission() {
    if (currentMission == nullptr){
        return false;
    }else{
        if(currentMission->getStatus() == FINISHED){
            return false;
        }
        return true;
    }
}

void Bot::assignMission(Mission *mission) {
    currentMission = mission;

}

Mission *Bot::getMission() {
    return currentMission;
}

void Bot::setInMovement(bool inM){
    inMovement = inM;
}


