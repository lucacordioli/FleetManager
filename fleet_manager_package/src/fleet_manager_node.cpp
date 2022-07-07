#include <vector>
#include "fleet_manager_package/mission.h"
#include "fleet_manager_package/bot.h"
#include "rclcpp/rclcpp.hpp"
#include "fleet_manager_package/graph.h"
#include "lm_interfaces/msg/request.hpp"
#include "lm_interfaces/msg/botstatus.hpp"
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unistd.h>
#include <thread>

#define MAX_BOT 100

#define MAXN 1000

using namespace std;

/**
 * @brief FleetManager class extends from rclcpp::Node, used for manage fleet of robots.
 */

class FleetManager : public rclcpp::Node {
public:
    /**
     * Create a new FleetManager object.
     * @brief Constructor.
     * @param nodeName The name of the node.
     */
    explicit FleetManager(): rclcpp::Node("central") {
        // Number of robots in the world
        this->declare_parameter<std::int16_t>("n_robots", 3);
        this->get_parameter("n_robots", nRobots);
        RCLCPP_INFO(this->get_logger(), "Robots in the world: %d", nRobots);

        for (int i=0; i<nRobots; i++){
            string completion_topic = "bot" + to_string(i) + "/completion";
            function<void (const std_msgs::msg::Float64::SharedPtr msg)> nextStepCallback = std::bind(
                    &FleetManager::positionReached, this, std::placeholders::_1, i);
            bot_completion[i] = create_subscription<std_msgs::msg::Float64>(completion_topic, 10, nextStepCallback);

            string pub_goal_topic = "bot" + to_string(i) + "/goal";
            pub_goal[i] = create_publisher<geometry_msgs::msg::Pose2D>(pub_goal_topic, rclcpp::SystemDefaultsQoS());
            
            string pub_status_topic = "bot" + to_string(i) + "/botstatus";
            pub_status[i] = create_publisher<lm_interfaces::msg::Botstatus>(pub_status_topic, rclcpp::SystemDefaultsQoS());

            bots.push_back(new Bot(i,i,0,5,100));
            RCLCPP_INFO(this->get_logger(), "Created bot class id:%d", i);

            string position_topic = "bot" + to_string(i) + "/odom";
            function<void (const nav_msgs::msg::Odometry::SharedPtr msg)> reachInitPositionCallback = std::bind(
                    &FleetManager::reachInitPosition, this, std::placeholders::_1, i);
            bot_position[i] = create_subscription<nav_msgs::msg::Odometry>(position_topic, 10, reachInitPositionCallback);
        }
        telegram_request = create_subscription<lm_interfaces::msg::Request>("telegram_request", 10, std::bind(&FleetManager::add_request, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Inizialization completed!");
    }

private:
    int16_t nRobots{};
    vector<Mission*> pendingMissions;
    vector<Bot*> bots;
    Graph graph;
    thread nsThread[MAX_BOT];


    /**
     * This function is called one time, when the robot's position is received.
     * @brief Reach the robot's base postion.
     * @param msg Current posizion of the robot.
     * @param id Id of the robot.
     */
    void reachInitPosition(const nav_msgs::msg::Odometry::SharedPtr msg, int id){
        RCLCPP_INFO(this->get_logger(), "Reaching initial position...");

        int nearest_node = graph.getNearestNode(float(msg->pose.pose.position.x), float(msg->pose.pose.position.y));
        RCLCPP_INFO(this->get_logger(), "Nearest node: %d", nearest_node);

        bots[id]->assignMission(new Mission(nearest_node, bots[id]->getBaseNode(), 0, LOW, DELIVERING));

        geometry_msgs::msg::Pose2D poseMsg;
        LMNode* nextLMNode = graph.getNode(nearest_node);
        poseMsg.x = nextLMNode->getX();
        poseMsg.y = nextLMNode->getY();
        poseMsg.theta = nextLMNode->getTh();

        RCLCPP_INFO(this->get_logger(), "Nearest node position x: %f, y: %f, theta: %f", poseMsg.x, poseMsg.y, poseMsg.theta);

        bots[id]->setCurrentNode(nearest_node);
        while(nextLMNode->isOccupied()){}
        nextLMNode->occupy(true);
        pub_goal[id]->publish(poseMsg);
        bots[id]->setInMovement(true);
        RCLCPP_INFO(this->get_logger(), "Reaching");

        // unsubscribe
        bot_position[id].reset();
    }

    /**
     * This function is called when a new request is received, provide to create a new instance of class Request.
     * @brief Add request.
     * @param msg Request message.
     */
    void add_request(const lm_interfaces::msg::Request::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "NEW REQUEST");
        auto* request = new Mission(msg->pickup_id, msg->dest_id, msg->slots, static_cast<priority>(msg->priority), NOT_STARTED);
        assignToBot(request);
    }

    /**
     * This function is called when the robot's position is reached.
     * @brief Assign to bot
     * @param request Pointer to a instance of Mission class.
     */
    void assignToBot(Mission* request){
        RCLCPP_INFO(this->get_logger(), "Assigning to bot...");
        float distance = INF;
        Bot* bestBot = nullptr;
        for (Bot* bot: bots){
            if (bot->havePower() && bot->haveSlots(request->getSlots())){
                if(bot->haveMission() && (bot->getMission()->getStatus() <= PICKING || bot->getMission()->getStatus() >= FINISHED)){
                    if (request->getPriority() > bot->getMission()->getPriority() || bot->getMission()->getStatus() >= FINISHED){
                        float tmpDistance = graph.calculateDistance(bot->getCurrentNode(), request->getPickUpId());
                        if (tmpDistance < distance){
                            distance = tmpDistance;
                            bestBot = bot;
                        }
                    }
                } else {
                    float tmpDistance = graph.calculateDistance(bot->getCurrentNode(), request->getPickUpId());
                    if (tmpDistance < distance){
                        distance = tmpDistance;
                        bestBot = bot;
                    }
                }
            }
        }
        if (bestBot == nullptr){
            RCLCPP_INFO(this->get_logger(), "No bot available for this request, now is in pending");
            pendingMissions.push_back(request);
        } else {
            RCLCPP_INFO(this->get_logger(), "Bot chosed");
            if (nsThread[bestBot->getId()].joinable()){
                nsThread[bestBot->getId()].join();
            }
            if(bestBot->haveMission() && bestBot->getMission()->getStatus() < FINISHED){
                pendingMissions.push_back(bestBot->getMission());
                assignPendingMission();
            }
            bestBot->assignMission(request);
            if (!bestBot->isInMovement()){
                nsThread[bestBot->getId()] = thread(&FleetManager::nextStep, this, bestBot->getId());
            }
            RCLCPP_INFO(this->get_logger(), "Mission assigned to bot");
        }
    }

    /**
     * This function is called when a new message is published on the goal topic.
     * @brief Position Reached
     * @param msg Goal message.
     * @param id Id of the robot.
     */
    void positionReached(const std_msgs::msg::Float64::SharedPtr& msg, int id){
        // RCLCPP_INFO(this->get_logger(), "Bot %d, Status: %f", id, msg->data);
        if (msg->data == 1){
            if (nsThread[id].joinable()){
                nsThread[id].join();
            }
            bots[id]->setInMovement(false);
            nsThread[id] = thread(&FleetManager::nextStep, this, id);
        }
    }

    /**
     * Publish to /goal topic the next node to reach, update the status of the mission
     * and call the function botPubStatus.
     * @brief Next step
     * @see botPubStatus
     * @see Graph::shortestPath
     * @param id The id of the robot.
     */
    void nextStep(int id){
        graph.getNode(bots[id]->getCurrentNode())->occupy(false);
        int nextNode = bots[id]->getMission()->getNextNode();
        RCLCPP_INFO(this->get_logger(), "Next node: %d", nextNode);
        status missionStatus = bots[id]->getMission()->getStatus();
        if (nextNode == -1){
            switch (missionStatus){
                case NOT_STARTED:
                    bots[id]->getMission()->changeStatus(PICKING);
                    graph.shortestPath(bots[id]->getCurrentNode(), bots[id]->getMission()->getPickUpId(), bots[id]->getMission()->path);
                    RCLCPP_INFO(this->get_logger(), "Mission in PICKING  with bot %d", id);
                    break;
                case PICKING:
                    RCLCPP_INFO(this->get_logger(), "Upload pack on robot %d", id);
                    bots[id]->occupySlots(bots[id]->getMission()->getSlots());
                    // we now use a timer but in future wait for a button on the bot
                    usleep(2000000);
                    bots[id]->getMission()->changeStatus(DELIVERING);
                    graph.shortestPath(bots[id]->getMission()->getPickUpId(), bots[id]->getMission()->getDestinationId(), bots[id]->getMission()->path);
                    RCLCPP_INFO(this->get_logger(), "Mission in DELIVERING with bot %d", id);
                    break;
                case DELIVERING:
                    RCLCPP_INFO(this->get_logger(), "Download pack on robot %d", id);
                    bots[id]->cleanSlots(bots[id]->getMission()->getSlots());
                    // we now use a timer but in future wait for a button on the bot
                    usleep(2000000);
                    bots[id]->getMission()->changeStatus(FINISHED);
                    graph.shortestPath(bots[id]->getMission()->getDestinationId(), bots[id]->getBaseNode(), bots[id]->getMission()->path);
                    bots[id]->cleanSlots(bots[id]->getMission()->getSlots());
                    RCLCPP_INFO(this->get_logger(), "Mission FINISHED with bot %d, bot is returning in base", id);
                    assignPendingMission();
                    break;
                case FINISHED:
                    delete bots[id]->getMission();
                    bots[id]->assignMission(nullptr);
                    RCLCPP_INFO(this->get_logger(), "Bot %d is in base", id);
                    return;
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "Error: not in correct status.");
            }
            nextNode = bots[id]->getMission()->getNextNode();
            nextNode = bots[id]->getMission()->getNextNode();
            if (nextNode == -1){
                RCLCPP_INFO(this->get_logger(), "Yet in position");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Next node if preview was -1: %d", nextNode);
        }
        if (nextNode == bots[id]->getCurrentNode()){
            RCLCPP_INFO(this->get_logger(), "Yet in position");
            return;
        }
        geometry_msgs::msg::Pose2D msg;
        LMNode* nextLMNode = graph.getNode(nextNode);
        msg.x = nextLMNode->getX();
        msg.y = nextLMNode->getY();
        msg.theta = nextLMNode->getTh();
        RCLCPP_INFO(this->get_logger(), "Next node position x: %f, y: %f, theta: %f", msg.x, msg.y, msg.theta);

        bots[id]->setCurrentNode(nextNode);
        while(nextLMNode->isOccupied()){
            RCLCPP_INFO(this->get_logger(), "Node %d is occupied", nextNode);
        }
        nextLMNode->occupy(true);
        pub_goal[id]->publish(msg);
        pubBotStatus(id);
        bots[id]->setInMovement(true);
    }

    /**
     * Check if there is a pending mission and assign the most important to a bot.
     * @brief Assign pending mission
     */
    void assignPendingMission() {
        if(!pendingMissions.empty()){
            priority p = LOW;
            Mission* m = nullptr;
            int idMission = -1;
            int i = 0;
            for (auto & pendingMission : pendingMissions){
                if(pendingMission->getPriority() >= p){
                    p = pendingMission->getPriority();
                    m = pendingMission;
                    idMission = i;
                }
                i++;
            }
            RCLCPP_INFO(this->get_logger(), "Pending mission assigned");
            pendingMissions.erase(pendingMissions.begin() + idMission);
            m->changeStatus(NOT_STARTED);
            while (!m->path.empty()){
                m->path.pop();
            }
            assignToBot(m);
        }else{
            RCLCPP_INFO(this->get_logger(), "No mission in pending.");
        }
    };

    /**
     * Publish the robot's status to /status topic.
     * @param botId The id of the robot.
     */
    void pubBotStatus (int botId) {
        lm_interfaces::msg::Botstatus msg;
        msg.status = bots[botId]->getMission()->getStatus();
        msg.free_slots = bots[botId]->getFreeSlots();
        msg.current_node_id = bots[botId]->getCurrentNode();

        pub_status[botId]->publish(msg);
    }

    rclcpp::Subscription<lm_interfaces::msg::Request>::SharedPtr telegram_request;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bot_completion[MAX_BOT];
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bot_position[MAX_BOT];
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_goal[MAX_BOT];
    rclcpp::Publisher<lm_interfaces::msg::Botstatus>::SharedPtr pub_status[MAX_BOT];
};

/**
 * Main function.
 * @param argc Number of arguments.
 * @param argv Arguments.
 * @return 0 if everything went well.
 */
int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FleetManager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

