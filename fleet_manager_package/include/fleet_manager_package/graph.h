#ifndef BUILD_GRAPH_H
#define BUILD_GRAPH_H

#include <iostream>
#include <list>
#include <queue>
#include <utility>
#include <vector>
#include <stack>
#include <fstream>
#include <math.h>

# define INF 0x3f3f3f3f

using namespace std;

typedef pair<int, float> iPair;

/**
 * @brief LMNode class for manage node in graoh.
 */
class LMNode {
private:
    float x;
    float y;
    float th;
    bool occupied{};
public:
    /**
     * Create a new LMNode object.
     * @brief Constructor.
     * @param x The x coordinate of the node.
     * @param y The y coordinate of the node.
     * @param th The orientation of the node.
     */
    LMNode(float x, float y, float th);
    /**
     * Get the x coordinate of the node.
     * @brief Get x
     * @return The x coordinate of the node.
     */
    [[nodiscard]] float getX() const;
    /**
     * Get the y coordinate of the node.
     * @brief Get y
     * @return The y coordinate of the node.
     */
    [[nodiscard]] float getY() const;
    /**
     * Get the orientation of the node.
     * @brief Get th
     * @return The orientation of the node.
     */
    [[nodiscard]] float getTh() const;
    /**
     * Check if the node is occupied.
     * @brief Is occupied
     * @return True if the node is occupied, false otherwise.
     */
    [[nodiscard]] bool isOccupied() const;
    /**
     * Set the node as occupied.
     * @brief Set occupied
     * @param occupied The node is occupied or not.
     */
    void occupy(bool o);
    /**
     * Print the node's data, used for test.
     * @brief Print
     */
    void print() const;
};

/**
 * @brief Graph class for manage graph.
 */

class Graph{
private:
    int N;
    list<pair<int, float>> *adj;
    vector<LMNode*> nodes;

public:
    /**
     * Create a new Graph object.
     * @brief Constructor.
     */
    Graph();
    /**
     * Add a node to the graph.
     * @brief Add node
     * @param id The id of the node.
     * @param node The pointer at the node to add.
     */
    void addNode(int id, LMNode* node);
    /**
     * Add an edge to the graph.
     * @brief Add edge
     * @param start The id of the first node.
     * @param end The id of the second node.
     * @param lenght The distance between the two nodes.
     */
    void addEdge(int start, int end, float length);
    /**
     * Get the pointer at the node with the current id.
     * @brief Get node
     * @return The pointer at the node with the current id.
     */
    LMNode* getNode(int id);
    /**
     * Print nodes of the graph, used for test.
     * @brief Print nodes
     */
    void printNodes();
    /**
     * Get the nearest node to the given coordinates.
     * @brief Get nearest node
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The id of the nearest node.
     */
    int getNearestNode(float x, float y);
    /**
     * Find the shortest path to go from source to destination.
     * @brief Get shortest path
     * @param src The id of the source node.
     * @param dst The id of the destination node.
     * @param path A stack pointer where the path is saved.
     */
    void shortestPath(int src, int dst, stack<int>& path);
    /**
     * Get the distance between two nodes.
     * @brief Get distance
     * @param src The id of the source node.
     * @param dst The id of the destination node.
     * @return The distance between the two nodes.
     */
    float calculateDistance(int src, int dst);

};

#endif //BUILD_GRAPH_H
