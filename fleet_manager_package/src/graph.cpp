#include "fleet_manager_package/graph.h"


LMNode::LMNode(float x, float y, float th) : x(x), y(y), th(th) {}

float LMNode::getX() const {
    return x;
}

float LMNode::getY() const {
    return y;
}

float LMNode::getTh() const {
    return th;
}

bool LMNode::isOccupied() const {
    return occupied;
}

void LMNode::print() const {
    cout << "Node -> X: " << x << "; Y: " << y << "; Th: " << th << endl;

}

void LMNode::occupy(bool o) {
    occupied = o;
}


Graph::Graph() {
    int N, L;
    ifstream file ("/Users/lucacordioli/Documents/Lavori/TESI/LogicMove/src/fleet_manager_package/maps/povoGraph.txt");
    string sf;
    // Read N
    file >> sf;
    N = stoi(sf);
    // Real L
    file >> sf;
    L = stoi(sf);

    // Initialize value
    this->N = N;
    adj = new list<iPair> [N];

    float x, y, th;
    for(int i=0; i<N; i++){
        file >> sf;
        x = stof(sf);
        file >> sf;
        y = stof(sf);
        file >> sf;
        th = stof(sf);
        auto* n = new LMNode(x, y, th);
        addNode(i, n);
    }
    int s, e;
    float length;
    for(int i=0; i<L; i++){
        file >> sf;
        s = stoi(sf);
        file >> sf;
        e = stoi(sf);
        length = float(sqrt(pow(nodes[s]->getX() - nodes[e]->getX(), 2) + pow(nodes[s]->getY() - nodes[e]->getY(), 2)));
        addEdge(s, e, length);
    }
}

void Graph::addNode(int id, LMNode* node){
    nodes.insert(nodes.begin() + id, node);
}

void Graph::addEdge(int start, int end, float length) {
    adj[start].emplace_back(end, length);
}

LMNode* Graph::getNode(int id) {
    return nodes[id];
}

void Graph::printNodes() {
    for(auto & node : nodes){
        node->print();
    }
}

int Graph::getNearestNode(float x, float y) {
    float minDist = INF;
    int idNode = -1;
    float distance;
    for(int i=0; i<N; i++){
        distance = float(sqrt(pow(nodes[i]->getX() - x, 2) + pow(nodes[i]->getY() - y, 2)));
        if(distance < minDist){
            minDist = distance;
            idNode = i;
        }
    }
    return idNode;
}

void Graph::shortestPath(int src, int dst, stack<int>& path) {
    // Create a priority queue to store vertices that are being preprocessed.
    priority_queue<iPair, vector<iPair>, greater<iPair>> pq;

    // Create fathers vector
    vector<int> fathers(N, -1);

    // Create vector for distance and initialize as infinite
    vector<float> dist(N, INF);

    // Insert source itself in priority queue and initialize its distance as 0.
    pq.push(make_pair(0, src));
    dist[src] = 0;

    //Looping till priority queue becomes empty (or all distances are not finalized)
    while(!pq.empty()) {
        // The first vertex in pair is the minimum distance
        // vertex, extract it from priority queue.
        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = pq.top().second;
        pq.pop();

        // 'i' is used to get all adjacent vertices of a vertex
        list< pair<int, float> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = (*i).first;
            float weight = (*i).second;

            //  If there is shorted path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                // Updating distance of v
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));
                fathers[v] = u;
            }
        }
    }

    // cycle
    int v = dst;
    while (v!=-1){
        path.push(v);
        v = fathers[v];
    }
}

float Graph::calculateDistance(int src, int dst) {
    stack<int> path;
    shortestPath(src, dst, path);
    float distance = 0;
    int prevNode = src;
    int currentNode;
    while(!path.empty()){
        currentNode = path.top();
        distance = distance + float(sqrt(pow(nodes[currentNode]->getX() - nodes[prevNode]->getX(), 2) + pow(nodes[currentNode]->getY() - nodes[prevNode]->getY(), 2)));
        path.pop();
        prevNode = currentNode;
    }
    return distance;
}


