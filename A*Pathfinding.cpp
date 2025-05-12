#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
using namespace std;

//---------------------------------------------------------
// Node structure for each cell in the grid
//---------------------------------------------------------
struct Node {
    int x, y;          // Grid coordinates
    float g;           // Cost from the start node to this node
    float h;           // Heuristic: estimated cost from this node to the goal
    float f;           // Total cost: f = g + h
    Node* parent;      // Pointer to parent node (used for reconstructing the path)
    
    // Constructor initializing the coordinates and default values.
    Node(int _x, int _y) : x(_x), y(_y), g(0), h(0), f(0), parent(nullptr) {}
};

//---------------------------------------------------------
// Comparator to order nodes in the priority queue
// Nodes with lower f values have higher priority.
//---------------------------------------------------------
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;
    }
};

//---------------------------------------------------------
// Utility function: Check if a given coordinate is within the grid bounds.
//---------------------------------------------------------
bool isValid(int x, int y, int rows, int cols) {
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}

//---------------------------------------------------------
// Utility function: Check if a given cell is blocked (obstacle).
// Here, grid cells with value 1 are obstacles.
//---------------------------------------------------------
bool isBlocked(const vector<vector<int>>& grid, int x, int y) {
    return grid[x][y] != 0;
}

//---------------------------------------------------------
// Calculate the heuristic (Manhattan distance) between two points.
// This estimates the cost to reach the goal.
//---------------------------------------------------------
float calculateHeuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

//---------------------------------------------------------
// Get valid neighboring nodes (up, down, left, right) from the current node.
//---------------------------------------------------------
vector<Node*> getNeighbors(Node* current, const vector<vector<int>>& grid) {
    vector<Node*> neighbors;
    int rows = grid.size();
    int cols = grid[0].size();
    // Directions: up, down, left, right
    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};
    
    for (int i = 0; i < 4; i++) {
        int newX = current->x + dx[i];
        int newY = current->y + dy[i];
        // Check bounds and if the cell is walkable
        if (isValid(newX, newY, rows, cols) && !isBlocked(grid, newX, newY)) {
            neighbors.push_back(new Node(newX, newY));
        }
    }
    return neighbors;
}

//---------------------------------------------------------
// Reconstruct the path from goal to start by following parent pointers.
// The path is then reversed to get the order from start to goal.
//---------------------------------------------------------
vector<pair<int, int>> reconstructPath(Node* endNode) {
    vector<pair<int,int>> path;
    Node* current = endNode;
    while (current != nullptr) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

//---------------------------------------------------------
// The A* search algorithm implementation.
// It returns a vector of coordinate pairs representing the path.
//---------------------------------------------------------
vector<pair<int, int>> aStarSearch(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {
    int rows = grid.size();
    int cols = grid[0].size();
    
    // Priority queue for the open list (nodes to be evaluated).
    priority_queue<Node*, vector<Node*>, CompareNode> openList;
    // Closed list: a grid tracking which cells have been evaluated.
    vector<vector<bool>> closedList(rows, vector<bool>(cols, false));

    // Create the start node and initialize its costs.
    Node* startNode = new Node(start.first, start.second);
    startNode->g = 0;
    startNode->h = calculateHeuristic(start.first, start.second, goal.first, goal.second);
    startNode->f = startNode->g + startNode->h;
    openList.push(startNode);

    // Keep track of all dynamically allocated nodes for later cleanup.
    vector<Node*> allNodes;
    allNodes.push_back(startNode);

    // Main loop: process nodes until the open list is empty.
    while (!openList.empty()) {
        // Get the node with the lowest f cost.
        Node* current = openList.top();
        openList.pop();

        // Mark the current cell as visited.
        closedList[current->x][current->y] = true;

        // If we reached the goal, reconstruct and return the path.
        if (current->x == goal.first && current->y == goal.second) {
            vector<pair<int,int>> path = reconstructPath(current);
            // Clean up all allocated nodes.
            for (Node* node : allNodes)
                delete node;
            return path;
        }

        // Get neighbors of the current node.
        vector<Node*> neighbors = getNeighbors(current, grid);
        for (Node* neighbor : neighbors) {
            // Skip if the neighbor is already evaluated.
            if (closedList[neighbor->x][neighbor->y]) {
                delete neighbor; // Free memory for this temporary node.
                continue;
            }

            // Calculate tentative g score (assume cost=1 for adjacent moves).
            float tentativeG = current->g + 1;

            // Check if this neighbor is already in our list of allocated nodes.
            bool inOpenList = false;
            for (Node* node : allNodes) {
                if (node->x == neighbor->x && node->y == neighbor->y) {
                    inOpenList = true;
                    // If we found a better path to this node, update its cost and parent.
                    if (tentativeG < node->g) {
                        node->g = tentativeG;
                        node->f = node->g + node->h;
                        node->parent = current;
                    }
                    break;
                }
            }
            // If the neighbor is new, initialize its scores and add it to the open list.
            if (!inOpenList) {
                neighbor->g = tentativeG;
                neighbor->h = calculateHeuristic(neighbor->x, neighbor->y, goal.first, goal.second);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                openList.push(neighbor);
                allNodes.push_back(neighbor);
            }
        }
    }
    // If no path is found, clean up and return an empty path.
    for (Node* node : allNodes)
        delete node;
    return vector<pair<int, int>>();
}

//---------------------------------------------------------
// Main function: Set up the grid, define start/goal, run A* and display the path.
//---------------------------------------------------------
int main() {
    // Define a simple grid:
    // 0 - walkable cell
    // 1 - obstacle
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {1, 1, 0, 0, 0},
        {0, 0, 0, 1, 0}
    };

    pair<int, int> start = {0, 0}; // Starting cell (top-left corner)
    pair<int, int> goal = {4, 4};  // Goal cell (bottom-right corner)

    // Run the A* algorithm.
    vector<pair<int, int>> path = aStarSearch(grid, start, goal);

    // Display the resulting path if found.
    if (path.empty()) {
        cout << "No path found." << endl;
    } else {
        cout << "Path found:" << endl;
        for (auto coord : path) {
            cout << "(" << coord.first << ", " << coord.second << ") ";
        }
        cout << endl;
    }

    return 0;
}
