#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);

     ////////////////// TODO: Implement your A* search here //////////////////////////
    mbot_lcm_msgs::robot_path_t path;
    PriorityQueue openSet;
    std::vector<Node*> closedSet;

    // if the start node is already the goal
    if(startCell == goalCell) {
        // std::vector<Node*> nodes = extract_node_path(&goalNode, &startNode);
        // path.path = extract_pose_path(nodes, distances);
        std::cout << "Already at Destination!" << std::endl;
        path.path_length = path.path.size();
        return path;
    }

    if(!distances.isCellInGrid(startCell.x, startCell.y) || !distances.isCellInGrid(goalCell.x, goalCell.y)){
        std::cout << "Point is not valid!" << std::endl;
        path.path_length = path.path.size();
        return path;
    }

    // Load the startNode into the priority queue
    Node startNode(startCell.x, startCell.y);
    startNode.cell = startCell;
    startNode.h_cost = 0;
    startNode.g_cost = 0;
    openSet.push(&startNode);

    // If openSet is not empty, continue running
    while(!openSet.empty()){
        Node *n = openSet.pop();

        std::vector<Node*> kids = expand_node(n, distances, params);

    }

    return path;
}

double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    if (dx >= dy){
        h_cost = 14*dy + 10*(dx-dy);
    }
    else{
        h_cost = 14*dx + 10*(dy-dx);
    }
    return h_cost;
}

double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0;
    int dx = abs(from->parent->cell.x - from->cell.x);
    int dy = abs(from->parent->cell.y - from->cell.y);
    // If the cell is diagonal
    if(dx == 1 && dy == 1){
        g_cost = 14;
    }
    // If the cell is adjacent
    else{
        g_cost = 10;
    }

    g_cost += from->parent->g_cost;
    return g_cost;
}

double o_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double o_cost = 0;
    if(distances(from->cell.x, from->cell.y) > params.minDistanceToObstacle &&
       distances(from->cell.x, from->cell.y) <= params.maxDistanceWithCost){

        o_cost = pow(params.maxDistanceWithCost - distances(from->cell.x, from->cell.y)*2000, params.distanceCostExponent);
    }

    return o_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    std::vector<Node*> children;
    
    // using 8-way (diagonal distance) heuristics on grid
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};
    
    for(int i = 0; i < 8; i++){
        // Made a new node that is adjacent to the selected node
        cell_t adjacentCell(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        // if it is in the distance grid
        if(distances.isCellInGrid(adjacentCell.x, adjacentCell.y)){
            // if there is no collision
            Node child(adjacentCell.x, adjacentCell.y);
            child.parent = node;
            children.push_back(&child);
        }
    }
    return children;

}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node* current_node = goal_node;
    while (!(*current_node == *start_node)) {
        path.push_back(current_node);
        current_node = current_node->parent;
    }
    path.push_back(start_node);
    return path;

}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
