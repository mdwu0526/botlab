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
    ////////////////// TODO: Implement your A* search here //////////////////////////
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node goalNode(goalCell.x, goalCell.y);
    Node startNode(startCell.x, startCell.y);
    // initialize open list and close list
    PriorityQueue openList;
    std::vector<Node*> closeList;
    // initialize the path
    mbot_lcm_msgs::robot_path_t path;

    // if the start node is already the goal
    if (startCell == goalCell) {
        std::vector<Node*> nodes = extract_node_path( &goalNode, &startNode);
        path.path = extract_pose_path(nodes, distances);
        path.path_length = path.path.size();
        return path;
    }
    
    // start of open list, parent node is NULL
    openList.push(&startNode);

    // loop through the open list
    while (!openList.empty()) {
        // pop the best score node
        Node* n = openList.pop();
        // find neighbors of the best score node
        std::vector<Node*> kids = expand_node(n, distances, params);
        
        for (auto& kid : kids) {
            // calculate f cost for kid: f_cost = h_cost + g_cost
            kid->h_cost = h_cost(kid, &goalNode, distances);
            kid->g_cost = g_cost(n, kid, distances, params);
            kid->f_cost();
            // if this kid reaches the goal >>> return the path
            if (*kid == goalNode) // might use: kid->cell == goalCell
            {
                std::vector<Node *> nodes = extract_node_path(&goalNode, &startNode);
                path.path = extract_pose_path(nodes, distances);
                path.path_length = path.path.size();
                return path;
            }
            // if this kid not reaches the goal && it is not in close list (not yet explored) >>> store this node into open list
            if (!is_in_list(kid,closeList)) {
                openList.push(kid);
            }
            // store the node to clost list
            closeList.push_back(n);
            
        }
    }

    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    double dx = abs(from->cell.x - goal->cell.x);
    double dy = abs(from->cell.y - goal->cell.y);
    h_cost = (dx + dy) + (1.4142 - 2) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = 0;   // TODO: go diagnal is set to 1 in obstacle_distance_grid.cpp, check whether it is correct
    
    // if goal node is far enough from wall
    if (distances(to->cell.x, to->cell.y) >= params.maxDistanceWithCost) {
        g_cost = from->g_cost + 1;
    }
    // if goal node is close to wall
    else {
        // self-defined linear penalty equation, higher penalty when closer to wall
        double g_penalty_max = 100; // modifiable, need to be choose
        double k = - g_penalty_max/params.maxDistanceWithCost;
        double d_distance  = distances(to->cell.x, to->cell.y) - distances(from->cell.x, from->cell.y);
        double g_penalty =k * d_distance;
        // current g cost = privious g cost + penalty g cost
        g_cost = from->g_cost + g_penalty;
    }
    
    return g_cost;
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
            if(distances(adjacentCell.x, adjacentCell.y) > params.minDistanceToObstacle){ // TODO: convert grid to meters
                // add node to children and set parent node, h cost and g cost are calculated seperately
                // Node* adjacentNode = new Node(adjacentCell.x, adjacentCell.y);
                // adjacentNode->parent = node;
                // children.push_back(adjacentNode); 
                Node adjacentNode(adjacentCell.x, adjacentCell.y);
                adjacentNode.parent = node;
                children.push_back(&adjacentNode); //TODO: check, here pushing local variable, maybe point to null pointer?
            }
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
