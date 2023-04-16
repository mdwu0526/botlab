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
    // initialize the path
    mbot_lcm_msgs::robot_path_t path;
    
    if (distances(goalCell.x, goalCell.y) < params.minDistanceToObstacle) {
        return path;
    }
    
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node goalNode(goalCell.x, goalCell.y);
    Node startNode(startCell.x, startCell.y);
    // initialize open list and close list
    PriorityQueue openList;
    std::vector<Node*> closeList;


    // if the start node is already the goal
    if (startCell == goalCell) {
        // std::cout << "Start node is already the goal" << std::endl;
        std::vector<Node*> nodes = extract_node_path( &goalNode, &startNode);
        path.path = extract_pose_path(nodes, distances);
        path.path_length = path.path.size();
        return path;
    }
    
    // start of open list, parent node is NULL
    openList.push(&startNode);

    // loop through the open list
    while (!openList.empty()) {
        
        // std::cout << "In while loop" << std::endl;
        // pop the best score node
        Node* n = openList.pop();
        
        // find neighbors of the best score node
        std::vector<Node*> kids = expand_node(n, distances, params);
        // for (auto& kid : kids) {
        //     std::cout << kid->cell.x << " " << kid->cell.y << std::endl;
        // }
        if (!kids.empty()) {
            for (auto& kid : kids) {
                // std::cout << "--Looping expanded nodes" << std::endl;
                // calculate f cost for kid: f_cost = h_cost + g_cost
                kid->h_cost = h_cost(kid, &goalNode, distances);
                kid->g_cost = g_cost(n, kid, distances, params);
                kid->f_cost();
                // if this kid reaches the goal >>> return the path
                if ((kid->cell.x == goalNode.cell.x) && (kid->cell.y == goalNode.cell.y)) // might use: kid->cell == goalCell
                {
                    std::cout << "Find the goal!" << std::endl;
                    std::vector<Node *> nodes = extract_node_path(kid, &startNode);
                    path.path = extract_pose_path(nodes, distances);
                    path.path_length = path.path.size();
                    // std::cout << "Length of path: " << path.path_length << std::endl;
                    return path;
                }
                // if this kid not reaches the goal && it is not in close list (not yet explored) >>> store this node into open list
                if (!is_in_list(kid,closeList) && !is_in_list(kid,openList.elements)) {
                    openList.push(kid);
                }
            }
            // store the node to clost list
            closeList.push_back(n);
        }
    }
    return path;
}

void deleteKids (std::vector<Node*> &kids) {
    for (auto & kid : kids) {
        delete kid;
    }
}

double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    // std::cout << "--RUNNING h_cost" << std::endl;
    double h_cost = 0;
    double dx = abs(from->cell.x - goal->cell.x);
    double dy = abs(from->cell.y - goal->cell.y);
    h_cost = (dx + dy) + (sqrt(2) - 2) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
//    std::cout << "--RUNNING g_cost" << std::endl;
   double g_cost = 0;   // TODO: go diagonal is set to 1 in obstacle_distance_grid.cpp, check whether it is correct
    
    // g_cost = from->g_cost + 1;

    // // if goal node is far enough from wall
    // if (distances(to->cell.x, to->cell.y) * distances.metersPerCell() >= params.maxDistanceWithCost) {
    //     g_cost = from->g_cost + 1;
    // }
    // // if goal node is close to wall
    // else {
    //     // self-defined linear penalty equation, higher penalty when closer to wall
    //     double g_penalty_max = 100; // modifiable, need to be choose
    //     double k = - g_penalty_max/params.maxDistanceWithCost;
    //     double d_distanceInCell  = distances(to->cell.x, to->cell.y) - distances(from->cell.x, from->cell.y);
    //     double g_penalty =k * d_distanceInCell;
    //     // current g cost = privious g cost + penalty g cost
    //     g_cost = from->g_cost + g_penalty;
    // }

    double cost_obst = 0;
    float cellDistance = distances(from->cell.x, from->cell.y) ;
    if (cellDistance > params.minDistanceToObstacle && cellDistance < params.maxDistanceWithCost){
        cost_obst = pow(params.maxDistanceWithCost - cellDistance, 2.5/*params.distanceCostExponent*/) ;
    }
    
    double cost_move = 0;
    cost_move = sqrt((from->cell.x - to->cell.x)*(from->cell.x - to->cell.x) + (from->cell.y - to->cell.y)*(from->cell.y - to->cell.y));

    g_cost = from->g_cost + cost_obst + cost_move;
    
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    // std::cout << "--RUNNING expand_node..." << std::endl;
    std::vector<Node*> children;
    
    // using 8-way (diagonal distance) heuristics on grid
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};
    
    for(int i = 0; i < 8; i++){
        // Made a new node that is adjacent to the selected node
        cell_t adjacentCell(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        // std::cout << "---" << adjacentCell.x << " " << adjacentCell.y << std::endl;
        // if it is in the distance grid
        if(distances.isCellInGrid(adjacentCell.x, adjacentCell.y)){
            // std::cout << "----"  << "this cell is in grid"  << std::endl;
            
            // // do not check the collision
            // Node* adjacentNode = new Node(adjacentCell.x, adjacentCell.y);
            // adjacentNode->parent = node;
            // children.push_back(adjacentNode); 

            // // if there is no collision
            // // std::cout << "---parameter, metersPerCell: " << distances.metersPerCell() << std::endl;
            // std::cout << "---distance to nearest obstacle in cells: " << distances(adjacentCell.x, adjacentCell.y) << std::endl;
            // std::cout << "---distance to nearest obstacle: " << distances(adjacentCell.x, adjacentCell.y) * distances.metersPerCell() << std::endl;
            // // std::cout << "---robot radius: " << params.minDistanceToObstacle << std::endl;
            // the node is not obstacle and also no collision
            if(distances(adjacentCell.x, adjacentCell.y) >= params.minDistanceToObstacle) {
                Node* adjacentNode = new Node(adjacentCell.x, adjacentCell.y);
                adjacentNode->parent = node;
                children.push_back(adjacentNode); 
            }

        }
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    // std::cout << "--RUNNING extract_node_path..." << std::endl;
    std::vector<Node*> path;
    Node* current_node = goal_node;
    path.push_back(current_node);
    while (current_node->parent != nullptr) {
        current_node = current_node->parent;
        path.push_back(current_node);
    }
    std::reverse(path.begin(), path.end());
    

    // for (int i=0; i< path.size(); i++) {
    //     std::cout << "Print path: " << std::endl;
    //     std::cout << path[i]->cell.x << path[i]->cell.y << std::endl;
    // }

    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    // std::cout << "--RUNNING extract_pose_path..." << std::endl;
    // simple version
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    for (auto& node : nodes) {
        mbot_lcm_msgs::pose_xyt_t pose;
        // convert the cell position to global position
        Point<double> gridPosition = node->cell;
        Point<double> globalPosition = grid_position_to_global_position(gridPosition , distances);

        pose.x = globalPosition.x;
        pose.y = globalPosition.y;
        // waypoints in drive square are all zero, for testing
        pose.theta = 0;
        // calculate pose as the same direction of the line between current and parent point
        // pose.theta = atan2(node->cell.y - node->parent->cell.y, node->cell.x - node->parent->cell.x);
        path.push_back(pose);
    }

    // interpolation version
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
