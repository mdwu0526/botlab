#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
    : width_(0), height_(0), metersPerCell_(0.05f), cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid &map)
{
    //////////// TODO: initialize the dstances for the obstacle distance grid
    int width = map.widthInCells();
    int height = map.heightInCells();

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // Cell is free
            if (map.logOdds(x, y) < 0)
            {
                distance(x, y) = -1;
            }
            // Cell is occupied
            else
            {
                distance(x, y) = 0;
            }
        }
    }

    return;
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid &map)
{
    resetGrid(map);

    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    initializeDistances(map);

    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(map, *this, searchQueue);

    while (!(searchQueue.empty()))
    {
        // std::cout << "obstacle_distance_grid.cpp: setDistances while loop..." << std::endl;
        auto nextNode = searchQueue.top();
        searchQueue.pop();
        expand_node(nextNode, *this, searchQueue);
    }
    
    // // print out the grid after set it
    // int i = 0;
    // for (auto y = 0; y < heightInCells(); ++y)
    // {
    //     for (auto x = 0; x < widthInCells(); ++x)
    //     {
    //         std::cout << cells_[i] << " ";
    //         i++;
    //     }
    //     std::cout << std::endl;
    // }
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}

void ObstacleDistanceGrid::resetGrid(const OccupancyGrid &map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // If the grid is already the correct size, nothing needs to be done
    if ((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }

    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();

    cells_.resize(width_ * height_);
}

void enqueue_obstacle_cells(const OccupancyGrid &map,
                            ObstacleDistanceGrid &grid,
                            std::priority_queue<DistanceNode> &search_queue)
{
    ///////// TODO: Implement the method for enqueing neighboring cells
    int width = grid.widthInCells();
    int height = grid.heightInCells();
    cell_t cell;

    // If neighboring cells are not an obstacle, add it to the priority queue
    for (cell.y = 0; cell.y < height; cell.y++)
    {
        for (cell.x = 0; cell.x < width; cell.x++)
        {
            if (grid.operator()(cell.x, cell.y) == 0)
            {
                expand_node(DistanceNode(cell, 0), grid, search_queue);
            }
        }
    }
    return;
}

void expand_node(const DistanceNode &node, ObstacleDistanceGrid &grid, std::priority_queue<DistanceNode> &search_queue)
{
    // TODO: Expand to neighboring nodes
    // std::cout << "RUNNING ODG expand_node..." << std::endl;
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};

    for (int i = 0; i < 8; i++)
    {
        // Made a new node that is adjacent to the selected node
        cell_t adjacentCell(node.cell.x + xDeltas[i], node.cell.y + yDeltas[i]);
        if (grid.isCellInGrid(adjacentCell.x, adjacentCell.y))
        {
            if (grid(adjacentCell.x, adjacentCell.y) == -1)
            {
                DistanceNode adjacentNode(adjacentCell, node.distance);
                // If it is a corner adjacent node, add 1.4
                if (abs(xDeltas[i]) + abs(yDeltas[i]) > 1)
                {
                    adjacentNode.distance += 1;
                }
                // If it is a regular adjacent node, just add 1
                else
                {
                    // If diagonal distance is determined
                    // adjacentNode.distance += 1.4142;
                    adjacentNode.distance += 1;
                }
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                search_queue.push(adjacentNode);
            }
        }
    }
}

bool is_cell_free(cell_t cell, const OccupancyGrid &map)
{
    return map.logOdds(cell.x, cell.y) < 0;
}

bool is_cell_occupied(cell_t cell, const OccupancyGrid &map)
{
    return map.logOdds(cell.x, cell.y) >= 0;
}
