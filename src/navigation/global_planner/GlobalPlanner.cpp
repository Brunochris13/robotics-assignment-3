#include "GlobalPlanner.h"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner
{

    GlobalPlanner::GlobalPlanner()
    {
    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            mapSize = width * height;

            occupancyGridMap = new bool[mapSize];
            for (unsigned int iy = 0; iy < height; iy++)
            {
                for (unsigned int ix = 0; ix < width; ix++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                    if (cost == 0)
                        occupancyGridMap[iy * width + ix] = true;
                    else
                        occupancyGridMap[iy * width + ix] = false;
                }
            }

            ROS_INFO("Global Planner Initialized");
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized");
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized");
            return false;
        }

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), start.header.frame_id.c_str());
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                  goal.pose.position.x, goal.pose.position.y);

        //clear the plan, just in case
        plan.clear();

        // Get start and goal world x and y
        float startWorldX = start.pose.position.x;
        float startWorldY = start.pose.position.y;
        float goalWorldX = goal.pose.position.x;
        float goalWorldY = goal.pose.position.y;

        unsigned int startMapX, startMapY;
        if (!costmap_->worldToMap(startWorldX, startWorldY, startMapX, startMapY))
        {
            ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
            return false;
        }

        unsigned int goalMapX, goalMapY;
        if (!costmap_->worldToMap(goalWorldX, goalWorldY, goalMapX, goalMapY))
        {
            ROS_WARN("The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }

        if (!isCellValid(startMapX, startMapY) || !isCellValid(goalMapX, goalMapY))
        {
            ROS_WARN("The planner failed to find a path, choose other goal position");
            return false;
        }

        vector<Cell> bestPath;
        bestPath.clear();

        bestPath = getAStarPath(startMapX, startMapY, goalMapX, goalMapY);

        //if the global planner finds a path
        if (bestPath.size() > 0)
        {
            // convert the path
            for (int i = 0; i < bestPath.size(); i++)
            {
                float x = 0.0;
                float y = 0.0;

                float previous_x = 0.0;
                float previous_y = 0.0;

                Cell cell = bestPath[i];
                Cell previous_cell;
                convertMapToWorld(cell.x, cell.y, x, y);

                if (i != 0)
                    previous_cell = bestPath[i - 1];
                else
                    previous_cell = cell;

                convertMapToWorld(previous_cell.x, previous_cell.y, previous_x, previous_y);

                //Orient the bot towards target
                tf::Vector3 vectorToTarget;
                vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
                float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

                geometry_msgs::PoseStamped pose = goal;

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

                plan.push_back(pose);
            }

            return true;
        }
        else
        {
            ROS_WARN("The planner failed to find a path, choose other goal position");
            return false;
        }
    }

    bool GlobalPlanner::isCellValid(int x, int y)
    {
        bool result = isFree(x, y) if findFreeNeighborCells (x, y).size() == 0 result = false return result
    }

    bool GlobalPlanner::isFree(int x, int y){
        return occupancyGridMap[y * width + x]}

    vector<Cell> GlobalPlanner::findFreeNeighborCells(int x, int y)
    {
        vector<Cell> freeNeighborCells;

        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
            {
                //check whether the index is valid
                if ((y + i >= 0) && (y + i < height) && (x + j >= 0) && (x + j < width) && (!(i == 0 && j == 0)))
                {
                    if (isFree(x, y))
                    {
                        Cell cell;
                        cell.x = x;
                        cell.y = y;
                        freeNeighborCells.push_back(cell);
                    }
                }
            }
        return freeNeighborCells;
    }

    vector<int> GlobalPlanner::getAStarPath(int start_x, int start_y, int goal_x, int goal_y)
    {
        vector<int> bestPath;

        float g_score[width][height];

        for (uint i = 0; i < width; i++)
            for (uint j = 0; j < width; i++)
                g_score[i][j] = infinity;

        bestPath = findPath(start_x, start_y, goal_x, goal_y, g_score);

        return bestPath;
    }

    vector<int> GlobalPlanner::findPath(int start_x, int start_y, int goal_x, int goal_y, float g_score[][])
    {
        vector<int> bestPath;
        vector<int> emptyPath;
        Cell cell;

        multiset<Cell> openCellsList;

        //calculate cost of the start position
        g_score[start_x][start_y] = 0;
        cell.x = start_x;
        cell.y = start_y;
        cell.cost = calculateHScore(start_x, start_y, goal_x, goal_y);

        //add the start cell to the open list
        openCellsList.insert(cell);

        int current_x = start_x;
        int current_y = start_y;

        //while the open list is not empty and till goal square is reached continue the search
        while (!openCellsList.empty() && g_score[goal_x][goal_y] == infinity)
        {
            //choose the cell that has the lowest cost fCost in the open set
            current_x = openCellsList.begin()->x;
            current_y = openCellsList.begin()->y;
            //remove that cell from the openList
            openCellsList.erase(openCellsList.begin());
            //search the neighbors of that cell
            vector<Cell> neighborCells;
            neighborCells = findFreeNeighborCells(current_x, current_y);
            for (uint i = 0; i < neighborCells.size(); i++) //for each neighbor v of cell
            {
                // if the g_score of the neighbor is equal to INF: unvisited cell
                if (g_score[neighborCells[i].x][neighborCells[i].y] == infinity)
                {
                    g_score[neighborCells[i].x][neighborCells[i].y] = g_score[current_x][current_y] + getMoveCost(current_x, current_y, neighborCells[i].x, neighborCells[i].y);
                    Cell cell;
                    // insert the neighborCell
                    cell.x = neighborCells[i].x
                                 cell.y = neighborCells[i].y
                                              cell.cost = g_score[cell.x][cell.y] + calculateHScore(cell.x, cell.y, goal_x, goal_y);
                    openCellsList.insert(cell);
                }
            }
        }

        if (g_score[goal_x][goal_y] != infinity) // if goal gridCell has been reached
        {
            bestPath = constructPath(start_x, start_y, goal_x, goal_y, g_score);
            return bestPath;
        }
        else
        {
            ROS_INFO("Failure to find a path !");
            return emptyPath;
        }
    }

    float GlobalPlanner::calculateHScore(int x1, int y1, int x2, int y2)
    {
        //return abs(x1 - x2) + abs(y1 - y2);
        return sqrt(sqr(x1 - x2) + sqr(y1 - y2));
    }

    float GlobalPlanner::getMoveCost(int i1, int j1, int i2, int j2)
    {
        float moveCost = infinity; //start cost with maximum value. Change it to real cost of gridSquares are connected
        //if gridSquare(i2,j2) exists in the diagonal of gridSquare(i1,j1)
        if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
            moveCost = sqrt(2);
        //if gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
        else if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
            moveCost = 1;
        return moveCost;
    }

    vector<Cell> GlobalPlanner::constructPath(int start_x, int start_y, int goal_x, int goal_y, float g_score[])
    {
        vector<Cell> bestPath;
        vector<Cell> path;

        Cell goalCell;
        goalCell.x = goal_x;
        goalCell.y = goal_y;
        path.insert(0, goalCell);

        int current_x = goal_x;
        int current_y = goal_y;

        while ((goal_x != start_x) && (goal_y != start_y))
        {
            vector<Cell> neighborCells;
            neighborCells = findFreeNeighborCells(current_x, current_y);

            vector<float> gScoresNeighbors;
            for (uint i = 0; i < neighborCells.size(); i++)
                gScoresNeighbors.push_back(g_score[neighborCells[i].x][neighborCells[i].y]);

            int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));

            current_x = neighborCells[posMinGScore].x;
            current_y = neighborCells[posMinGScore].y;

            Cell currentCell;
            currentCell.x = current_x;
            currentCell.y = current_y;

            //insert the neighbor in the path
            path.insert(path.begin() + path.size(), currentCell);
        }
        for (uint i = 0; i < path.size(); i++)
            bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

        return bestPath;
    }

    void GlobalPlanner::convertMapToWorld(int map_x, int map_y, float &world_x, float &world_y)
    {
        world_x = map_x * resolution + originX;
        world_y = map_y * resolution + originY;
    }
};