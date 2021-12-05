
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <set>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

struct Cell
{
    int x;
    int y float cost;
};

namespace global_planner
{

    class GlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
        bool isCellValid(int x, int y);
        bool isFree(int x, int y);
        vector<Cell> findFreeNeighborCells(int x, int y);
        vector<int> getAStarPath(int start_x, int start_y, int goal_x, int goal_y);
        vector<int> findPath(int start_x, int start_y, int goal_x, int goal_y, float g_score[][]);
        float calculateHScore(int x1, int y1, int x2, int y2);
        float getMoveCost(int i1, int j1, int i2, int j2);
        vector<Cell> constructPath(int start_x, int start_y, int goal_x, int goal_y, float g_score[]);
        void convertMapToWorld(int map_x, int map_y, float &world_x, float &world_y);

        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        bool *occupancyGridMap;
        float originX;
        float originY;
        float resolution;
        int width;
        int height;

    protected:
        bool initialized_;
    };
};
#endif