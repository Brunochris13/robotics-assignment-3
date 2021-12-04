#include "GlobalPlanner.h"
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

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
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
    }

};