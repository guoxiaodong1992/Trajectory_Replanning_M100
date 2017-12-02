#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher vis_pub;
ros::Publisher traj_pub;
ros::Publisher globalmap_pub;

class planner {
public:
    void setStart(double x, double y, double z)
    {
        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(x,y,z);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef->clearStartStates();
        pdef->addStartState(start);
    }
    void setGoal(double x, double y, double z)
    {
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(x,y,z);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef->clearGoal();
        pdef->setGoalState(goal);
        std::cout << "goal set to: " << x << " " << y << " " << z << std::endl;
    }
    void updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
    {
        tree_obj = map;
    }
    // Constructor
    planner(void)
    {
        //四旋翼的障碍物几何形状
        Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.03, 0.03, 0.03));
        //分辨率参数设置
        fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
        tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);

        //解的状态空间
        space = ob::StateSpacePtr(new ob::SE3StateSpace());

        // create a start state
        ob::ScopedState<ob::SE3StateSpace> start(space);

        // create a goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space);

        // set the bounds for the R^3 part of SE(3)
        // 搜索的三维范围设置
        ob::RealVectorBounds bounds(3);

        bounds.setLow(0,-5);
        bounds.setHigh(0,5);
        bounds.setLow(1,-5);
        bounds.setHigh(1,5);
        bounds.setLow(2,0);
        bounds.setHigh(2,6);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        start->setXYZ(0,0,0);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // start.random();

        goal->setXYZ(0,0,5);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // goal.random();


        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));

        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // set Optimizattion objective
        pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));

        std::cout << "Initialized: " << std::endl;
    }
    // Destructor
    ~planner()
    {
    }
    void replan(void)
    {

        std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
        if(path_smooth->getStateCount () <= 2)
            plan();
        else
        {
            for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
            {
                if(!replan_flag)
                    replan_flag = !isStateValid(path_smooth->getState(idx));
                else
                    break;

            }
            if(replan_flag)
                plan();
            else
                std::cout << "Replanning not required" << std::endl;
        }

    }
    void plan(void)
    {

        // create a planner for the defined space
        og::InformedRRTstar* rrt = new og::InformedRRTstar(si);

        //设置rrt的参数range
        rrt->setRange(0.05);

        ob::PlannerPtr plan(rrt);

        // set the problem we are trying to solve for the planner
        plan->setProblemDefinition(pdef);

        // perform setup steps for the planner
        plan->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        std::cout << "problem setting\n";
        // print the problem settings
        pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = plan->solve(1);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            std::cout << "Found solution:" << std::endl;
            ob::PathPtr path = pdef->getSolutionPath();
            og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            pth->printAsMatrix(std::cout);
            // print the path to screen
            // path->print(std::cout);


            nav_msgs::Path msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "/world";

            for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
            {
                const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

                // extract the first component of the state and cast it to what we expect
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // extract the second component of the state and cast it to what we expect
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                geometry_msgs::PoseStamped pose;

//              pose.header.frame_id = "/world"

                pose.pose.position.x = pos->values[0];
                pose.pose.position.y = pos->values[1];
                pose.pose.position.z = pos->values[2];

                pose.pose.orientation.x = rot->x;
                pose.pose.orientation.y = rot->y;
                pose.pose.orientation.z = rot->z;
                pose.pose.orientation.w = rot->w;

                msg.poses.push_back(pose);

            }
            for(int i=0;i<10;i++)
            {
            traj_pub.publish(msg);
            //std::cout<<"Publish Marker!"<<std::endl;
            ros::Duration(0.1).sleep();
            }


/*
            //Path smoothing using bspline
            //B样条曲线优化
            og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
            path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            pathBSpline->smoothBSpline(*path_smooth,3);
            // std::cout << "Smoothed Path" << std::endl;
            // path_smooth.print(std::cout);


            //Publish path as markers

            nav_msgs::Path smooth_msg;
            smooth_msg.header.stamp = ros::Time::now();
            smooth_msg.header.frame_id = "/world";

            for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
            {
                    // cast the abstract state type to the type we expect
                const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

                // extract the first component of the state and cast it to what we expect
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // extract the second component of the state and cast it to what we expect
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                geometry_msgs::PoseStamped point;

                point.header.frame_id = "/world";

                point.pose.position.x = pos->values[0];
                point.pose.position.y = pos->values[1];
                point.pose.position.z = pos->values[2];

                point.pose.orientation.x = rot->x;
                point.pose.orientation.y = rot->y;
                point.pose.orientation.z = rot->z;
                point.pose.orientation.w = rot->w;

                smooth_msg.poses.push_back(point);

                //std::cout << "Published marker: " << idx << std::endl;
            }
            vis_pub.publish(smooth_msg);
             ros::Duration(0.1).sleep();*/


            // Clear memory
            pdef->clearSolutionPaths();
            replan_flag = false;

        }
        else
            std::cout << "No solution found" << std::endl;
    }
private:

    // construct the state space we are planning in
    ob::StateSpacePtr space;

    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si;

    // create a problem instance
    ob::ProblemDefinitionPtr pdef;

    og::PathGeometric* path_smooth;

    bool replan_flag = false;

    std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

    std::shared_ptr<fcl::CollisionGeometry> tree_obj;

    bool isStateValid(const ob::State *state)
    {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        fcl::CollisionObject treeObj((tree_obj));
        fcl::CollisionObject aircraftObject(Quadcopter);

        // check validity of state defined by pos & rot
        fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        aircraftObject.setTransform(rotation, translation);
        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
        if(!collisionResult.isCollision())std::cout<<"State valid!";
        else std::cout<<std::endl<<"Invalid!"<<std::endl;
        return(!collisionResult.isCollision());
    }

    // Returns a structure representing the optimization objective to use
    // for optimal motion planning. This method returns an objective which
    // attempts to minimize the length in configuration space of computed
    // paths.
    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        // obj->setCostThreshold(ob::Cost(1.51));
        return obj;
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
        return obj;
    }

};

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{
    //loading octree from binary
    // const std::string filename = "/home/xiaopeng/dense.bt";
    // octomap::OcTree temp_tree(0.1);
    // temp_tree.readBinary(filename);
    // fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));

    // convert octree to collision object
    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));

    // Update the octree used for collision checking
    planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
    //planner_ptr->replan();
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
{
    planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
    planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
    planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
    planner_ptr->plan();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_planner");
    ros::NodeHandle n;
    planner planner_object;

    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
    // ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/rovio/odometry", 1, boost::bind(&odomCb, _1, &planner_object));
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/goal/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));
    ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&startCb, _1, &planner_object));

//  vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    vis_pub = n.advertise<nav_msgs::Path>( "visualization_marker", 0 );
//  traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
    traj_pub = n.advertise<nav_msgs::Path>("waypoints",1);
    globalmap_pub= n.advertise<octomap_msgs::Octomap>("/octomap_binary",1);

//publish the global map
    octomap_msgs::Octomap octomap;
    octomap.binary=1;
    octomap.id=1;
    octomap.resolution=1;
    octomap.header.frame_id="/world";
    octomap.header.stamp=ros::Time::now();
    const std::string filename = "/home/gxd/catkin_replan/src/rrt_octomap/map/fr_079.bt";
    octomap::OcTree*  octo_tree=new octomap::OcTree(filename);
    bool res=octomap_msgs::fullMapToMsg(*octo_tree,octomap);
    if(res) std::cout<<"Convert Successful!"<<std::endl;

    //update map
    fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(octo_tree));

    // Update the octree used for collision checking
    planner_object.updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
    for(int i=0;i<3;i++)
    {
       globalmap_pub.publish(octomap);
       sleep(1);
       std::cout<<i<<"times"<<std::endl;
    }

    planner_object.plan();
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    ros::spin();

    return 0;
}
