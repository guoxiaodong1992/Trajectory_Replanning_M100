/**
* This file is part of Ewok.
*
* Copyright 2017 Vladyslav Usenko, Technical University of Munich.
* Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
* for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Ewok is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Ewok is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Ewok. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MARSGO_RRT_INIT_WAYPOINT_
#define MARSGO_RRT_INIT_WAYPOINT_


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <sophus/se3.hpp>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/CholmodSupport>

#include <ewok/polynomial_3d_optimization.h>
#include <vector>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;


struct obstacleShape
{
     double x_min,x_max,y_min,y_max;
     obstacleShape(double xmin, double xmax, double ymin, double ymax): x_min(xmin),x_max(xmax),y_min(ymin),y_max(ymax){}
};



namespace marsgo {
template<typename _Scalar = double>
class RrtInitWaypoint{
public:

   typedef Eigen::Matrix<_Scalar, 2, 1> Vector2;
   typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
   typedef Eigen::Matrix<_Scalar, 4, 1> Vector4;

    typedef std::shared_ptr <RrtInitWaypoint> Ptr;
    typename ewok::Polynomial3DOptimization<10>::Vector3Array initWaypoint(typename ewok::Polynomial3DOptimization<10>::Vector3Array& vec)
   {
          setStartGoal(vec);
          setObstacle();
          planWithSimpleSetup();
          return resWp;
   }

   void setStartGoal(typename ewok::Polynomial3DOptimization<10>::Vector3Array& vec)
   {
         startWp=vec[0];
         goalWp=vec[1];
   }


   void setObstacle()
   {
     obs.push_back(obstacleShape(1,5,1,2));
     obs.push_back(obstacleShape(0,4,5,6));
     obs.push_back(obstacleShape(4,8,10,11));
     obs.push_back(obstacleShape(1.5,5.5,11.5,12.5));
     obs.push_back(obstacleShape(6,10,13,14));
   }

   void planWithSimpleSetup()
   {
       // construct the state space we are planning in
       auto space(std::make_shared<ob::SE3StateSpace>());

       // set the bounds for the R^3 part of SE(3)
       ob::RealVectorBounds bounds(3);
       bounds.setLow(-6);
       bounds.setHigh(6);

       space->setBounds(bounds);

       // define a simple setup class
       og::SimpleSetup ss(space);

       // set state validity checking for this space
       ss.setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });


       ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
       // create a random start state
       ob::ScopedState<ob::SE3StateSpace>  start(space);
       start->setX(startWp(0));
       start->setY(startWp(1));
       start->setZ(startWp(2));
       start->rotation().setIdentity();

       // create a random goal state
       ob::ScopedState<ob::SE3StateSpace>  goal(space);
       goal->setX(goalWp(0));
       goal->setY(goalWp(1));
       goal->setZ(goalWp(2));
       goal->rotation().setIdentity();

       // set the start and goal states
       ss.setStartAndGoalStates(start, goal);

       // this call is optional, but we put it in to get more output information
       ss.setup();
       //ss.print();

       // attempt to solve the problem within one second of planning time
       ob::PlannerStatus solved = ss.solve(10.0);

       if (solved)
       {
           std::cout << "Found solution By GuoXiaodong:" << std::endl;
           // print the path to screen
           ss.simplifySolution();
           //ss.getSolutionPath().printAsMatrix(std::cout);

           auto it=ss.getSolutionPath().getStates();

           ob::SE3StateSpace::StateType *se3state=it[0]->as<ob::SE3StateSpace::StateType>();
           ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);


           for(int i=0;i<it.size();i++)
           {
             Vector3 temp(pos->values[0],pos->values[1],pos->values[2]);
             resWp.push_back(temp);
           }
       }
       else
           std::cout << "No solution found" << std::endl;
   }

   bool isStateValid(const ob::State *state)
   {
    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot
    double x=pos->values[0];
    double y=pos->values[1];
    double z=pos->values[2];

    for(int i=0;i<obs.size();i++)
    {
        if(x>=obs[i].x_min&&x<=obs[i].x_max&&y>=obs[i].y_min&&y<=obs[i].y_max)
           return 0;
    }

    if(x>10) return 0;
    return 1;
   }



private:

  std::vector<obstacleShape> obs;

public:

  Vector3 startWp;
  Vector3 goalWp;

  typename ewok::Polynomial3DOptimization<10>::Vector3Array resWp;




};
}

#endif
