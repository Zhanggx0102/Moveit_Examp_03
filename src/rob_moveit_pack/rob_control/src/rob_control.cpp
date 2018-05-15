/*********************************************************************
Copyright (c) <2018>, <Shawn Zhang>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the <organization>.
4. Neither the name of the <organization> nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY <Shawn Zhang> ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <Shawn Zhang> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *********************************************************************/
// Created on: Dec. 22th 2017
// Last change: Dec. 22th, 2017

#include <rob_control.h>
#include <list>

//#include "armplaning_client.h"

using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> R_TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> L_TrajectoryServer;

std::list<robotState> r_targetPointList;		// list of points to move to
std::list<robotState> l_targetPointList;		// list of points to move to

double deg2rad = 3.14159/180.0;
double rad2deg = 180.0/3.14159;

//***************************************************************************
// Processing and JointTrajectoryAction
void r_executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& r_goal, R_TrajectoryServer* r_as)
{
  //double rad2deg = 180.0 / 3.141;
  robotState rs;

  float lastDuration = 0.0;

  int nrOfPoints = r_goal->trajectory.points.size();		// Number of points to add
  for(int i=0; i<nrOfPoints; i++)
  {
	  rs.j[0] = r_goal->trajectory.points[i].positions[3]*rad2deg;	// ros values come in rad, internally we work in degree
	  rs.j[1] = r_goal->trajectory.points[i].positions[0]*rad2deg;
	  rs.j[2] = r_goal->trajectory.points[i].positions[2]*rad2deg;
	  rs.j[3] = r_goal->trajectory.points[i].positions[1]*rad2deg;
	  rs.j[4] = r_goal->trajectory.points[i].positions[4]*rad2deg;
	  
	  float dtmp = r_goal->trajectory.points[i].time_from_start.toSec();
	  rs.duration = dtmp - lastDuration;// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  r_targetPointList.push_back(rs);//push_back() 在list的末尾添加一个元素 
  }
  r_as->setSucceeded();

  //debug msg
 ROS_INFO("right arm recv: %f %f %f %f %f ,duration: %f", rs.j[0],rs.j[1],rs.j[2],rs.j[3],rs.j[4],rs.duration);
  
}
//***************************************************************************
// Processing and JointTrajectoryAction
void l_executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& l_goal, R_TrajectoryServer* l_as)
{
  //double rad2deg = 180.0 / 3.141;
 ///* 
 robotState ls;

 float lastDuration = 0.0;

  int nrOfPoints = l_goal->trajectory.points.size();		// Number of points to add
  for(int i=0; i<nrOfPoints; i++){
	  ls.j[0] = l_goal->trajectory.points[i].positions[3]*rad2deg;	// ros values come in rad, internally we work in degree
	  ls.j[1] = l_goal->trajectory.points[i].positions[0]*rad2deg;
	  ls.j[2] = l_goal->trajectory.points[i].positions[2]*rad2deg;
	  ls.j[3] = l_goal->trajectory.points[i].positions[1]*rad2deg;
	  ls.j[4] = l_goal->trajectory.points[i].positions[4]*rad2deg;
	  
	  float dtmp = l_goal->trajectory.points[i].time_from_start.toSec();
	  ls.duration = dtmp - lastDuration;// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  l_targetPointList.push_back(ls);//push_back() 在list的末尾添加一个元素 
  }
  l_as->setSucceeded();

  //debug msg
 ROS_INFO("left arm recv: %f %f %f %f %f ,duration: %f", ls.j[0],ls.j[1],ls.j[2],ls.j[3],ls.j[4],ls.duration);
//*/
}

//*************************************************************************
void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

//******************** MAIN ************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "rob_mover");
	ros::NodeHandle n2;

	//Start the ActionServer for JointTrajectoryActions and GripperCommandActions from MoveIT
	R_TrajectoryServer r_tserver(n2, "r_rob_mover/follow_joint_trajectory", boost::bind(&r_executeTrajectory, _1, &r_tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
  	r_tserver.start();

	L_TrajectoryServer l_tserver(n2, "l_rob_mover/follow_joint_trajectory", boost::bind(&l_executeTrajectory, _1, &l_tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
  	l_tserver.start();

	// Start the robot
	rob_robots::rob_control robot;
	robot.init();	
	robot.mainLoop();		//spinning is done inside the main loop			

  	signal(SIGINT,quit);	
	return(0);
}

namespace rob_robots{
	//*************************************************************************************
	rob_control::~rob_control(){

	}
	//*************************************************************************************
	void rob_control::init(){
		ROS_INFO("...initing...");

		flag_stop_requested = false;
		//this deside the control msg send frequency 
        cycleTime = 10.0;// in ms
		nrOfJoints = 5;

		r_setPointState.j[0] =  0.0;	// values are initialized with 6 field to be usable for rob right arm 
		r_setPointState.j[1] =  0.0;
		r_setPointState.j[2] =  0.0;
		r_setPointState.j[3] =  0.0;
        r_setPointState.j[4] =  0.0;
		r_setPointState.duration = 0;

		l_setPointState.j[0] =  0.0;	// values are initialized with 6 field to be usable for rob left arm 
		l_setPointState.j[1] =  0.0;
		l_setPointState.j[2] =  0.0;
		l_setPointState.j[3] =  0.0;
        l_setPointState.j[4] =  0.0;
		l_setPointState.duration = 0;

		// when starting up (or when reading the HW joint values) the target position has to be aligned with the setPoint position
		for(int i=0; i<nrOfJoints; i++)
		{
			r_targetState.j[i] = r_setPointState.j[i];
			l_targetState.j[i] = l_setPointState.j[i];
		}
			
        r_targetState.duration = 0.0;//init the timeStamp
		l_targetState.duration = 0.0;//init the timeStamp

		for(int j=0;j<6;j++)
		{
			r_controlData[j] = 0;
			l_controlData[j] = 0;
		}

		r_msgJointsCurrent.header.stamp = ros::Time::now();
		r_msgJointsCurrent.name.resize(5);
		r_msgJointsCurrent.position.resize(5);
		r_msgJointsCurrent.name[0] ="r_shoulder_joint";
		r_msgJointsCurrent.position[0] = 0.0;
		r_msgJointsCurrent.name[1] ="r_bigarm_joint";
		r_msgJointsCurrent.position[1] = 0.0;
        r_msgJointsCurrent.name[2] ="r_rotatearm_joint";
		r_msgJointsCurrent.position[2] = 0.0;
        r_msgJointsCurrent.name[3] ="r_elbow_joint";
		r_msgJointsCurrent.position[3] = 0.0;
		r_msgJointsCurrent.name[4] ="r_wrist_joint";
		r_msgJointsCurrent.position[4] = 0.0;

		l_msgJointsCurrent.header.stamp = ros::Time::now();
		l_msgJointsCurrent.name.resize(5);
		l_msgJointsCurrent.position.resize(5);
		l_msgJointsCurrent.name[0] ="l_shoulder_joint";
		l_msgJointsCurrent.position[0] = 0.0;
		l_msgJointsCurrent.name[1] ="l_bigarm_joint";
		l_msgJointsCurrent.position[1] = 0.0;
        l_msgJointsCurrent.name[2] ="l_rotatearm_joint";
		l_msgJointsCurrent.position[2] = 0.0;
        l_msgJointsCurrent.name[3] ="l_elbow_joint";
		l_msgJointsCurrent.position[3] = 0.0;
		l_msgJointsCurrent.name[4] ="l_wrist_joint";
		l_msgJointsCurrent.position[4] = 0.0;

		// Publish the current joint states
		pubJoints = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

	}
	//*************************************************************************************
	void rob_control::mainLoop()
	{
  		ROS_INFO("Starting Mover Main Loop");
		
		//output angle data to a txt file
		int i = 0;
  		
 	 	for(;;)
  		{
			MotionGeneration();			// Generate the joint motion 
			CommunicationHW();			// Forward the new setpoints to the hardware

			if(flag_stop_requested)
				break;

			ros::spinOnce();
			ros::Duration(cycleTime/1000.0).sleep();		// main loop with 20 Hz.(50/1000=0.05s=50ms)
						
  		}

		ROS_INFO("Closing Mover Main Loop");

	} 
	//************************************************************************************
	//
	void rob_control::MotionGeneration()
	{
		int i=0;
		int j=0;

		//printf("r_targetPointList.size() = %ld \n",r_targetPointList.size());
       
		if(r_targetPointList.size() > 0)
		{
			r_targetState = r_targetPointList.front();
			r_targetPointList.pop_front();

			for(int i=0; i<nrOfJoints; i++)
			{
				r_setPointState.j[i] = r_targetState.j[i];
			}
			r_setPointState.duration = r_targetState.duration;
			
			for(j=0;j<5;j++)
			{
				r_controlData[j] = r_setPointState.j[j];
			}
			r_controlData[5] = r_setPointState.duration;
		}

		if(l_targetPointList.size() > 0)
		{

			l_targetState = l_targetPointList.front();
			l_targetPointList.pop_front();

			for(int i=0; i<nrOfJoints; i++)
			{
				l_setPointState.j[i] = l_targetState.j[i];
			}
			l_setPointState.duration = l_targetState.duration;
			
			for(j=0;j<5;j++)
			{
				l_controlData[j] = l_setPointState.j[j];
			}
			l_controlData[5] = l_setPointState.duration;
		}

	}
	//************************************************************************************
	// Forward the new setpoints to the hardware

	void rob_control::CommunicationHW()
	{

		int i = 0;
		for(i=0;i<6;i++)//write right arm control data
		{
			r_executeData[i] = r_controlData[i];
			l_executeData[i] = l_controlData[i];
		}

		if(r_executeData[0]!=r_executeData_old[0] && r_executeData[1]!=r_executeData_old[1])
		{
			
			//add send mesage command here	
			//************************************

			Comm.SetRightJoints(r_executeData);//send new target angle 

			//************************************

			ROS_INFO("Right arm MSG [%f][%f][%f][%f][%f][%f]", r_executeData[0],r_executeData[1],r_executeData[2],r_executeData[3],r_executeData[4],r_executeData[5]);

			//save the date to old
			for (int i=0;i<6;i++)
			{
				r_executeData_old[i] = r_executeData[i];
			}

			R_CommunicationROS();			// Publish the joint states and error info
		}

		if(l_executeData[0]!=l_executeData_old[0] && l_executeData[1]!=l_executeData_old[1])
		{
			//add send mesage command here		
			//************************************

			Comm.SetLeftJoints(l_executeData);//send new target angle 

			//************************************


			ROS_INFO("Left arm MSG [%f][%f][%f][%f][%f][%f]", l_executeData[0],l_executeData[1],l_executeData[2],l_executeData[3],l_executeData[4],l_executeData[5]);

			//save the date to old
			for (int i=0;i<6;i++)
			{
				l_executeData_old[i] = l_executeData[i];
			}

			L_CommunicationROS();			// Publish the joint states and error info
		}
	}
	//************************************************************************************
	// forward the current joints to RViz etc
	void rob_control::L_CommunicationROS()
	{
		l_msgJointsCurrent.header.stamp = ros::Time::now();
		l_msgJointsCurrent.position[0] = l_executeData[0]*deg2rad;		// Robot SER communication works in degree
		l_msgJointsCurrent.position[1] = l_executeData[1]*deg2rad;
		l_msgJointsCurrent.position[2] = l_executeData[2]*deg2rad;
		l_msgJointsCurrent.position[3] = l_executeData[3]*deg2rad;
        l_msgJointsCurrent.position[4] = l_executeData[4]*deg2rad;

		pubJoints.publish(l_msgJointsCurrent);	// ROS communication works in Radian

	}
	//************************************************************************************
	// forward the current joints to RViz etc
	void rob_control::R_CommunicationROS()
	{
		r_msgJointsCurrent.header.stamp = ros::Time::now();
		r_msgJointsCurrent.position[0] = r_executeData[0]*deg2rad;		// Robot SER communication works in degree
		r_msgJointsCurrent.position[1] = r_executeData[1]*deg2rad;
		r_msgJointsCurrent.position[2] = r_executeData[2]*deg2rad;
		r_msgJointsCurrent.position[3] = r_executeData[3]*deg2rad;
        r_msgJointsCurrent.position[4] = r_executeData[4]*deg2rad;

		pubJoints.publish(r_msgJointsCurrent);	// ROS communication works in Radian

	}

}

