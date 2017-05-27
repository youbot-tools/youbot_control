/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include <iostream>
#include <assert.h>
#include <unistd.h> 
#include<fstream>
#include<string>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_test");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

// the v:
    ros::Publisher armVelocitiesPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

// the v:
    armVelocitiesPublisher = n.advertise<brics_actuator::JointVelocities > ("arm_1/arm_controller/velocitie_command", 1);


	ros::Rate rate(20); //Hz
    double step = 0.001;
	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) {
        brics_actuator::JointPositions command;

 // the v:
        brics_actuator::JointVelocities command_v;

		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

// the v:
        vector <brics_actuator::JointValue> armJointVelocities;
        armJointVelocities.resize(numberOfArmJoints);

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;

		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();

        //ifstream in("main.txt");
		string s;
        double angle_con[7];
		int i = 0;
        ifstream in("main.txt");
		while(getline(in,s))
		{
        sscanf(s.c_str(),"%lf",&angle_con[i]);
		i++;
		//cout<<a[i]<<endl;
		//cout<<endl;
		}
        sleep(0.3);
        for (int j = 0; j < 5; ++j)
		{				
//			cout << "Please type in value for joint " << i + 1 << endl;
//			cin >> readValue;
//
			jointName.str("");
            jointName << "arm_joint_" << (j + 1);
//
            armJointPositions[j].joint_uri = jointName.str();
            armJointPositions[j].value = angle_con[j];

            armJointPositions[j].unit = boost::units::to_string(boost::units::si::radians);
            cout << "Joint " << armJointPositions[j].joint_uri << " = " << armJointPositions[j].value << " " << armJointPositions[j].unit << endl;

		};

 // the v:
        string sv;
         ifstream file("vel.txt");
         int k = 0;
         double velo_con[5];
        while(getline(file, sv))
        {
            sscanf(sv.c_str(),"%lf",&velo_con[k]);
            k++;
        }
        sleep(0.2);
        for(int k = 0; k < 5; ++k)
        {
            jointName.str("");
            jointName << "arm_joint_" << (k + 1);
            armJointVelocities[k].joint_uri = jointName.str();
            armJointVelocities[k].value = angle_con[k];
            armJointVelocities[k].unit = boost::units::to_string(boost::units::si::radians);
        }

//		cout << "Please type in value for a left jaw of the gripper " << endl;
//		cin >> readValue;
        gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
        gripperJointPositions[0].value = angle_con[5];
        gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

//		cout << "Please type in value for a right jaw of the gripper " << endl;
//		cin >> readValue;
       gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
       gripperJointPositions[1].value =angle_con[6];
       gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

		cout << "sending command ..." << endl;
        cout<<gripperJointPositions[0].value<<":  "<<gripperJointPositions[1].value<<endl;

        command.positions = armJointPositions;

// the v:
        command_v.velocities = armJointVelocities;
        armVelocitiesPublisher.publish(command_v);

        armPositionsPublisher.publish(command);


        if (angle_con[5] != step){
                command.positions = gripperJointPositions;
                gripperPositionPublisher.publish(command);
                step = angle_con[5];
        }



		cout << "--------------------" << endl;
        ros::spinOnce();
		rate.sleep();

	}

	return 0;
}

/* EOF */
