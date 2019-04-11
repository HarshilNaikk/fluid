//
// Created by simengangstad on 11.10.18.
//

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "../../include/states/move_state.h"
#include "../../include/tools/pose_util.h"
#include "../../include/mavros/type_mask.h"

bool fluid::MoveState::hasFinishedExecution() {
    bool atPositionTarget = PoseUtil::distanceBetween(current_pose_, position_target) < 0.3 && 
    	   				 	std::abs(getCurrentTwist().twist.linear.x) < 0.1 && 
    	   					std::abs(getCurrentTwist().twist.linear.y) < 0.1 && 
    	   					std::abs(getCurrentTwist().twist.linear.z) < 0.1;

    tf2::Quaternion quat(getCurrentPose().pose.orientation.x, 
                         getCurrentPose().pose.orientation.y, 
                         getCurrentPose().pose.orientation.z, 
                         getCurrentPose().pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set 
    // it to zero. 
    yaw = std::isnan(yaw) ? 0.0 : yaw;

    bool atYawTarget = std::abs(position_target.yaw - yaw) < 0.3; 

    return atYawTarget && atPositionTarget;
}

void fluid::MoveState::tick() {
    position_target.type_mask = fluid::TypeMask::Default;
}