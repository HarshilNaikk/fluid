#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "rotate_state.h"
#include "util.h"
#include "core.h"

bool fluid::RotateState::hasFinishedExecution() const {
    bool atPositionTarget = Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) < fluid::Core::distance_completion_threshold && 
    	   				 	std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold && 
    	   					std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;

    bool atYawTarget = std::abs(Util::angleBetween(getCurrentPose().pose.orientation, setpoint.yaw)) < fluid::Core::yaw_completion_threshold; 

    return atYawTarget && atPositionTarget;
}

void fluid::RotateState::initialize() {
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = getCurrentPose().pose.position.z;
    setpoint.type_mask = TypeMask::Position;

    double dx = path.front().x - getCurrentPose().pose.position.x;
    double dy = path.front().y - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);
}
