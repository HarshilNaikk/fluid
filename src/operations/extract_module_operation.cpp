/**
 * @file extract_module_operation.cpp
 */
#include "extract_module_operation.h"

#include "mavros_interface.h"
#include "util.h"

#include <std_srvs/SetBool.h>

ExtractModuleOperation::ExtractModuleOperation() : Operation(OperationIdentifier::EXTRACT_MODULE, false) {
    module_pose_subscriber =
        node_handle.subscribe("/airsim/module_position", 10, &ExtractModuleOperation::modulePoseCallback, this);
    backpropeller_client = node_handle.serviceClient<std_srvs::SetBool>("/airsim/backpropeller");
}

void ExtractModuleOperation::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.setParam("MPC_XY_VEL_MAX", speed);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat speed to: " << speed);

    mavros_interface.setParam("MPC_TILTMAX_AIR", 20);
    mavros_interface.setParam("MPC_Z_VEL_MAX_DN", 0.5);

    // Use the current position as setpoint until we get a message with the module position
    setpoint.position = getCurrentPose().pose.position;
}

bool ExtractModuleOperation::hasFinishedExecution() const { return module_state == ModuleState::EXTRACTED; }

void ExtractModuleOperation::modulePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr) {
    module_pose = *module_pose_ptr; //TODO: Theo Question: wouldn't it be some potential problems here? another position is broadcasted, then module_pose changes value without calling the function modulePoseCallback ??
}

void ExtractModuleOperation::tick() {//TODO: Theo Question: I am not sure i understand the name of the function. Could it be more explicit?
    setpoint.type_mask = TypeMask::POSITION;

    // Wait until we get the first module position readings before we do anything else.
    if (module_pose.header.seq == 0) {
        return;
    }

    const double dx = module_pose.pose.pose.position.y - getCurrentPose().pose.position.x; //Theo Question: Ahaha, wtf, shouldn't we try to get a nicer structure? pose.pose.pose.pose.pose
    const double dy = module_pose.pose.pose.position.x - getCurrentPose().pose.position.y; //Theo: to make it more readable and easier, we coult create a pointer to module_pose.pose.pose.position?
    //TODO: Theo question: why don't we care about z?

    setpoint.yaw = std::atan2(dy, dx) - M_PI / 18.0;

    const double distance_to_module = sqrt(dx * dx + dy * dy);

    const double dvx = getCurrentTwist().twist.linear.x; //Theo question: why is it called Twist?
    const double dvy = getCurrentTwist().twist.linear.y;
    const double dvz = getCurrentTwist().twist.linear.z;

    const double speed = sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

    switch (module_state) {
        //TODO: Theo: In general, it is not nice to have raw numbers the one added to the module position in all thoses cases.
        //The setpoint could be defined elsewhere and make the code smaller and more readable.

        case ModuleState::APPROACHING: { //TODO: Theo: as I see it, we are changing the setpoint at each iteration. It is subject to some weird comportement of the drone when it will approch the setpoint
        // (Ok, the position of the module should be stabilized by the perception group and we are using it as a precise reference since we stop APPROACHING 30cm away from the setpoint)
            setpoint.position.x = module_pose.pose.pose.position.y;
            // TODO: This has to be fixed, should be facing towards the module from any given position,
            // not just from the x direction
            setpoint.position.y = module_pose.pose.pose.position.x + 1.5;
            setpoint.position.z = module_pose.pose.pose.position.z;

            if (distance_to_module < 1.8) { //TODO: Theo: Here, the drone could be aside but not in front again. Shouldn't the distance to the setpoint be checked instead?
                module_state = ModuleState::OVER;
            }

            break;
        }
        case ModuleState::OVER: { //TODO: Theo: what is the referential/frame here? it may be the direciton that the mast is facing to take into account?
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 0.78;
            setpoint.position.z = module_pose.pose.pose.position.z + 0.3;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.1 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                //TODO: Theo: We may also want to check that the speed of the drone is close to the speed of the module as we don't want the module to push the drone away.
                module_state = ModuleState::BEHIND_WITH_HOOKS;
            }

            break;
        }
        case ModuleState::BEHIND_WITH_HOOKS: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 0.78;
            setpoint.position.z = module_pose.pose.pose.position.z - 0.1;//TODO: Theo: here we may want a variable or a define to be able to easily thune how much the drone go down to the hooks. It is an important setting as we don't want the module to carryt he drone, but we want the drone to be low enough

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.05 && getCurrentTwist().twist.linear.z < 0.03 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                module_state = ModuleState::EXTRACTING;
            }

            break;
        }
        case ModuleState::EXTRACTING: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 2.0;
            setpoint.position.z = module_pose.pose.pose.position.z - 0.1;

            if (!called_backpropeller_service) {
                std_srvs::SetBool request;
                request.request.data = true;
                backpropeller_client.call(request);  //May be wise to ask for the good direction as well. We may want to use the backpropeller both way.
                called_backpropeller_service = true;
            }           

            // If the module is on the way down
            // TODO: this should be checked in a better way
            //TODO: Theo, indeed, we have sensors for that in the hooks.
            if (module_pose.pose.pose.position.z < 0.5) {
                module_state = ModuleState::EXTRACTED;
                std_srvs::SetBool request;
                request.request.data = false;
                backpropeller_client.call(request);
                called_backpropeller_service = false;
            }

            break;
        }
    }
}
