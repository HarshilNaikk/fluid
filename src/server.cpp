#include <ascend_msgs/FluidGoal.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>

#include <assert.h>
#include <algorithm>
#include <cmath>

#include "core.h"
#include "server.h"
#include "util.h"

Server::Server() : actionlib_server_(node_handle_, "fluid_operation", false) {
    actionlib_server_.registerPreemptCallback(boost::bind(&Server::preemptCallback, this));
    actionlib_server_.start();
}

std::shared_ptr<Operation> Server::retrieveNewOperation() {
    // We accept the new goal and initialize variables for target pose and the type of operation identifier.
    // This is necessary in order to modify some of them before we initiate the different operations further down.
    // E.g. the init operation shouldn't be called with a different pose than (0, 0, 0), so we make sure this is the
    // case.

    if (!actionlib_server_.isNewGoalAvailable()) {
        return nullptr;
    }

    if (actionlib_server_.isActive()) {
        actionlib_server_.setPreempted();
    }

    auto goal = actionlib_server_.acceptNewGoal();
    std::vector<geometry_msgs::Point> path = goal->path;

    if (path.empty()) {
        path.push_back(Core::getGraphPtr()->current_state_ptr->getCurrentPose().pose.position);
    }

    Core::getStatusPublisherPtr()->status.path = path;

    StateIdentifier state_identifier = StateIdentifier::Null;

    auto result = std::find_if(StateIdentifierStringMap.begin(), StateIdentifierStringMap.end(),
                               [goal](const auto& entry) { return entry.second == goal->state; });

    if (result != StateIdentifierStringMap.end()) {
        state_identifier = result->first;
    } else {
        ROS_FATAL_STREAM("Could not find the state (" << goal->state << ") passed, did you spell it correctly?");

        actionlib_server_.setAborted();
        return nullptr;
    }

    return std::make_shared<Operation>(state_identifier, path);
}

void Server::preemptCallback() { actionlib_server_.setPreempted(); }

void Server::start() {
    ros::Rate rate(Core::refresh_rate);

    std::shared_ptr<Operation> current_operation_ptr;
    std::shared_ptr<State> last_state_ptr;

    while (ros::ok()) {
        if (current_operation_ptr) {
            Core::getStatusPublisherPtr()->status.current_operation =
                StateIdentifierStringMap.at(current_operation_ptr->getDestinationStateIdentifier());

            current_operation_ptr->perform(

                [&]() -> bool {
                    ascend_msgs::FluidFeedback feedback;
                    std::shared_ptr<State> current_state_ptr = Core::getGraphPtr()->current_state_ptr;
                    feedback.pose_stamped = current_state_ptr->getCurrentPose();
                    feedback.state = StateIdentifierStringMap.at(current_state_ptr->identifier);
                    actionlib_server_.publishFeedback(feedback);

                    return !actionlib_server_.isPreemptRequested() && ros::ok();
                },

                [&](bool completed) {
                    // We completed the operation and want to end at the final state of the operation (e.g. hold)
                    // state for move. One can think of this step as making sure that the state machine is at a
                    // state where it's easy to execute a new operation. If we did not complete the operation has
                    // already transitioned to a steady state and we just set the last state to that state.
                    if (completed) {
                        last_state_ptr = current_operation_ptr->getFinalStatePtr();
                    } else {
                        last_state_ptr = Core::getGraphPtr()->current_state_ptr;
                    }

                    // Will notify the operation client what the outcome of the operation was. This will end up
                    // calling the callback that the operation client set up for completion.

                    if (!actionlib_server_.isActive()) {
                        return;
                    }

                    ascend_msgs::FluidResult result;

                    Core::getStatusPublisherPtr()->status.path.clear();

                    if (completed) {
                        ROS_INFO_STREAM("Operation completed.");
                        result.pose_stamped = Core::getGraphPtr()->current_state_ptr->getCurrentPose();
                        result.state = StateIdentifierStringMap.at(last_state_ptr->identifier);
                        actionlib_server_.setSucceeded(result);
                    } else {
                        ROS_INFO_STREAM("Operation cancelled.");
                        result.pose_stamped = Core::getGraphPtr()->current_state_ptr->getCurrentPose();
                        result.state = StateIdentifierStringMap.at(last_state_ptr->identifier);
                        actionlib_server_.setPreempted(result);
                    }
                });

            current_operation_ptr.reset();
        }
        // We don't have a current operation, so we just continue executing the last state.
        else {
            Core::getStatusPublisherPtr()->status.current_operation = "none";

            if (last_state_ptr) {
                if (last_state_ptr->identifier == StateIdentifier::Init) {
                    Core::getStatusPublisherPtr()->status.current_state = "none";
                } else {
                    last_state_ptr->perform(
                        [&]() -> bool {
                            // We abort the execution of the current state if there is a new operation.
                            return !actionlib_server_.isNewGoalAvailable();
                        },
                        true);
                }
            }
        }

        Core::getStatusPublisherPtr()->publish();

        // Setup for the new operation.
        if (actionlib_server_.isNewGoalAvailable() && !actionlib_server_.isActive()) {
            current_operation_ptr = retrieveNewOperation();
        }

        ros::spinOnce();
        rate.sleep();
    }
}
