#!/usr/bin/env python3

import rospy
import math

import ascend_msgs.msg
#from ascend_msgs.msg import *
from fluid.srv import *
from geometry_msgs.msg import Point

# Declare the operations we can perform
take_off = rospy.ServiceProxy('fluid/take_off', TakeOff)
explore = rospy.ServiceProxy('fluid/explore', Explore)
travel = rospy.ServiceProxy('fluid/travel', Travel)
land = rospy.ServiceProxy('fluid/land', Land)
extract_module = rospy.ServiceProxy('fluid/extract_module', ExtractModule)

finished_operation = ""
is_executing_operation = False


def fluidOperationCompletionCallback(request):
    """
    Grab the finished operation when it completes. This is just a service callback that fluid will
    call to. It's not mandatory to implement this service callback, the result of the callback
    does not affect the internal state of Fluid, but in that case the client will of course not know when an operation
    completes. In other words, just leave this here to have a two-way communication :)

    Arguments:
        request {OperationCompletionRequest} -- Holds the operation completed.

    Returns:
        OperationCompletionResponse -- Just an empty struct for the service.
    """

    global finished_operation, is_executing_operation
    print("Finished operation " + request.operation)
    finished_operation = request.operation
    is_executing_operation = False
    return OperationCompletionResponse()


def gotConnectionWithServices(timeout):
    """Make sure that every operation we are going to call are up and running.

    Arguments:
        timout {Int} -- The amount of time to wait for the services. 

    Returns:
        Bool -- Whether we got connection with the services. 
    """

    try:
        rospy.wait_for_service('fluid/take_off', timeout=timeout)
        rospy.wait_for_service('fluid/explore', timeout=timeout)
        rospy.wait_for_service('fluid/travel', timeout=timeout)
        rospy.wait_for_service('fluid/land', timeout=timeout)
        rospy.wait_for_service('fluid/extract_module', timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def main():
    global is_executing_operation
    rospy.init_node('fluid_client')

    # Don't need the service server, we only need to set up the callback, so leave it blank
    _ = rospy.Service("fluid/operation_completion", OperationCompletion, fluidOperationCompletionCallback)

    # Wait until we get connection with the services
    while not gotConnectionWithServices(2) and not rospy.is_shutdown():
        rospy.logerr("Did not get connection with Fluid's services, is Fluid running?")

    # Perform a take off to 3 meters above ground
    take_off_response = take_off(2)
    is_executing_operation = True

    # Checks if the operation request was successful
    # E.g. a travel operation is not allowed before the drone has taken off
    # It's important to check the response in order to keep the clients logic
    # in sync with the Fluid. If the request was not successful, Fluid will just keep
    # executing its current state
    if (not take_off_response.success):
        rospy.logerr(take_off_response.message)

    while not rospy.is_shutdown():

        # Do some more operations when take off has finished
        # This structure is just an example, the logic for when an operation
        # is executing should probably be implemented differently
        if not is_executing_operation:
            if finished_operation == "TAKE_OFF":

                # Perform a explore with a (list of) point(s)
                point = Point()
                point.x = 5
                response = explore([point])
                if (not response.success):
                    rospy.logerr(response.message)
                else:
                    is_executing_operation = True
            elif finished_operation == "EXPLORE":

                # Perform a travel with a list of points
                point = Point()
                point.x = 100
                response = travel([point])
                if (not response.success):
                    rospy.logerr(response.message)
                else:
                    is_executing_operation = True
            elif finished_operation == "TRAVEL":

                # Perform a land
                response = land()
                if (not response.success):
                    rospy.logerr(response.message)
                else:
                    is_executing_operation = True


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as exception:
        print("Program interrupted: " + str(exception))
