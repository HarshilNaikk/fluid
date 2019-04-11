#include <utility>

//
// Created by simengangstad on 04.10.18.
//

#ifndef FLUID_FSM_OPERATION_H
#define FLUID_FSM_OPERATION_H

#include <memory>
#include <vector>
#include <string>
#include "../state.h"
#include "../transition.h"
#include "state_graph.h"
#include <mavros_msgs/PositionTarget.h>
#include "../core.h"

namespace fluid {

    /** \class Operation
     *  \brief Manages the transitions between multiple states and their execution.
     */
    class Operation {
    private:

        const std::string destination_state_identifier_;                        ///< The state the operation should
                                                                                ///< transition to.

        const std::string final_state_identifier_;                              ///< The state the operation should
                                                                                ///< transition to after it has carried
                                                                                ///< out the logic in the destination
                                                                                ///< state. E.g. if destination state
                                                                                ///< is set to a move state for a move
                                                                                ///< operation, we want the operation
                                                                                ///< to finish at a position hold
                                                                                ///< state.

    public:

        mavros_msgs::PositionTarget position_target;                ///< Position target of the operation.

        const std::string identifier;                               ///< Identifier of the operation.

        /**
         * Sets up the operation with a destination state and a final state. The difference between them is that the
         * destination state is the state the operation will transition to and carry out logic on, whereas final state
         * is the state we want to be at after the operation. E.g. a move operation would want to be at a final state
         * of position hold after a given move state.
         *
         * @param identifier                     The identifier of the operation.
         * @param destination_state_identifier   The destination state identifier of the operation.
         * @param final_state_identifier         The final state identifier.
         * @param position_target                The target position of this operation.
         */
        Operation(std::string identifier,
                  std::string destination_state_identifier,
                  std::string final_state_identifier,
                  mavros_msgs::PositionTarget position_target);


        /**
         * Checks if the operation is valid from the current state. 
         * 
         * This makes sure that some operations are not
         * carried out given that they make no sense from the current state. E.g. doing any operation before
         * everything is initialized or doing a land operation during take off.
         * 
         * This is overridden by subclasses which provide the logic given the current state and what is reasonable
         * for the given operaiton. 
         *
         * @param current_state_p       The current state.
         *
         * @return A flag determining the validation of the operation given the current state.
         */
        virtual bool validateOperationFromCurrentState(std::shared_ptr<fluid::State> current_state_p) = 0;

        /** 
         * Performs the operation.
         *
         * Runs through the different states and performs the necessary transitions.
         *
         * @param shouldAbort Called each tick, makes it possible to abort operations in the midst of an execution.
         * @param completionHandler Callback function for whether the operation completed or not.
         */
        void perform(std::function<bool (void)> shouldAbort, std::function<void (bool)> completionHandler);

        /**
         * @brief Sets the pose for a new state and performs the transition to that state from the current state.
         * 
         * This function also sets the current state of the state graph to the state passed as the argument.
         */
        void transitionToState(std::shared_ptr<fluid::State> state_p);

        /**
         * @return The state the operation should end at.
         */
        std::shared_ptr<fluid::State> getFinalStatePtr();

        /**
         * @return The current state of the operation.
         */
        std::shared_ptr<fluid::State> getCurrentStatePtr();
    };
}

#endif //FLUID_FSM_OPERATION_H
