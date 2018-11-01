//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_MOVE_STATE_H
#define FLUID_FSM_MOVE_STATE_H

#include "../mavros/mavros_state.h"
#include <ros/ros.h>

namespace fluid {

    /** \class MoveState
     *  \brief Represents the state where the drone is moving from a to b.
     */
    class MoveState: public MavrosState {

    public:

        /** Initializes the move state.
         */
        explicit MoveState(ros::NodeHandlePtr node_handle_p) : MavrosState(node_handle_p, "move") {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution() override;

        /**
         * Overridden function. @see State::tick
         */
        void tick() override;
    };
}

#endif //FLUID_FSM_MOVE_STATE_H
