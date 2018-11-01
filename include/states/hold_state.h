//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_HOLD_STATE_H
#define FLUID_FSM_HOLD_STATE_H

#include <ros/ros.h>
#include "../core/state.h"
#include "../mavros/mavros_state.h"

namespace fluid {

    /** \class HoldState
     *  \brief Represents the state when the drone is hovering at a certain altitude
     */
    class HoldState: public MavrosState {

    public:

        /**
         * Initializes the hold state.
         */
        explicit HoldState(ros::NodeHandlePtr node_handle_p) : MavrosState(node_handle_p, "hold") {}

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

#endif //FLUID_FSM_HOLD_STATE_H
