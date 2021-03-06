/**
 * @file extract_module_operation.h
 */

#ifndef EXTRACT_MODULE_OPERATION_H
#define EXTRACT_MODULE_OPERATION_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "operation.h"
#include "operation_identifier.h"

/**
 * @brief Represents the operation where the drone is extracting the module.
 */
class ExtractModuleOperation : public Operation {
   private:
	
	enum class ModuleState {
		APPROACHING,
		OVER,
		BEHIND_WITH_HOOKS,
	 	EXTRACTING,
		EXTRACTED
	};

	ModuleState module_state = ModuleState::APPROACHING;

    const float speed = 0.3;

    geometry_msgs::PoseWithCovarianceStamped module_pose;

    ros::Subscriber module_pose_subscriber;

    void modulePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose);

    ros::ServiceClient backpropeller_client;

    bool called_backpropeller_service = false;

   public:
    /**
     * @brief Sets up the subscriber for the module pose.
     */
    explicit ExtractModuleOperation();

    /**
     * @brief Sets up #speed at which to move.
     */
    void initialize() override;

    /**
     * @return true When the module has been extracted.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Makes sure the drone is following the module and reacting to the extraction signal.
     */
    void tick() override;
};

#endif
