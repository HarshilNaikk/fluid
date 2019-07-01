#include "core.h"


int fluid::Core::refresh_rate = 20;
int fluid::Core::message_queue_size = 10;
bool fluid::Core::auto_arm = false;
bool fluid::Core::auto_set_offboard = false;

std::shared_ptr<fluid::StateGraph> fluid::Core::graph_p_;
std::shared_ptr<fluid::StatusPublisher> fluid::Core::status_publisher_p_;

geometry_msgs::Point32 fluid::Core::min, fluid::Core::max;

double fluid::Core::distance_completion_threshold = 0.3;
double fluid::Core::velocity_completion_threshold = 0.1;
double fluid::Core::yaw_completion_threshold = 0.2;
double fluid::Core::default_height = 1.0;
double fluid::Core::positionFollowHeight = 2.3;

std::shared_ptr<fluid::StateGraph> fluid::Core::getGraphPtr() {
	if (!graph_p_) {
		graph_p_ = std::make_shared<fluid::StateGraph>();
	}

	return graph_p_;
}

std::shared_ptr<fluid::StatusPublisher> fluid::Core::getStatusPublisherPtr() {
	if (!status_publisher_p_) {
		status_publisher_p_ = std::make_shared<fluid::StatusPublisher>();
	}

	return status_publisher_p_;
}