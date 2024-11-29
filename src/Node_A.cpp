#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"  // Include the generated service header for Objs.srv
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/tempAction.h>

typedef actionlib::SimpleActionClient<ir2324_group_24::tempAction> Client

// Feedback callback function
void feedbackCallback(const ir2324_group_24::tempFeedbackConstPtr &feedback) {
    ROS_INFO("Current robot status: %s", feedback->robot_status.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apriltags_ids_client");
	ros::NodeHandle nh;

	// Create a service client to call the service '/apriltag_ids_srv'
	ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");

	// Create a service request
	tiago_iaslab_simulation::Objs srv;
	srv.request.ready = true;  // Indicate that the request is ready for processing

	// Call the service
	if (client.call(srv))
	{
		// Print the received IDs in the response
		ROS_INFO("Received Apriltag IDs: ");
		for (int id : srv.response.ids)
			ROS_INFO("ID: %d", id);
	}
	else ROS_ERROR("Failed to call service /apriltag_ids_srv");

	// Send the acquired ids to Node_B
	// test
	actionlib::SimpleActionClient<ir2324_group_24::tempAction> ac("Node_A",true);
	ROS_INFO("Waiting for the Node_B to start");
	ac.waitForServer();
	ROS_INFO("Node_B server started, sending goal");
	ir2324_group_24::tempGoal goal;
	goal.apriltag_ids = srv.response.ids;
	ac.sendGoal(goal, Client::SimpleDoneCallback(), Client::SimpleActiveCallback(), &feedbackCallback);
	bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));
	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
		// Check if the goal was successfully achieved
		if (ac.getResult()->positions) 
			ROS_INFO("Positions obtained successfully.");
	       	else 
			ROS_WARN("Error in retrieving positions.");

	}
       	else
		ROS_WARN("Action did not finish before the timeout.");

    return 0;
}

