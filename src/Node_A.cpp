#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"  // Include the generated service header for Objs.srv
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/tiagoAction.h> // action file

typedef actionlib::SimpleActionClient<ir2324_group_24::tiagoAction> Action_Client; // alias

// Feedback callback function for the status of the robot
void feedbackCallback(const ir2324_group_24::tiagoFeedbackConstPtr &feedback) {
    ROS_INFO("Current robot status: %s", feedback->robot_status.c_str());
}

int main(int argc, char **argv)
{
	// to initialize the Client node for the ids_generator_node
	ros::init(argc, argv, "apriltags_ids_client");
	ros::NodeHandle nh;

	// Create a service client to call the service '/apriltag_ids_srv'
	ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs> ("/apriltag_ids_srv");

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

	// ------------------  Send the acquired ids to Node_B ------------------------
	
	// create a client object to communicate with the server Node_B
	Action_Client ac("Node_B",true);

	// wait for the node B to start
	ROS_INFO("Waiting for the Node_B to start");
	ac.waitForServer();
	ROS_INFO("Node_B server started, sending goal");

	// defining the object to store the goal
	ir2324_group_24::tiagoGoal goal;

	// assign to the goal object the vector of apriltag ids
	goal.apriltag_ids.assign(srv.response.ids.begin(),srv.response.ids.end());

	// send the goal object to the Node_B server
	// the feedbackCallBack arg is the function to get the robot status from the server
	ac.sendGoal(goal, Action_Client::SimpleDoneCallback(),Action_Client::SimpleActiveCallback(), &feedbackCallback);
	
	// waiting for the server for the result
	ac.waitForResult();
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s", state.toString().c_str());
	
	return 0;
}

