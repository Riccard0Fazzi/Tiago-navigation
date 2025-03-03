#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"  // Include the generated service header for Objs.srv
#include <actionlib/client/simple_action_client.h>
#include <ir2425_group_24/TiagoAction.h> // action file

typedef actionlib::SimpleActionClient<ir2425_group_24::TiagoAction> Action_Client; // alias for the Action Client

// Feedback callback function to print the feedbacks coming from Node_B
void feedbackCallback(const ir2425_group_24::TiagoFeedbackConstPtr &feedback) {
    ROS_INFO("[FEEDBACK] %s", feedback->robot_status.c_str());
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
	if (!client.call(srv))
	{
		ROS_ERROR("Failed to call service /apriltag_ids_srv");
	}

	// ------------------  Send the acquired ids to Node_B ------------------------
	
	// create a client object to communicate with the server Node_B
	Action_Client ac("Node_B",true);

	// wait for the node B to start
	ac.waitForServer();

	// defining the object to store the goal
	ir2425_group_24::TiagoGoal goal;

	// assign to the goal object the vector of apriltag ids
	goal.apriltag_ids = std::vector<int64_t>(srv.response.ids.begin(), srv.response.ids.end());

	

	// send the goal object to the Node_B server
	// the feedbackCallBack arg is the function to get the robot status from the server
	ac.sendGoal(goal, Action_Client::SimpleDoneCallback(), Action_Client::SimpleActiveCallback(), &feedbackCallback);
	
	// waiting for the server for the result
	ac.waitForResult();
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s", state.toString().c_str());

	// print all aprilTag IDs found and their poses wrt map
	// Get the result from Node_B
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ir2425_group_24::TiagoResultConstPtr result_ = ac.getResult();

		// Print header for clarity
		std::cout << std::setw(12) << "AprilTag ID"
				<< std::setw(50) << "Position (x, y)"
				<< std::endl;

		std::cout << std::string(100, '-') << std::endl; // Divider line

		// Iterate through the vector of AprilTag poses and print each pose
		for (const auto& pose : result_->aprilTags_poses) {
			const auto& id = pose.header.seq;

			// Extract position and orientation
			const auto& position = pose.pose.position;
			const auto& orientation = pose.pose.orientation;

			// Print the pose details with alignment
			std::cout << std::setw(12) << id
					<< std::setw(50) 
					<< "(" + std::to_string(position.x) + ", " 
					+ std::to_string(position.y) + ")" 
					<< std::endl;
		}
	}

}

