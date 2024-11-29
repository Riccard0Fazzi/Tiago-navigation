#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"  // Include the generated service header for Objs.srv
#include <actionlib/client/simple_action_client.h>

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
	actionlib::SimpleActionClient<ir2324_24::test> ac("Node_A",true);
	ROS_INFO("Waiting:");
    return 0;
}

