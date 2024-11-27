#include "ros/ros.h"
#include "tiago_iaslab_simulation/Objs.h"  // Include the generated service header for Objs.srv

// Service client function
void request_apriltags_ids()
{
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
        {
            ROS_INFO("ID: %d", id);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /apriltag_ids_srv");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltags_ids_client");
    request_apriltags_ids();  // Call the function to make the service request
    return 0;
}

