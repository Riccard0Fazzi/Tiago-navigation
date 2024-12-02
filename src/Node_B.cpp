#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ir2324_group_24/TiagoAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>  // for the laser scanner data
#include <nav_msgs/OccupancyGrid.h> // for the local costmap data
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

typedef actionlib::SimpleActionServer<ir2324_group_24::TiagoAction> Action_Server; // alias for the Action Server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // alias for the Action Server
// Global variable to store the costmap data
nav_msgs::OccupancyGrid::ConstPtr global_costmap;
// Costmap callback to store the costmap data
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
    global_costmap = costmap;  // Store the costmap for later use
}
// callBack that gives us information for when the robot is moving
void MoveBaseCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("Current robot position: [%.2f, %.2f]",
             feedback->base_position.pose.position.x,
             feedback->base_position.pose.position.y);
}

geometry_msgs::PoseStamped transformToMapFrame(const geometry_msgs::PoseStamped& pose_in_base, tf2_ros::Buffer& tf_buffer) {
    geometry_msgs::PoseStamped pose_in_map;
    try {
        pose_in_map = tf_buffer.transform(pose_in_base, "map", ros::Duration(1.0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Could not transform pose to map frame: %s", ex.what());
    }
    return pose_in_map;
}

// callBack to obtain the laser scan readings
/*
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max) {
            ROS_INFO("Obstacle detected at angle: %.2f, range: %.2f", angle, range);
        }
    }
}
*/
/*
// callBack to obtain the data in the grid of the local costmap
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
    int width = costmap->info.width;
    int height = costmap->info.height;
    double resolution = costmap->info.resolution;
    // Example: Analyzing the center cell of the costmap
    int center_index = (width / 2) + (height / 2) * width;
    int cost = costmap->data[center_index];
    if (cost == 0) {
        ROS_INFO("Center cell is free.");
    } else if (cost > 0) {
        ROS_WARN("Center cell has obstacles.");
    }
}

*/

class TiagoAction{
	// Class variables 
	protected:
		ros::NodeHandle nh_;
		Action_Server as_;
		ir2324_group_24::TiagoFeedback feedback_;
		ir2324_group_24::TiagoResult result_;
		std::string action_name_;
		std::string tiago_status;
		std::vector<int> goal_;
	
	public:
		// Constructor 
		TiagoAction(std::string name):as_(nh_, name, boost::bind(&TiagoAction::TiagoCB, this, _1), false), action_name_(name){
			as_.start();
			tiago_status = "Tiago ready to navigate";
			// Call Navigation after processing the goal
		}

		// Destructor
		~TiagoAction(void){}
		// Status Callback
	void TiagoCB(const ir2324_group_24::TiagoGoalConstPtr &goal) {
		ros::Rate r(1);
		// Clear goal_ before populating to handle multiple goals
		goal_.clear();
		ROS_INFO("Received AprilTag IDs:");
		for (int i = 0; i < goal->apriltag_ids.size(); i++) {
			goal_.push_back(goal->apriltag_ids[i]);
			ROS_INFO("ID: %d", goal->apriltag_ids[i]);
		}
		feedback_.robot_status = tiago_status;
		as_.publishFeedback(feedback_);
		// Simulate processing
		r.sleep();
		as_.setSucceeded();
	}
};

geometry_msgs::PoseStamped computeNextGoal() {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "base_link";
    goal.header.stamp = ros::Time::now();
    if (!global_costmap) {
        ROS_WARN("No costmap data available. Defaulting to straight ahead.");
        goal.pose.position.x = 1.0;  // Default: 1 meter straight ahead
        goal.pose.position.y = 0.0;
        goal.pose.orientation.w = 1.0;
        return goal;
    }

    // Analyze the costmap
    int width = global_costmap->info.width;
    int height = global_costmap->info.height;
    double resolution = global_costmap->info.resolution;
    int center_x = width / 2;
    int center_y = height / 2;
    // Example: Find the nearest free cell ahead of the robot
    for (int i = center_x; i < width; ++i) {  // Scan rows ahead of the robot
        int index = center_y * width + i;  // Row-major index
        if (global_costmap->data[index] == 0) {  // Check for free space
            // Convert grid cell to world coordinates
            double world_x = global_costmap->info.origin.position.x + i * resolution;
            double world_y = global_costmap->info.origin.position.y + center_y * resolution;
            // Set the goal to this free cell
            goal.pose.position.x = world_x;
            goal.pose.position.y = world_y;
            goal.pose.orientation.w = 1.0;  // Keep orientation facing forward
            ROS_INFO("Goal computed: (%.2f, %.2f)", world_x, world_y);
            return goal;
        }
    }

    // Default fallback if no free cell is found
    ROS_WARN("No free cell found. Defaulting to straight ahead.");
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.0;
    goal.pose.orientation.w = 1.0;
    return goal;
}



int main (int argc, char** argv){
	// create the Node_B that acts as a Server for the Node_A
	ros::init(argc, argv, "Node_B");
	ros::NodeHandle nh;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	// constructor
	TiagoAction tiago("Node_B");
	// create a node "simple_navigation_goals" that acts as a Client to send requests to the move_base sever
	// create a node simple_navigation_goals that acts as a Client to send requests to the move_base sever
//	ros::init(argc,argv,"simple_navigation_goals");

	MoveBaseClient ac("move_base",true);
	// wait for the move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the move_base action server to come up");
	ac.waitForServer();
	ros::Subscriber costmap_sub = nh.subscribe("/move_base/local_costmap/costmap", 10, &costmapCallback);
	ros::Rate rate(10);
	while (ros::ok()) {
		// Compute the next goal based on the costmap
		geometry_msgs::PoseStamped goal_in_base = computeNextGoal();
		// Send the goal to move_base
		// Transform the goal to the map frame
		geometry_msgs::PoseStamped goal_in_map = transformToMapFrame(goal_in_base, tf_buffer);
		if (goal_in_map.header.frame_id == "map") {
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose = goal_in_map;
			// Send the goal to move_base
			ac.sendGoal(goal, MoveBaseClient::SimpleDoneCallback(), MoveBaseClient::SimpleActiveCallback(), &MoveBaseCallback);
			ac.waitForResult();
			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
				ROS_INFO("Goal reached successfully!");
			else 
				ROS_WARN("Failed to reach the goal. Re-planning...");
			
		} else {
			ROS_WARN("Goal transformation failed, skipping...");
		}	
		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}
