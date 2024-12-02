#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ir2324_group_24/TiagoAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<ir2324_group_24::TiagoAction> Action_Server; // alias for the Action Server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // alias for the Action Server

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

int main (int argc, char** argv){
	// create the Node_B that acts as a Server for the Node_A
	ros::init(argc, argv, "Node_B");

	// constructor
	TiagoAction tiago("Node_B");

	// create a node "simple_navigation_goals" that acts as a Client to send requests to the move_base sever
	ros::init(argc,argv,"simple_navigation_goals");
	MoveBaseClient ac_("move_base",true);

	// wait for the move_base action server to come up
	while(!ac_.waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the move_base action server to come up");

	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;
	ROS_INFO("Sending goal");
	ac_.sendGoal(goal);
	ac_.waitForResult();
	if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	ROS_INFO("Hooray, the base moved 1 meter forward");
	else
	ROS_INFO("The base failed to move forward 1 meter for some reason");
	return 0;
}
