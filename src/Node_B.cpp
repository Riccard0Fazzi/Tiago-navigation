#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2324_group_24/TiagoAction.h>

typedef actionlib::SimpleActionServer<ir2324_group_24::TiagoAction> Action_Server; // alias for the Action Server

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
		}

		// Destructor
		~TiagoAction(void){}

		// Status Callback
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

		// Call Navigation after processing the goal
		Navigation();

		// Set the action as succeeded
		as_.setSucceeded();
	}


	// Navigation 
	void Navigation() {
		ROS_INFO("Starting Navigation...");
		if (goal_.empty()) {
			ROS_WARN("No goals to navigate to!");
			return;
		}
		for (int i = 0; i < goal_.size(); i++) {
			ROS_INFO("Navigating to ID: %d", goal_[i]);
		}
	}
};

int main (int argc, char** argv){
	ros::init(argc, argv, "Node_B");
	TiagoAction tiago("Node_B");
	ros::spin();
	return 0;
}
