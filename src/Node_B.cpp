#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/TiagoAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // alias for the action client
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
		//nav_msgs::OccupancyGrid::ConstPtr global_costmap;
        bool found_all_aprilTags = false;
        bool got_aprilTagsIDs = false;
	public:
		// Constructor 
		TiagoAction(std::string name):as_(nh_, name, boost::bind(&TiagoAction::tiagoActionCB, this, _1), false), action_name_(name)
		{
            // Start the action server
            as_.start();
            ROS_INFO("Action server [tiago_action] started.");
        }

		// Destructor
		~TiagoAction(void){
        }

        //------------------------ METHODS ---------------------------------

        // updates tiago_status and publish the feedback
        // WARNING: use as private method for synchronization
        void feedback(std::string status){
            tiago_status = status;
            // update the feedback to publish 
		    feedback_.robot_status = tiago_status;
            // publish feedback
            as_.publishFeedback(feedback_);
        }
        
        // exploration mode activated
        void explorationMode( MoveBaseClient &ac){
            // loop until the aprilTags are found is accomplished
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "base_link";
            int count = 0;// default avanzamento di 1 metro 3 volte
            while(!found_all_aprilTags){
                if(count==2){
                    found_all_aprilTags = true;
                }
                // define the goal
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = 1.0;
                goal.target_pose.pose.position.y = 0.0;
                goal.target_pose.pose.orientation.w = 1.0;

                // send the goal
                feedback("next goal defined and processing ...");
                ac.sendGoal(goal);

                // wait for the goal to be accomplished
                ac.waitForResult();
                // verify success
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    feedback("Hooray, the base moved 1 meter forward");
                }
                else
                    feedback("The base failed to move forward 1 meter for some reason");
                count++;
            }
            return;
        }

    private:
	    //------------------------ CALLBACK FUNCTIONS -----------------------------

        // communication of the Tiago Action with Node_A
        void tiagoActionCB(const ir2324_group_24::TiagoGoalConstPtr &goal) {
            ros::Rate r(1);
            if(!got_aprilTagsIDs){
                // retrieve the IDs 
                ROS_INFO("Received AprilTag IDs:");
                for (int i = 0; i < goal->apriltag_ids.size(); i++) {
                    goal_.push_back(goal->apriltag_ids[i]);
                    ROS_INFO("ID: %d", goal->apriltag_ids[i]);
                }
                got_aprilTagsIDs = true;
                // set status as ready
                feedback("Tiago ready to navigate");
            }
            
            // initializing the action client
            MoveBaseClient ac("move_base", true);
            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            // start exploration mode
            explorationMode(ac);

            r.sleep();
            //
            as_.setSucceeded();
        }
    
};




int main(int argc, char** argv){

    // initializing the node
    ros::init(argc, argv, "Node_B");

    // initialising the action server
    TiagoAction tiago("Node_B");
    ros::spin();

    return 0;
}


