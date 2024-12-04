#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/TiagoAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // alias for the action client
typedef actionlib::SimpleActionServer<ir2324_group_24::TiagoAction> Action_Server; // alias for the Action Server


class TiagoAction{ 

	private:

        // Class variables
		ros::NodeHandle nh_; 
		Action_Server as_;  
		ir2324_group_24::TiagoFeedback feedback_;
		ir2324_group_24::TiagoResult result_;
		std::string action_name_;
		std::string tiago_status;
		std::vector<int> goal_;
        bool found_all_aprilTags = false;
        bool got_aprilTagsIDs = false;
        int count = 0; // to delete

        //------------------------ CALLBACK FUNCTION -----------------------------

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

        //------------------------ METHODS ---------------------------------

        // method to update tiago_status and publish the feedback
        void feedback(std::string status){
            tiago_status = status;
            // update the feedback to publish 
		    feedback_.robot_status = tiago_status;
            // publish feedback
            as_.publishFeedback(feedback_);
        }
        
        
        // method to compute the Next Goal
        void computeNextGoal(move_base_msgs::MoveBaseGoal &goal){

            // Compute Next Goal
            float x = 1.0;
            float y = 0.0;
            float w = 1.0;

            //set next goal
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
            goal.target_pose.pose.orientation.w = w;
            count++;

            // publish feedback about the goal
            // define the feedback string
            std::string next_goal_feedback = "[Next Goal] x = " + std::to_string(x) +
                         ", y = " + std::to_string(y) +
                         ", w = " + std::to_string(w);
            feedback(next_goal_feedback);
        }


        // method to trigger exploration mode
        void explorationMode( MoveBaseClient &ac){
            // initialize goal
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "base_link";

            // loop until the aprilTags are found is accomplished
            while(!found_all_aprilTags){
                if(count==2){
                    found_all_aprilTags = true;
                }

                // Compute Next Goal
                computeNextGoal(goal);

                // send Next Goal
                ac.sendGoal(goal);
                // publishing feedback
                feedback("[Next Goal] Defined and processing ...");
                // wait for the goal to be accomplished by move_base
                ac.waitForResult();
                // verify success
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    feedback("Hooray, Tiago accomplished the goal !");
                }
                else
                    feedback("Tiago failed to accomplish the goal");
            }
            return;
        }



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

};




int main(int argc, char** argv){

    // initializing the node
    ros::init(argc, argv, "Node_B");

    // initialising the action server
    TiagoAction tiago("Node_B");
    ros::spin();

    return 0;
}


