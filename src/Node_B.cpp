#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/TiagoAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Vector3.h> //Import Vector3 to define threedimensional vectors for linear and angular velocities

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // alias for the move_base action client
typedef actionlib::SimpleActionServer<ir2324_group_24::TiagoAction> Action_Server; // alias for the Node_A communication Action Server


class TiagoAction{ 

	private:

        // Class variables
		ros::NodeHandle nh_; // Node_B handle
		Action_Server as_;  // action server for the communication with Node_B (client)
		ir2324_group_24::TiagoFeedback feedback_; // storing Tiago feedback_ during its journey
		//ir2324_group_24::TiagoResult result_; // not needed for now
		std::string action_name_; //  storing the name of the node: Node_B
		std::string tiago_status; // string storing Tiago's status to be set as feedback when necessary
		std::vector<int> goal_; // vector storing the AprilTags ID's to be found
        std::vector<double> scan_ranges;// containing the values of the scanner
        double scan_angle_increment; // minimum angle of the scanner
        ros::Publisher pub; // publisher for velocity commands
        bool found_all_aprilTags = false; // its true when all AprilTags are found!
        int count = 0; // to delete 

        //------------------------ CALLBACK FUNCTION -----------------------------

        // communication of the Tiago Action with Node_A
        // _____________________________________________
        // this is the first callback to be called 
        // this callback will contain all Tiago journey 
        // towards the goal, first retrieves the aprilTags 
        // IDs, then initialize exploration mode, and 
        // finally activates exploration mode to start 
        // moving Tiago. When the goal is achieved, 
        // exploration mode terminates, The first call 
        // of the callback waits for the end of the 1Hz
        // current loop and sets the goal as succeded to
        // Node_A, which terminates it. 
        void tiagoActionCB(const ir2324_group_24::TiagoGoalConstPtr &goal) {
            ros::Rate r(1);
            // retrieve the IDs first
            ROS_INFO("Received AprilTag IDs:");
            for (int i = 0; i < goal->apriltag_ids.size(); i++) {
                goal_.push_back(goal->apriltag_ids[i]);
                ROS_INFO("ID: %ld", goal->apriltag_ids[i]);
            }
            // set status as ready
            feedback("Tiago ready to navigate");

            // INITIALIZING EXPLORATION MODE
            
            // initializing the geometry_msgs/twist publisher
            // to publish the new tiago velocities commands
            pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);


            // ACTIVATE EXPLORATION MODE!
            ROS_INFO("[EXPLORATION MODE] activated");
            explorationMode();
            // exploration mode deactivated: goal achieved!
            ROS_INFO("[EXPLORATION MODE] deactivated: Tiago found all AprilTags !");
            // wait for current loop to finish (1Hz)
            r.sleep();
            // sets the goal as succeded to Node_A, which terminates!
            as_.setSucceeded();
        }

        // Callback for LaserScan
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            scan_ranges.clear();
            for(size_t i = 0; i < msg -> ranges.size(); i++){
                scan_ranges.push_back(msg -> ranges[i]);
            }
            scan_angle_increment = msg -> angle_increment;
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
        
        
        // method to compute the Next Tiago Pose to keep exploring
        // untill all AprilTags are found
        geometry_msgs::Twist computeNextVelCmd(){
            
            // define next velocity commands
            geometry_msgs::Twist next_cmd_vel;

            //define the vector3 linear velocity parameters
            tf::Vector3 next_linear_vel(1.0, 0, 0);
            next_cmd_vel.linear.x = next_linear_vel.x();
            next_cmd_vel.linear.y = next_linear_vel.y();
            next_cmd_vel.linear.z = next_linear_vel.z();

            //define the vector3 angular velocity parameters
             tf::Vector3 next_angular_vel(0, 0, 0);
            next_cmd_vel.angular.x = next_angular_vel.x();
            next_cmd_vel.angular.y = next_angular_vel.y();
            next_cmd_vel.angular.z = next_angular_vel.z();

            return next_cmd_vel;
        }


        // EXPLORATION MODE method
        //__________________________________________________
        // this method contains the whole Tiago's journey
        // towards achieving the goal: finding all AprilTags
        // contained in goal_.
        void explorationMode(){

            // EXPLORATION MODE journey
            // ends when all AprilTags are found
            while(!found_all_aprilTags){
                // to delete
                if(count==1){
                    found_all_aprilTags = true;
                }
                pub.publish(computeNextVelCmd());

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
            // Subscribers for LaserScan and PointCloud and Map
            ros::Subscriber laser_scan_sub = nh_.subscribe("/scan", 1, &TiagoAction::laserScanCallback, this);
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


