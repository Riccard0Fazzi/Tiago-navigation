#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/TiagoAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
        ros::Publisher vel_cmd_pub; // publisher for velocity commands
        ros::Subscriber laser_scan_sub = nh_.subscribe("/scan", 10, &TiagoAction::laserScanCallback, this); // subscriber for laser scanner topic
        ros::Subscriber tag_sub = nh_.subscribe("/tag_detections", 10, &TiagoAction::tagDetectionCallback, this); // subscriber for apriltag detection
        image_transport::Subscriber image_sub;
        bool found_all_aprilTags = false; // its true when all AprilTags are found!
        int count = 0; // to delete 

        //------------------------ CALLBACK FUNCTIONS -----------------------------

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
            // and actually move Tiago's motors
            vel_cmd_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);


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
        // this updates scan latest information
        // every 10 Hz
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            scan_ranges.clear();
            for(size_t i = 0; i < msg -> ranges.size(); i++){
                scan_ranges.push_back(msg -> ranges[i]);
            }
            scan_angle_increment = msg -> angle_increment;
            feedback("laser");
        
        }

        // CallBack for AprilTags Detection
        // (20 Hz)
        // Callback function to handle detected tags
        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
            // No AprilTags found!
            if (msg->detections.empty()) {
                //ROS_INFO("No AprilTags detected.");
                // returhn to terminate the callback
                return;
            }

            for (const auto& detection : msg->detections) {
                int id = detection.id[0]; // AprilTag ID
                double x = detection.pose.pose.pose.position.x;
                double y = detection.pose.pose.pose.position.y;
                double z = detection.pose.pose.pose.position.z;

                ROS_INFO("Detected tag ID: %d, Position: [%.2f, %.2f, %.2f]", id, x, y, z);
            }
            // update remaining AprilTags
        }

        // CallBack for displaying a Tiago's camera view 
        // at any moment [VISUAL PURPOSE ONLY]
        // Callback to process the RGB image (30Hz)
        void tiagoVisualCallback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;

                // Optional: Display the image (for debugging purposes)
                cv::imshow("Tiago's Perspective", img);
                cv::waitKey(1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
            }
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


        // BEHAVIOR ONE method
        // _______________________________
        // simple behaviour ...
        geometry_msgs::Twist behaviorOne(){
            // input: laserScan (10Hz) || output: velocity commands (10Hz) 

            // std::vector<double> scan_ranges; containing the values of the scanner
            // double scan_angle_increment; angle of increment in the Tiago's view
            // laserScan data elaboration

            // define the next velocity commands for Tiago's 
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
        
        
        // BEHAVIORAL CONTROL method
        // _______________________________
        // this method implements a Tiago
        // velocity controller that 
        // manages different simple 
        // behaviours that will keep 
        // Tiago exploring the environment
        // before finding all AprilTags
        void behavioralControl(){
            // input: laserScan (10Hz) || output: velocity commands (10Hz) 
            // (not synchronous)
            // initialize the next velocity commands objects
            geometry_msgs::Twist next_cmd_vel;
            geometry_msgs::Twist behavior_one;
            // ecc

            // Call BEHAVIOUR ONE
            behavior_one = behaviorOne();

            // ...

            // BEHAVIOURAL CONTROLLER
            next_cmd_vel = behavior_one;

            // MOVE TIAGO
            // publish the next velocity commands
            vel_cmd_pub.publish(next_cmd_vel);
            // publish feedback about velocity commands
            feedback("NEXT VELOCITY COMMAND: \n[Linear]  | Vx = " + 
                std::to_string(next_cmd_vel.linear.x) + "| Vy = " +
                std::to_string(next_cmd_vel.linear.y) + "| Vz = " +
                std::to_string(next_cmd_vel.linear.z) + "| \n" +
                "[Angular] | Wx = " + std::to_string(next_cmd_vel.angular.x) + "| Wy = " +
                std::to_string(next_cmd_vel.angular.y) + "| Wz = " +
                std::to_string(next_cmd_vel.angular.z) + "|");

            count++; // to delete
        }


        // EXPLORATION MODE method
        //__________________________________________________
        // this method contains the whole Tiago's journey
        // towards achieving the goal: finding all AprilTags
        // contained in goal_.
        void explorationMode(){
            // using behavioural control 
            feedback("[BEHAVIOURAL CONTROL] activated");
            // Set the rate for the velocity commands to Tiago 
            ros::Rate rate(10);

            // EXPLORATION MODE journey
            // ends when all AprilTags are found  
            while(!found_all_aprilTags && ros::ok()){
                // to delete (300, duration of test)
                if(count==300){
                    found_all_aprilTags = true;
                }
                // activate BEHAVIORAL CONTROL
                // this control manages different simple
                // behaviours of Tiago to explore the 
                // environment
                behavioralControl();
                // Sleep to enforce the rate
                rate.sleep();
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
            // Subscriber for LaserScan topic /scan (messages rate: 10 Hz)
            //laser_scan_sub = nh_.subscribe("/scan", 10, &TiagoAction::laserScanCallback, this);
            // Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_transport::ImageTransport it(nh_); // image transport for the camera topic
            image_sub = it.subscribe("/xtion/rgb/image_color", 100, &TiagoAction::tiagoVisualCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            //tag_sub = nh_.subscribe("/tag_detections", 10, &TiagoAction::tagDetectionCallback, this);
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


