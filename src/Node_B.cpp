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
#include <string> // just for visual debugging purpose

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
        double scan_angle_increment; // minimum increment in the angles of the scanner
        double scan_angle_min; // initial angle of the scanner
        double tiago_shape = 0.2; // distance from Tiago base frame and Tiago's perimeter in meters
        ros::Publisher vel_cmd_pub; // publisher for velocity commands
        ros::Subscriber laser_scan_sub; // subscriber for laser scanner topic
        ros::Subscriber tag_sub; // subscriber for apriltag detection
        image_transport::Subscriber image_sub; // subscriber for camera 
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
            // update the scan measurements
            scan_ranges.clear();
            for(size_t i = 0; i < msg -> ranges.size(); i++){
                scan_ranges.push_back(msg -> ranges[i]);
            }
            // retrieve the scan constant angle increment 
            scan_angle_increment = msg -> angle_increment;
            // retrieve the scan constant initial angle
            scan_angle_min = msg -> angle_min;
        
        }

        // CallBack for AprilTags Detection
        // (20 Hz)
        // Callback function to handle detected tags
        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
            // No AprilTags found!
            if (msg->detections.empty()) {
                //feedback("No AprilTags detected.");
                // return to terminate the callback
                return;
            }

            for (const auto& detection : msg->detections) {
                int id = detection.id[0]; // AprilTag ID
                double x = detection.pose.pose.pose.position.x;
                double y = detection.pose.pose.pose.position.y;
                double z = detection.pose.pose.pose.position.z;

                feedback("Detected tag ID: " + std::to_string(id) + ", Position: [" + 
                        std::to_string(x) + ", " + 
                        std::to_string(y) + ", " + 
                        std::to_string(z) + "]");
            }
            // update remaining AprilTags
        }

        // CallBack for displaying a Tiago's camera view 
        // at any moment [VISUAL PURPOSE ONLY]
        // Callback to process the RGB image (30Hz)
        void tiagoEyesCallback(const sensor_msgs::ImageConstPtr& msg) {
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


        // EXPLORE BEHAVIOR method
        // __________________________________________________
        // wandering + obstacle avoidance 
        geometry_msgs::Twist exploreBehavior(const double OBSTACLE_DISTANCE_THRESHOLD, const double FORWARD_LINEAR_SPEED, const double REPULSION_SCALE){
            // input: laserScan (10Hz) || output: velocity commands (10Hz) 

            // define object to contain velocity commands
            geometry_msgs::Twist next_cmd_vel;

            // Obstacles generate a repulsive force proportional to their proximity
            // A global wandering vector (e.g., forward motion) acts as an attractive force
            // Combine the forces into a resultant motion vector.


            // Initialize total repulsive angular velocity
            double repulsive_angular_velocity = 0.0;

            // Compute repulsion from obstacles
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                double distance = scan_ranges[i];
                if (distance < OBSTACLE_DISTANCE_THRESHOLD && distance > tiago_shape) {
                    feedback(std::to_string(distance));
                    // Calculate repulsion angle based on scan index
                    double angle = scan_angle_min + i * scan_angle_increment;
                    double repulsion = REPULSION_SCALE / distance;  // Stronger repulsion when closer
                    repulsive_angular_velocity += -repulsion * sin(angle);  // Summing angular effects
                }
            }

            if (std::abs(repulsive_angular_velocity) > 0.0) {
                // Obstacle detected: turn away using repulsive forces
                next_cmd_vel.angular.z = repulsive_angular_velocity;
                feedback("Repelling from obstacles.");
            }
            else{
                // Random wandering
                double random_wiggle = ((rand() % 200) - 100) / 1000.0; // Random value between -0.1 and 0.1
                next_cmd_vel.angular.z = random_wiggle;
                feedback("No obstacles. Wandering...");
            }
            // always moving forward 
            next_cmd_vel.linear.x = FORWARD_LINEAR_SPEED;

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

            // INITIALIZATION
            // Initialize exploring-behavior parameters
            const double OBSTACLE_DISTANCE_THRESHOLD = 0.35; // Threshold to avoid obstacles (meters)
            const double FORWARD_LINEAR_SPEED = 0.1;       // Forward linear speed (m/s)
            const double REPULSION_SCALE = 0.05;    // Scale factor for repulsion

            // initialize the next velocity commands objects
            geometry_msgs::Twist next_cmd_vel;
            geometry_msgs::Twist explore_behavior;
            // ecc

            // Call the EXPLORING BEHAVIOUR 
            explore_behavior = exploreBehavior(OBSTACLE_DISTANCE_THRESHOLD,
                                         FORWARD_LINEAR_SPEED, REPULSION_SCALE);

            // ...

            // BEHAVIOURAL CONTROLLER
            next_cmd_vel = explore_behavior;

            // [MOVE TIAGO]
            // publish the next velocity commands
            vel_cmd_pub.publish(next_cmd_vel);
            // publish feedback about velocity commands
            feedback("[NEXT VELOCITY CMD] \n[Linear]  | Vx = " + 
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
            feedback("[BEHAVIORAL CONTROL] activated");
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
                // environment and detect all the AprilTags
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
            laser_scan_sub = nh_.subscribe("/scan", 10, &TiagoAction::laserScanCallback, this);
            // Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_transport::ImageTransport it(nh_); // image transport for the camera topic
            image_sub = it.subscribe("/xtion/rgb/image_color", 100, &TiagoAction::tiagoEyesCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            tag_sub = nh_.subscribe("/tag_detections", 10, &TiagoAction::tagDetectionCallback, this);
            // publisher for velocity commands
            // initializing the geometry_msgs/twist publisher
            // to publish the new tiago velocities commands
            // and actually move Tiago's motors
            vel_cmd_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
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


