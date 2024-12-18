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
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <tf/LinearMath/Vector3.h> //Import Vector3 to define threedimensional vectors for linear and angular velocities
#include <tf/tf.h> // For quaternion calculations

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
        double scan_angle_increment; // minimum increment in the angles of the scanner
        double scan_angle_min; // initial angle of the scanner
        double tiago_shape; // distance from Tiago base frame and Tiago's perimeter in meters
        ros::Publisher vel_cmd_pub; // publisher for velocity commands
        ros::Publisher tilt_cam_pub; // publisher for the initial tilt of the camera
        ros::Subscriber laser_scan_sub; // subscriber for laser scanner topic
        ros::Subscriber tag_sub; // subscriber for apriltag detection
        image_transport::Subscriber image_sub; // subscriber for camera 
        double AVOID_DISTANCE_THRESHOLD; // Threshold to avoid obstacles (meters)
        double ESCAPE_DISTANCE_THRESHOLD; // Threshold for escape directions (meters)
        double AVOID_CLUSTER_THRESHOLD; // Threshold to avoid obstacles (meters)
        double ESCAPE_CLUSTER_THRESHOLD; // Threshold for escape directions (meters)
        double avoid_direction_angle; // angle of avoidance of an obstacle
        double obstacle_distance; // distance of the closest obstacle
        double escape_direction_angle; // angle of escape direction
        double escape_distance; // distance of the escape direction
        double escape_factor;
        bool explore; // true when obstacle ahead
        bool no_escape;
        bool found_all_aprilTags; // its true when all AprilTags are found!
	bool corridor;

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

            // INITIALIZE CAMERA 
            initializeCamera();

            // ACTIVATE EXPLORATION MODE!
            explorationMode();
        
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
            scan_angle_min = msg->angle_min;
        
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

            int id = msg->detections[0].id[0]; // AprilTag ID found
            double x = msg->detections[0].pose.pose.pose.position.x;
            double y = msg->detections[0].pose.pose.pose.position.y;
            double z = msg->detections[0].pose.pose.pose.position.z;
            
            // update remaining AprilTags
            // Search for found id in the goal_ vector
            // containing all aprilTags to find
            auto it = std::find(goal_.begin(), goal_.end(), id); 
            // if it found the aprilTag in the vector remove it!
            if (it != goal_.end()) {
                goal_.erase(it); // Remove the found AprilTag
                //ROS_INFO("goal_ size after removal: %d",goal_.size());
                // print the found aprilTag
                feedback("[NEW APRILTAG FOUND] ID: " + std::to_string(id) + ", Position: [" + 
                        std::to_string(x) + ", " + 
                        std::to_string(y) + ", " + 
                        std::to_string(z) + "]");
            }

            // Conversion of aprilTag pose from camera_frame to base_link

            // check if Tiago found all aprilTags and terminate program if so
            if(goal_.size()==0){
                found_all_aprilTags = true;
            }
        }

        // CallBack for displaying a Tiago's camera view 
        // at any moment [VISUAL PURPOSE ONLY]
        // Callback to process the RGB image (30Hz)
        void tiagoEyesCallback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;

                // Optional: Display the image (for debugging purposes)
                cv::imshow("Tiago Eyes", img);
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

        // method to initialize the camera, tilting the camera 
        // enough to view the aprilTags while exploring
        void initializeCamera(){
            // define rate for the initialization operation
            ros::Rate cam_init_r(1);
            ROS_INFO("Initialize camera ...");
            // waiting for subscribers to /head_controller/command
            while(tilt_cam_pub.getNumSubscribers() == 0 && ros::ok()){
                cam_init_r.sleep();
            }
            // initialize object containing the command to
            // send to Tiago
            trajectory_msgs::JointTrajectory tilt_cmd;
            trajectory_msgs::JointTrajectoryPoint point;
            // set the tilt command 
            tilt_cmd.joint_names = {"head_1_joint","head_2_joint"};
            point.positions = {0.0,-13.0};
            point.time_from_start = ros::Duration(1.0);
            tilt_cmd.points.push_back(point);
            // send the tilt command to Tiago
            tilt_cam_pub.publish(tilt_cmd);
            // wait for Tiago to incline the camera ..
            cam_init_r.sleep();
            // publish feedback
            ROS_INFO("[Camera-Initialized]");
        }

        /*
        // method to update Tiago's explore behavior parameters
        // to achieve a control of smaller behaviors
        void behavioralControl(){
            // PROCESSING DATA -> behavior adaptation

            // Tiago Perception adaptation

            // compute the obstacles presence current contribute around tiago

            // Initialize total repulsive angular velocity
            double repulsive_angular_force = 0.0;
            int repulsive_angular_normalization = 0;
            // Compute the obstacles contribution
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                double distance = scan_ranges[i];
                // obstacle detected at distance
                if (distance < OBSTACLE_DISTANCE_THRESHOLD && distance > tiago_shape) {
                    //feedback(std::to_string(distance));
                    // Calculate repulsion angle based on scan index
                    double angle = scan_angle_min + i * scan_angle_increment;
                    repulsive_angular_force += - angle;  // Summing angular effects
                    repulsive_angular_normalization++;
                }
            }
            repulsive_angular_force = repulsive_angular_force/repulsive_angular_normalization;
            ROS_INFO("[repulsive_angular_force] = %f",repulsive_angular_force );

            // pick the minimum distance in a range of values near the forward direction 
            double straight_free_distance = 1000;
            for(size_t i = (scan_ranges.size()/2)-8; i < (scan_ranges.size()/2) + 8; i++)
            {
                if(scan_ranges[i] < straight_free_distance)
                    straight_free_distance=scan_ranges[i];

            }

            // BEHAVIOR ONE: follow corridor
            // if Tiago has 5 meters ahead move straight fast: "corridor behavior"
            if(straight_free_distance > 5){
                // assign new x speed proportional to straight distance
                // using a gretaer scale
                FORWARD_LINEAR_SPEED = LONG_DISTANCE_SCALE * straight_free_distance;
                // set long distance travel thresholding
                //OBSTACLE_DISTANCE_THRESHOLD = 0.35; 
                // set the scale of the repulsary contribute for long distance travel
                //REPULSION_SCALE = 0.1;
                // set scale of attractive contribute to zero since Tiago is already in the right direction 
                ESCAPE_SCALE = 0.0; 
                ESCAPE_DISTANCE_THRESHOLD = 100;
                // send feedback
                feedback("[BEHAVIOR ONE] follow corridor");
            }

            // BEHAVIOR TWO: avoid traps
            // if Tiago has something closer then 5 meters ahead but further than 3, follow escape direction 
            else if(straight_free_distance > 3){
                // assign new x speed proportional to straight distance 
                // using a smaller scale
                FORWARD_LINEAR_SPEED = SHORT_DISTANCE_SCALE + straight_free_distance/5;
                // set a short distance thresholding to not hit really close obstacles 
                //OBSTACLE_DISTANCE_THRESHOLD = 0.35; 
                // set the scale of the repulsary contribute to avoid even the little obstacles
                //REPULSION_SCALE = 0.3; 
                // set the scale of the attractive contribute to dominate with escape direction
                ESCAPE_SCALE = 1; 
                // set the distance above which the direction is considered an escape direction
                ESCAPE_DISTANCE_THRESHOLD = 3.5;
                // send feedback
                feedback("[BEHAVIOR TWO] obstacle above three meters ahead");
            }

	    // BEHAVIOR THREE: avoid traps
            // if Tiago has something closer then 5 meters ahead but further than 3, follow escape direction 
            else if(straight_free_distance > 1.5){
                // assign new x speed proportional to straight distance 
                // using a smaller scale
                FORWARD_LINEAR_SPEED = SHORT_DISTANCE_SCALE/2 + straight_free_distance/10;
                // set a short distance thresholding to not hit really close obstacles 
                //OBSTACLE_DISTANCE_THRESHOLD = 0.35; 
                // set the scale of the repulsary contribute to avoid even the little obstacles
                //REPULSION_SCALE = 0.3; 
                // set the scale of the attractive contribute to dominate with escape direction
                ESCAPE_SCALE = 1.5; 
                // set the distance above which the direction is considered an escape direction
                ESCAPE_DISTANCE_THRESHOLD = 2;
                // send feedback
                feedback("[BEHAVIOR THREE] obstacle above 1.5 meters ahead");
            }

            // BEHAVIOR FOUR: avoid osbtacle
            // if Tiago has something closer then 3 meters ahead, avoid the obstacle
            else{
                // assign new x speed proportional to straight distance 
                // using a smaller scale
                //FORWARD_LINEAR_SPEED = SHORT_DISTANCE_SCALE/2 + straight_free_distance/10;
                FORWARD_LINEAR_SPEED = 0;
                // set a higher distance to include more obstacles
                //OBSTACLE_DISTANCE_THRESHOLD = 0.6; 
                // set the scale of the repulsary contribute to a higher value to avoid obstacles
                //REPULSION_SCALE = 0.3; 
                // set the escape direction as guide
                ESCAPE_SCALE = 3; 
                // set the distance above which the direction is considered an escape direction
                ESCAPE_DISTANCE_THRESHOLD = 1.5;
                // send feedback
                feedback("[BEHAVIOR FOUR] obstacle below 1.5 meters ahead");
            }
                
        }
        */

        // ESCAPE BEHAVIOR method
        // _________________________________________
        // returns the angle of the escape direction
        void escapingPerception(){
            // cluster scan to detect the escape directions
            // long distances generate an attractive force proportional to their distance
            // Input: scan_ranges (distances), scan_angle_min, scan_angle_increment
            std::vector<std::vector<std::pair<double,double>>> clusters;
            no_escape = false;

            // Variables to keep track of clusters
            std::vector<std::pair<double,double>> current_cluster;
            size_t previous_index = 0;

            // Loop through scan_ranges to identify clusters
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                double distance = scan_ranges[i];
                double angle = scan_angle_min + i * scan_angle_increment;

                // Check if the distance meets the threshold
                if (distance > ESCAPE_DISTANCE_THRESHOLD) {
                    // Check for a gap in indices to split clusters
                    if ((i - previous_index) > ESCAPE_CLUSTER_THRESHOLD) {
                        // If a cluster exists, save it and start a new one
                        if (!current_cluster.empty()) {
                            clusters.push_back(current_cluster);
                            current_cluster.clear();
                        }
                        else{
                            current_cluster.clear();
                        }
                    }

                    // Add current angle to the cluster
                    current_cluster.push_back(std::make_pair(angle,distance)); // .first -> angle
                    previous_index = i;  // Update the previous index
                } 
            }

            // Add the last cluster if it exists
            if (!current_cluster.empty()) {
                clusters.push_back(current_cluster);
                current_cluster.clear();
            }
            if (clusters.empty()) {
                feedback("[WARNING] No escape direction found!");
                escape_direction_angle = 0.0;
                escape_distance = 0.0;
                no_escape = true;
                return;
            }
        

            // Seed the random number generator (once per program execution)
            static bool is_rand_seeded = false;
            if (!is_rand_seeded) {
                srand(static_cast<unsigned int>(time(0)));
                is_rand_seeded = true;
            }

            // Randomly select one cluster
            size_t random_cluster_index = rand() % clusters.size();
            std::vector<std::pair<double,double>>& selected_cluster = clusters[random_cluster_index];

            // Incrementally sum the angular contributes
            double escape_angle = 0.0;
            escape_distance = 0.0;
            for (std::pair<double,double> x : selected_cluster) {
                escape_angle += x.first;
                escape_distance += x.second;
            }
            escape_direction_angle = escape_angle/selected_cluster.size();
            escape_factor = selected_cluster.size();
            escape_distance = escape_distance/selected_cluster.size();

            return;
        }

        // AVOID OBSTACLE BEHAVIOR method
        // _________________________________________
        // returns the angle of the escape direction
        void avoidancePerception(){
            // cluster scan to detect the escape directions
            // long distances generate an attractive force proportional to their distance
            // Input: scan_ranges (distances), scan_angle_min, scan_angle_increment
            std::vector<std::vector<std::pair<double,double>>> clusters;

            // Variables to keep track of clusters
            std::vector<std::pair<double,double>> current_cluster;
            size_t previous_index = 0;
    
            // Loop through (90 LEFT, 90 RIGHT) scan_ranges to identify clusters
            for (size_t i = (scan_ranges.size()/2)-(90/scan_angle_increment); i < (scan_ranges.size()/2)+(90/scan_angle_increment) ; i++) {
                double distance = scan_ranges[i];
                double angle = scan_angle_min + i * scan_angle_increment;

                // Check if the distance meets the threshold
                if (distance < AVOID_DISTANCE_THRESHOLD && distance > tiago_shape) {
                    // Check for a gap in indices to split clusters
                    if ((i - previous_index) > AVOID_CLUSTER_THRESHOLD) {
                        // If a cluster exists and it's not too small, save it and start a new one
                        if (current_cluster.size() > 4) {
                            clusters.push_back(current_cluster);
                            current_cluster.clear();
                        }
                        else{
                            current_cluster.clear();
                        }
                    }

                    // Add current angle to the cluster
                    current_cluster.push_back(std::make_pair(angle,distance)); // .first -> angle
                    previous_index = i;  // Update the previous index
                } 
            }

            // Add the last cluster if it exists
            if (!current_cluster.empty()) {
                clusters.push_back(current_cluster);
            }
            
            // Output the clusters and number of clusters found
            ROS_INFO("Number of obstacles found: %lu", clusters.size());
            for (size_t j = 0; j < clusters.size(); ++j) {
                ROS_INFO("Obstacle %lu has %lu contributions", j, clusters[j].size());
            }

            if (clusters.empty()) {
                feedback("[WARNING] No obstacle ahead!");
                explore = true;
                return;
            }

            // compute the closest obstacle
            double min_distance = 1000.0;
            double closest_obstacle_distance = 0.0;
            double avoid_angle = 0.0;
            for (std::vector<std::pair<double,double>> obstacle : clusters) { // for each obstacle
                // take the minimum distance point of the obstacle 
                double obstacle_minimum_distance = 1000.0;
                for(std::pair<double,double> x : obstacle){
                    if(x.second<obstacle_minimum_distance){
                        closest_obstacle_distance = x.second; 
                        avoid_angle = x.first;
                    }
                }
                // choose the closest obstacle among all obstacles
                if(closest_obstacle_distance < min_distance){
                    min_distance = closest_obstacle_distance;
                    obstacle_distance = closest_obstacle_distance;
                    avoid_direction_angle = avoid_angle;
                }
            }
            ROS_INFO("%f < %f or %f > %f", avoid_direction_angle,-M_PI/8, avoid_direction_angle, M_PI/8);
            // setting obstacle variable
            if(avoid_direction_angle<-M_PI/8 || avoid_direction_angle > M_PI/8){
                explore = true;
            }

            //ROS_INFO("Closest obstacle at distance %f at %f direction angle", obstacle_distance, avoid_direction_angle);
            return;
        }
        // EXPLORE BEHAVIOR method
        // _______________________________
        // this method implements a Tiago
        // velocity controller that 
        // manages different simple 
        // behaviours that will keep 
        // Tiago exploring the environment
        // before finding all AprilTags
        void exploreBehavior(){
            // input: laserScan (10Hz) || output: velocity commands (10Hz) 
            // (not synchronous)

            // define velocity command object
            geometry_msgs::Twist next_cmd_vel;


            // [BEHAVIORAL CONTROL]

            // set tiago escape
            if(explore){
                // update perception
                escapingPerception();
                if(no_escape){
                    // rotate to get a better view
                    next_cmd_vel.angular.z = 0.3;
                    next_cmd_vel.linear.x = 0.0;
                    feedback("Searching for escape ...");
                    // [MOVE TIAGO]
                    // publish the next velocity commands
                    vel_cmd_pub.publish(next_cmd_vel);
                    return;
                }
                // set velocities
                next_cmd_vel.angular.z = sin(escape_direction_angle);
                next_cmd_vel.linear.x = 1;
                feedback("Exploring ...");
                // [MOVE TIAGO]
                // publish the next velocity commands
                vel_cmd_pub.publish(next_cmd_vel);
                return;
            }          
            // obstacle avoidance 
            else{
                // update perception
                avoidancePerception(); // avoid_direction_angle || obstacle_distance
                // choose left or right turning
                if(explore) return; // not rotate again if no obstacles found
                else{
                    double value = (std::rand() % 2 == 0) ? -1.0 : 1.0;
                    next_cmd_vel.angular.z = value*0.5;
                    next_cmd_vel.linear.x = 0.0;
                    feedback("Obstacle avoidance ...");
                    // [MOVE TIAGO]
                    // publish the next velocity commands
                    vel_cmd_pub.publish(next_cmd_vel);
                    return;
                }
            }
            /*
            // assign the next velocity commands
            next_cmd_vel.angular.z = (obstacle_distance*sin(escape_direction_angle))-
                                             (sin(avoid_direction_angle)/obstacle_distance);
            next_cmd_vel.linear.x = escape_distance * obstacle_distance * cos(escape_direction_angle);*/

            //ROS_INFO("[Wz] = %f", angular_velocity);
            //double random_wiggle = ((rand() % 200) - 100) / 1000.0; // Random value between -0.1 and 0.1
            //next_cmd_vel.angular.z = random_wiggle;

            //ROS_INFO("[Wz] = %f",next_cmd_vel.angular.z);
            //ROS_INFO("[Vx] = %f",next_cmd_vel.angular.x);
            feedback("should never arrive here!"); // debug
            return; 
        }

        void rotation()
        {
		ros::Rate r_rotation(20);
		int small_rotations = 0;
	       	while(ros::ok() && small_rotations < 57) // maybe 56 is the right value
		{
		    geometry_msgs::Twist next_cmd_vel;
                    next_cmd_vel.angular.z = M_PI;
		    vel_cmd_pub.publish(next_cmd_vel);
		    small_rotations++;
		    r_rotation.sleep();
		}
        }

	void corridor_mode()
	{
		ros::Rate r_corridor(10);
		// scan processing
		double left_future_side_distance;
		double right_future_side_distance;
		while(ros::ok()) 
		{
			left_future_side_distance = scan_ranges[(scan_ranges.size()/2)+(M_PI/4)/scan_angle_increment]*sin(M_PI/4);
			right_future_side_distance = scan_ranges[(scan_ranges.size()/2)+(M_PI/4)/scan_angle_increment]*sin(M_PI/4);
			geometry_msgs::Twist next_cmd_vel;
			if(left_future_side_distance > right_future_side_distance)
			{
				next_cmd_vel.linear.z = left_future_side_distance - right_future_side_distance; // increase if not curving enough 
			}
			else{
				next_cmd_vel.linear.z = -(right_future_side_distance - left_future_side_distance); // increase if not curving enough 
			}
			next_cmd_vel.linear.z = M_PI;
			vel_cmd_pub.publish(next_cmd_vel);
		}

		    
	}

        // EXPLORATION MODE method
        //__________________________________________________
        // this method contains the whole Tiago's journey
        // towards achieving the goal: finding all AprilTags
        // contained in goal_.
        void explorationMode(){
            // Set the rate 
            ros::Rate rate(1);
            
            MoveBaseClient ac_("move_base", true); // start the action client to send the next goal to move_base
            // wait for the action server to come up
            while(!ac_.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            // using behavioural control 
            ROS_INFO("[EXPLORATION MODE] initialized");

            // EXPLORATION MODE journey
            // ends when all AprilTags are found  
            while(!found_all_aprilTags && ros::ok()){ // use feedback to publish tiago status
                
                // 360 ROTATION with move_base
                // "checking around for AprilTags"  
              	rotation();
                // IF (corridor_mode) -> motion control law -> publish velocity commands
		if(corridor)
		{
			
		}
                // escapePerception and turn around randomically (move_base) if not valid

                // move_base to go to that position
                        /*
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
                                ROS_INFO("The base failed to move forward 1 meter for some reason");*/

                // Sleep to enforce the rate 
                rate.sleep();
            }
            
            // exploration mode deactivated: goal achieved!
            ROS_INFO("[EXPLORATION MODE] deactivated: Tiago found all AprilTags!");
            feedback("Tiago found all AprilTags!");
            return;
        }



	public:
		// Constructor 
		TiagoAction(std::string name):as_(nh_, name, boost::bind(&TiagoAction::tiagoActionCB, this, _1), false), action_name_(name)
		{
            // Initialize parameters
            tiago_shape = 0.2; // below it's tiago's body
            AVOID_DISTANCE_THRESHOLD = 1; // below it's a new obstacle found
            ESCAPE_DISTANCE_THRESHOLD = 4; // above it's an escape direction
            AVOID_CLUSTER_THRESHOLD = 1; // below it's a new obstacle found
            ESCAPE_CLUSTER_THRESHOLD = 5; // above it's an escape direction
            obstacle_distance = 0.0;
            escape_distance = 0.0;
            escape_factor = 0.0;
            avoid_direction_angle = 0.0;
            escape_direction_angle = 0.0;
            explore = false;
            no_escape = false;
            found_all_aprilTags = false; // no AprilTags found yet, start looking for them
	    corridor = true; // assumption: there's a corridor

          
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
            // publisher for the initial tilt of the camera 
            // to be ready to detect aprilTags
            tilt_cam_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
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
