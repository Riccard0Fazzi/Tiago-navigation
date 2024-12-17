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
#include <tf/LinearMath/Vector3.h> //Import Vector3 to define threedimensional vectors for linear and angular velocities

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
        double OBSTACLE_DISTANCE_THRESHOLD; // Threshold to avoid obstacles (meters)
        double ESCAPE_DISTANCE_THRESHOLD; // Threshold for escape directions (meters)
        double FORWARD_LINEAR_SPEED;       // Forward linear speed (m/s)
        double REPULSION_SCALE;    // Scale factor for repulsion
        double ATTRACTION_SCALE;    // Scale factor for attraction
        double ESCAPE_SCALE; // Scale factor for following escape distance
        double LONG_DISTANCE_SCALE; // scale factor for acceleration
        double SHORT_DISTANCE_SCALE;
        double CLUSTER_GAP_THRESHOLD; // used to discretize different escaping directions
        bool found_all_aprilTags; // its true when all AprilTags are found!

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
            feedback("Initialize camera ...");
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
            feedback("[Camera-Initialized]");
        }

        // method to update Tiago's explore behavior parameters
        // to achieve a control of smaller behaviors
        void behavioralControl(){

            // PROCESSING DATA FOR EACH BEHAVIOR
            // pick the minimum distance in a range of values near the forward direction 
            double straight_free_distance = 1000;
            for(size_t i = (scan_ranges.size()/2)-32; i < (scan_ranges.size()/2) + 32; i++)
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
                ESCAPE_DISTANCE_THRESHOLD = 3;
                // send feedback
                feedback("[BEHAVIOR TWO] avoid traps");
            }

            // BEHAVIOR THREE: avoid osbtacle
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
                feedback("[BEHAVIOR THREE] avoid obstacle");
            }
                
        }

        // ESCAPE DIRECTION method
        // ________________________________________________
        // find the escape direction to guide Tiago towards
        // the exploration
        double escapeDirection(){
            // long distances generate an attractive force proportional to their distance
            // Input: scan_ranges (distances), scan_angle_min, scan_angle_increment
            std::vector<std::vector<double>> clusters_of_angles;

            // Variables to keep track of clusters
            std::vector<double> current_cluster_angles;
            size_t previous_index = 0;
            bool in_cluster = false;

            // Loop through scan_ranges to identify clusters
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                double distance = scan_ranges[i];
                double angle = scan_angle_min + i * scan_angle_increment;

                // Check if the distance meets the threshold
                if (distance > ESCAPE_DISTANCE_THRESHOLD) {
                    // Check for a gap in indices to split clusters
                    if (!in_cluster || (i - previous_index) > CLUSTER_GAP_THRESHOLD) {
                        // If a cluster exists, save it and start a new one
                        if (!current_cluster_angles.empty()) {
                            clusters_of_angles.push_back(current_cluster_angles);
                            current_cluster_angles.clear();
                        }
                        in_cluster = true;
                    }

                    // Add current angle to the cluster
                    current_cluster_angles.push_back(angle);
                    previous_index = i;  // Update the previous index
                } 
                else {
                    // If not in a cluster, reset state
                    if (in_cluster) {
                        clusters_of_angles.push_back(current_cluster_angles);
                        current_cluster_angles.clear();
                    }
                    in_cluster = false;
                }
            }

            // Add the last cluster if it exists
            if (!current_cluster_angles.empty()) {
                clusters_of_angles.push_back(current_cluster_angles);
            }
            /*
            // Output the clusters and number of clusters found
            ROS_INFO("Number of clusters found: %lu", clusters_of_angles.size());
            for (size_t j = 0; j < clusters_of_angles.size(); ++j) {
                ROS_INFO("Cluster %lu has %lu angles", j, clusters_of_angles[j].size());
            }*/
            // choose best cluster
            // Handle case with no clusters
            if (clusters_of_angles.empty()) {
                feedback("[WARNING] No escape direction found!");
                return 0.0;
            }

            // Seed the random number generator (once per program execution)
            static bool is_rand_seeded = false;
            if (!is_rand_seeded) {
                srand(static_cast<unsigned int>(time(0)));
                is_rand_seeded = true;
            }

            // Randomly select one cluster
            size_t random_cluster_index = rand() % clusters_of_angles.size();
            const std::vector<double>& selected_cluster = clusters_of_angles[random_cluster_index];

            // Incrementally sum the angular contributes
            double escape_direction_angle = 0.0;
            int escape_direction_normalization = 0;
            for (const double& angle : selected_cluster) {
                escape_direction_angle += angle;
                escape_direction_normalization++;
            }
            escape_direction_angle = escape_direction_angle/escape_direction_normalization;
            // Return the final value of the contribution
            return ESCAPE_SCALE*sin(escape_direction_angle);
        }


        // EXECUTE BEHAVIOR method
        // __________________________________________________
        // publish the next velocity commands resulting to the
        // adapted explore behavior ( behavioralControl() )
        void executeBehavior(){
            // define object to contain velocity commands
            geometry_msgs::Twist next_cmd_vel;

            // long distances generate an attractive force proportional to their distance
            // Obstacles generate a repulsive force proportional to their proximity
            // Combine the forces into a resultant motion vector.

            /*
            // Initialize total repulsive angular velocity
            double repulsive_angular_velocity = 0.0;
            double escape_direction_angle = 0.0;
            int escape_direction_normalization = 0;
            int i_prec = 0;
            bool escape_direction_computed = false;
            std::vector<std::vector<double>> clusters_of_angles;

            // Compute the best next direction for tiago
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                double distance = scan_ranges[i];

                // escape direction vector computation
                if(distance > ESCAPE_DISTANCE_THRESHOLD && !escape_direction_computed){
                    // compute the angle of the escape direction contribute
                    double angle = scan_angle_min + i * scan_angle_increment;
                    ROS_INFO("[distance] = %f",distance);
                    ROS_INFO("[angle] = %f",angle);
                    //double attraction = ATTRACTION_SCALE;  // the more distant the stronger 
                    // sum the contribute
                    escape_direction_angle += angle; 
                    // update normalization factor
                    escape_direction_normalization++;
                    // take just one escape direction not multiples
                    if(i_prec!=0 && (i-i_prec)>50){
                        escape_direction_computed = true;
                    }

                    i_prec = i;
                }
                if(escape_direction_normalization !=0){
                    escape_direction_angle = escape_direction_angle/escape_direction_normalization;
                    repulsive_angular_velocity = ESCAPE_SCALE*sin(escape_direction_angle);
                }
                
                // obstacle detected at distance
                if (distance < OBSTACLE_DISTANCE_THRESHOLD && distance > tiago_shape) {
                    //feedback(std::to_string(distance));
                    // Calculate repulsion angle based on scan index
                    double angle = scan_angle_min + i * scan_angle_increment;
                    double repulsion = REPULSION_SCALE / distance;  // Stronger repulsion when closer
                    repulsive_angular_velocity += -repulsion * sin(angle);  // Summing angular effects

                }
            }*/
            double angular_velocity = 0.0;
            angular_velocity = escapeDirection();
            // Obstacle detected: turn away using repulsive forces
            next_cmd_vel.angular.z = angular_velocity;
            //ROS_INFO("[Wz] = %f", angular_velocity);
            //double random_wiggle = ((rand() % 200) - 100) / 1000.0; // Random value between -0.1 and 0.1
            //next_cmd_vel.angular.z = random_wiggle;

            // always moving forward 
            next_cmd_vel.linear.x = FORWARD_LINEAR_SPEED;

            // [MOVE TIAGO]
            // publish the next velocity commands
            vel_cmd_pub.publish(next_cmd_vel);
        }
        
        // 
        
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

            // INITIALIZATION

            // initialize the next velocity commands object
            geometry_msgs::Twist next_cmd_vel;

            // adapt the exploring behavior to the current situation
            behavioralControl();

            // set next velocity commands and move Tiago
            executeBehavior();

            
        }


        // EXPLORATION MODE method
        //__________________________________________________
        // this method contains the whole Tiago's journey
        // towards achieving the goal: finding all AprilTags
        // contained in goal_.
        void explorationMode(){
            // Set the rate for the velocity commands to Tiago 
            ros::Rate rate(10);
        
            // using behavioural control 
            feedback("[EXPLORATION MODE] initialized");
            feedback("[EXPLORE BEHAVIOR] activated");

            // EXPLORATION MODE journey
            // ends when all AprilTags are found  
            while(!found_all_aprilTags && ros::ok()){
                /*
                // to delete (300, duration of test)
                if(count==300){
                    found_all_aprilTags = true;
                }*/
                // activate WXPLORING BEHAVIOR
                // this behavior manages manu different simple
                // behaviours of Tiago to explore the 
                // environment and detect all the AprilTags
                exploreBehavior();
                // Sleep to enforce the rate
                rate.sleep();
            }
            
            // exploration mode deactivated: goal achieved!
            feedback("[EXPLORATION MODE] deactivated: Tiago found all AprilTags!");
            return;
        }



	public:
		// Constructor 
		TiagoAction(std::string name):as_(nh_, name, boost::bind(&TiagoAction::tiagoActionCB, this, _1), false), action_name_(name)
		{
            // Initialize parameters
            tiago_shape = 0.2; // below it's tiago's body
            OBSTACLE_DISTANCE_THRESHOLD = 0.35; // below it's an obstacle
            ESCAPE_DISTANCE_THRESHOLD = 1.5; // above it's an escape direction
            FORWARD_LINEAR_SPEED = 0.0; // Tiago straight speed
            REPULSION_SCALE = 0.1; // how much the obstacle count to go in opposite direction
            ATTRACTION_SCALE = 0.1; // how much the general direction weights as a contribute
            ESCAPE_SCALE = 1; // how much tiago is following escape directioon
            LONG_DISTANCE_SCALE = 0.3; // acceleration scale for long distance travel
            SHORT_DISTANCE_SCALE = 0.05; // acceleration scale for short distance attention
            CLUSTER_GAP_THRESHOLD = 2;
            found_all_aprilTags = false; // no AprilTags found yet, start looking for them

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
            // publisher for the initial tilt of the camera 
            // to be ready to detect aprilTags
            tilt_cam_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
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


