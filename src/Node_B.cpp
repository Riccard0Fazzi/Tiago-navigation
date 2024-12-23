#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <ir2324_group_24/TiagoAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/LinearMath/Vector3.h> 
#include <tf/tf.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <deque>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // alias for the move_base action client
typedef actionlib::SimpleActionServer<ir2324_group_24::TiagoAction> Action_Server; // alias for the Node_A communication Action Server


class TiagoAction{ 

	private:

        // Class variables
		ros::NodeHandle nh_; // Node_B handle
		Action_Server as_;  // action server for the communication with Node_B (client)
		ir2324_group_24::TiagoFeedback feedback_; // storing Tiago feedback_ during its journey
		ir2324_group_24::TiagoResult result_; // vector storing the results to send to Node_A
		std::string action_name_; //  storing the name of the node: Node_B
		std::vector<int> goal_; // vector storing the AprilTags ID's to be found
        std::vector<double> scan_ranges;// containing the values of the scanner
        std::vector<geometry_msgs::Point> tiago_path; // vector to store Tiago positions while moving
        std::deque<double> exploration_memory; // deque to keep memory and to not go back in the same path
        std::vector<geometry_msgs::PoseStamped> found_IDs_poses; // vector to store the poses of the found IDs
        geometry_msgs::PoseStamped table_pose_map; // hard-coded position of the table wrt map 
        geometry_msgs::PoseStamped corridor_map; // corridor position
        double scan_angle_increment; // minimum increment in the angles of the scanner
        double scan_angle_min; // initial angle of the scanner
        ros::Publisher vel_cmd_pub; // publisher for velocity commands
        ros::Publisher tilt_cam_pub; // publisher for the initial tilt of the camera
        ros::Subscriber laser_scan_sub; // subscriber for laser scanner topic
        ros::Subscriber tag_sub; // subscriber for apriltag detection
        ros::Subscriber path_sub; // subscriber to odom to get tiago_path
        image_transport::Subscriber image_sub; // subscriber for camera 
        double explore_vector_angle; // angle of explore vector
        double explore_vector_module; // distance of the explore vector
        bool found_all_aprilTags; // it's true when all AprilTags are found!
	    bool corridor; // it's true when Tiago is in the corridor
        bool bad_view; // it's true when Tiago cannot find an explore vector
        bool remember_tiago_path; // it's true when Tiago needs to remember it's path
        bool avoid_rotation; // it's true when Tiago can avoid the 360 rotation to find more AprilTags

        //------------------------ CALLBACK FUNCTIONS -----------------------------

        // CallBack for the Communication of Tiago 
        // Action with Node_A
        // _____________________________________________
        // this callback will contain all Tiago journey 
        // towards the goal, first retrieves the aprilTags 
        // IDs, then finally activates exploration mode 
        // to start moving Tiago. When the goal is achieved, 
        // exploration mode terminates, The first call 
        // of the callback waits for the end of the 1Hz
        // current loop and sets the goal as succeded to
        // Node_A, which terminates it. 
        void tiagoActionCB(const ir2324_group_24::TiagoGoalConstPtr &goal) {
            // fix the rate at 1 Hz (meaning: "check if Tiago found all AprilTags every sec")
            ros::Rate r(1);

            // retrieve the IDs first from Node_A
            ROS_INFO("Received AprilTag IDs:");
            for (int i = 0; i < goal->apriltag_ids.size(); i++) {
                goal_.push_back(goal->apriltag_ids[i]);
                ROS_INFO("ID: %ld", goal->apriltag_ids[i]);
            }
            feedback("Tiago received all the AprilTag IDs to be found."); // send feedback to Node_A

            // INITIALIZE CAMERA 
            initializeCamera(); // tilt the camera to search for AprilTags first

            // ACTIVATE EXPLORATION MODE!
            explorationMode(); // This call will contain Tiago's exploration until goal's reached
        
            // wait for current loop to finish (1Hz)
            r.sleep();

            // defining the results to send to Node_A
            for(geometry_msgs::PoseStamped pose : found_IDs_poses){
                result_.aprilTags_poses.push_back(pose);
            }
            // Add the header (for consistency across the result message)
            result_.header.stamp = ros::Time::now();
            result_.header.frame_id = "map";

            // sets the goal as succeded to Node_A, which terminates!
            as_.setSucceeded(result_);
        }

        // CallBack for LaserScan 
        // _____________________________________________
        // this updates scan_ranges with the laser scan
        // latest measurements. (every 10 Hz)
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            // clear
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
        // ______________________________________________
        // Callback that continuously search for AprilTags
        // in Tiago's camera and compute their position 
        // wrt map frame (20 Hz)
        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
            // If No AprilTags found!
            if (msg->detections.empty()) {
                // return to terminate the call
                return;
            }

            // if an AprilTag was found, get the informations about the detection
            int id = msg->detections[0].id[0]; // AprilTag ID found
            double x = msg->detections[0].pose.pose.pose.position.x;
            double y = msg->detections[0].pose.pose.pose.position.y;
            double z = msg->detections[0].pose.pose.pose.position.z;
            
            // Search for the found ID in the goal_ vector
            // containing all aprilTags to find
            auto it = std::find(goal_.begin(), goal_.end(), id); 
            // update the remaining AprilTag IDs in goal_
            // if present in the vector remove it!
            if (it != goal_.end()) {
                goal_.erase(it); // Remove the found AprilTag

                // Conversion of AprilTag pose from camera_frame to base_link
                // set frame transformer
                tf2_ros::Buffer tfBuffer; // buffer for look up transform
                // set frame transformer 
                tf2_ros::TransformListener tfListener(tfBuffer);
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                ros::Duration timeout(1.0); 
                // get the pose of the AprilTag wrt Tiago
                geometry_msgs::PoseStamped april_tag_pose_camera;
                april_tag_pose_camera.pose.position.x = x;
                april_tag_pose_camera.pose.position.y = y;
                april_tag_pose_camera.pose.position.z = z;
                april_tag_pose_camera.pose.orientation.x = msg->detections[0].pose.pose.pose.orientation.x;
                april_tag_pose_camera.pose.orientation.y = msg->detections[0].pose.pose.pose.orientation.y;
                april_tag_pose_camera.pose.orientation.z = msg->detections[0].pose.pose.pose.orientation.z;
                april_tag_pose_camera.pose.orientation.w = msg->detections[0].pose.pose.pose.orientation.w;

                geometry_msgs::PoseStamped april_tag_pose_map;
                // get the camera frame to base_link transformation
                for (int i = 0; i < 3; ++i) { // Retry 3 times
                    try {
                        transformStamped = tfBuffer.lookupTransform("map", "xtion_rgb_frame", ros::Time(0), timeout);
                        tf2::doTransform(april_tag_pose_camera, april_tag_pose_map, transformStamped);
                        break; // Exit loop on success
                    } catch (tf2::TransformException& ex) {
                        ROS_WARN("Retry %d: Could not get transform for camera transformations: %s", i + 1, ex.what());
                        ros::Duration(0.5).sleep(); // Wait before retrying
                    }
                }
                // store the pose wrt map with its ID
                april_tag_pose_map.header.seq = id; 
                found_IDs_poses.push_back(april_tag_pose_map);
                // send the found aprilTag as feedback to Node_A
                feedback("[NEW APRILTAG FOUND] ID: " + std::to_string(id) + ", Position: [" + 
                        std::to_string(april_tag_pose_map.pose.position.x) + ", " + 
                        std::to_string(april_tag_pose_map.pose.position.y) + "]");
                if(goal_.size()!=0){
                        feedback("Tiago has still "+ std::to_string(goal_.size()) +" AprilTags to find");
                }
            }

            // also check if Tiago found all aprilTags terminating the program if so
            if(goal_.size()==0){
                found_all_aprilTags = true;
            }
        }

        // CallBack to display Tiago's view 
        // ______________________________________________
        // [VISUAL PURPOSE ONLY]
        // show the current RGB image (30Hz)
        void tiagoEyesCallback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                // create the mat object to store Tiago's camera current image
                cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
                // let's downscale the image for visual purposes
                int down_width = img.cols/2; 
                int down_height = img.rows/2;
                cv::Mat resized_down;
                //resize down
                resize(img, resized_down, cv::Size(down_width, down_height), cv::INTER_LINEAR);
                        // Display the image 
                        cv::imshow("Tiago Eyes", resized_down);
                        cv::waitKey(1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
            }
        }

        // CallBack to store Tiago's path
        // ______________________________________________
        // stores Tiago's positions when a boolean is 
        // set to true (50Hz)
        void pathCB(const nav_msgs::Odometry::ConstPtr& msg) {
            // the remembering is triggered by remember_tiago_path
            if(remember_tiago_path){
                geometry_msgs::Point current_pos;
                // Extract Tiago's position wrt map frame
                current_pos.x = msg->pose.pose.position.x;
                current_pos.y = msg->pose.pose.position.y;
                current_pos.z = 0; // planar robot
                // store it in the vector
                tiago_path.push_back(current_pos);
            }
        }

        
        //------------------------ METHODS ---------------------------------


        // FeedBack method
        // ______________________________________________________
        // to update tiago_status and publish the feedback to 
        // Node_A
        void feedback(std::string status){
            // update the feedback to publish 
		    feedback_.robot_status = status;
            // publish feedback to Node_A
            as_.publishFeedback(feedback_);
        }

        // Camera Initialization method 
        // ___________________________________________________
        // method to initialize the camera, tilting the camera 
        // enough to view the AprilTags while exploring
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
            point.positions = {0.0,-1.0};
            point.time_from_start = ros::Duration(1.0);
            tilt_cmd.points.push_back(point);
            // send the tilt command to Tiago
            tilt_cam_pub.publish(tilt_cmd);
            // wait for Tiago to incline the camera ..
            cam_init_r.sleep();
            // publish feedback
            feedback("[Camera-Initialized]");
        }

        // Exploration Memory method
        // ___________________________________________________
        // method to remember Tiago's path after pathCB is 
        // activated 
        void explorationMemory() {
            // don't remember if no path is available
            if(tiago_path.empty() ){ 
                return;
            }
            // don't remember it if the path is  too short
            double x_lenght = std::abs(tiago_path.back().x - tiago_path.front().x);
            double y_length = std::abs(tiago_path.back().y - tiago_path.front().y);
            if(x_lenght < 0.5 && y_length < 0.5){ 
                return;
            }

            // Calculate the vector from the last point to the first point
            // to get the "not go back direction"
            geometry_msgs::Point vector;
            vector.x = tiago_path.back().x - tiago_path.front().x;
            vector.y = tiago_path.back().y - tiago_path.front().y;
            vector.z = 0;
            double angle = atan(vector.y/vector.x);
            if(angle > M_PI){
                angle -=  2 * M_PI;
            }
            if(angle < -M_PI){
                angle +=  2 * M_PI;
            }
            // finally take opposite direction to not go back
            if(angle<0){
                angle = angle + M_PI;
            }
            else if(angle>0){
                angle = angle - M_PI;
            }
            // when in corridor mode, Tiago also remembers the corridor
            // position, to avoid going inside the corridor if already 
            // been there
            if(corridor){ 
                corridor_map.pose.position.x = tiago_path.back().x;
                corridor_map.pose.position.y = tiago_path.back().y;
                corridor_map.pose.position.z = 0;
            }
            // always remember the path direction 
            else{
                exploration_memory.push_front(angle);  // Add to the front
                exploration_memory.pop_back(); 
            }  
            return; 
        }

        // EXPLORING VECTOR method
        // __________________________________________
        // returns an exploring vector based on Tiago
        // current perception
        void exploringVector(double ESCAPE_DISTANCE_THRESHOLD,double ESCAPE_CLUSTER_THRESHOLD){
            // cluster scan to detect explore vectors
            std::vector<std::vector<std::pair<double,double>>> clusters;
            // Variables to keep track of clusters
            std::vector<std::pair<double,double>> current_cluster;
            size_t previous_index = 0;
            // Loop through scan_ranges to identify clusters
            for (size_t i = 0; i < scan_ranges.size(); ++i) {
                double distance = scan_ranges[i];
                double angle = scan_angle_min + i * scan_angle_increment;
                // Check if the distance is an explore distance
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
                    current_cluster.push_back(std::make_pair(angle,distance));
                    previous_index = i;  // Update the previous index
                } 
            }
            // Add the last cluster if it exists
            if (!current_cluster.empty()) {
                clusters.push_back(current_cluster);
                current_cluster.clear();
            }
            
            // check if no clusters found and set the bad view status
            if (clusters.empty()) {
                bad_view = true;
                return;
            }

            // choose the BEST exploring vector

            // update Tiago's memory
            explorationMemory();

            // compute transformations before-hand because
            // tiago is not moving or rotating here
            // initialize buffer and listener
            tf2_ros::Buffer tfBuffer; 
            tf2_ros::TransformListener tfListener(tfBuffer);
            // set frame transformer
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            ros::Duration timeout(1.0); 
            // Wait for transform from base_link to map to be available
            for (int i = 0; i < 3; ++i) { // Retry 3 times if fails
                try {
                    transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), timeout);
                    break; // Exit loop on success
                } 
                catch (tf2::TransformException& ex) {
                    ros::Duration(0.5).sleep(); // Wait before retrying
                }
            }
            // set frame transformer
            tf2_ros::TransformListener tfListenerInv(tfBuffer);
            geometry_msgs::TransformStamped invTransformStamped;
            invTransformStamped.header.stamp = ros::Time::now();
            // Wait for transform from map to base_link to be available
            for (int i = 0; i < 3; ++i) { // Retry 3 times if fails
                try {
                    invTransformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0), timeout);
                    break; // Exit loop on success
                } 
                catch (tf2::TransformException& ex) {
                    ros::Duration(0.5).sleep(); // Wait before retrying
                }
            }

            // compute table position wrt Tiago
            geometry_msgs::PoseStamped table_pose_tiago;
            geometry_msgs::PoseStamped corridor_tiago;
            // Transform the pose from map to base_link
            tf2::doTransform(table_pose_map, table_pose_tiago, invTransformStamped);
            tf2::doTransform(corridor_map, corridor_tiago, invTransformStamped);  
            
            // discriminate found exploring vectors
            // initialize objects
            std::vector<double> valid_explore_angles;
            std::vector<double> valid_explore_angles_map;
            std::vector<double> valid_explore_modules;
            double explore_angle_map;
            double explore_angle;
            double explore_module = 0.0;
            geometry_msgs::PoseStamped explore_vector;
            geometry_msgs::PoseStamped explore_vector_map;
            // analyze each cluster of each explore vector candidate
            for(std::vector<std::pair<double,double>> cluster : clusters){
                if(cluster.size()>30){ // avoid explore vectors with too small clusters
                    // compute explore vector
                    explore_angle = 0.0;
                    explore_module = 9999.0;
                    for(std::pair<double,double> x : cluster){
                        // take average of the angle
                        explore_angle += x.first;
                        if(x.second<explore_module) explore_module = x.second;
                    }
                    explore_angle = explore_angle/cluster.size();
                    // transform it to map frame (assume tiago is not moving)
                    explore_vector.pose.position.x = explore_module*cos(explore_angle);
                    explore_vector.pose.position.y = explore_module*sin(explore_angle);
                    explore_vector.pose.position.z = 0;
                    // Current position of Tiago in the map frame
                    double current_x = transformStamped.transform.translation.x;
                    double current_y = transformStamped.transform.translation.y;
                    // Extract the robot's current yaw from the transform's quaternion
                    tf2::Quaternion current_orientation_q(
                        transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w
                    );
                    double roll, pitch, current_yaw;
                    tf2::Matrix3x3(current_orientation_q).getRPY(roll, pitch, current_yaw);
                    // Calculate explore vector orientation by adding explore_vector_angle to current_yaw
                    explore_angle_map = current_yaw + explore_angle;

                    // CHECK memory

                    bool is_valid = true;
                    for (double mem_angle : exploration_memory) {
                        // correction for angles from +- 135
                        if(mem_angle < - 3*M_PI/4  && explore_angle_map > 3*M_PI/4 && mem_angle < 9000){
                            if(explore_angle_map > mem_angle-(M_PI/4) +2 * M_PI){
                                is_valid = false; 
                                break;
                            }
                        }
                        if(mem_angle >  3*M_PI/4  && explore_angle_map < - 3*M_PI/4 && mem_angle < 9000){
                            if(explore_angle_map < mem_angle+(M_PI/4) -2 * M_PI){
                                is_valid = false; 
                                break;
                            }
                        }
                        // if memory is void 9999, don't consider it 
                        if (std::abs(mem_angle - explore_angle_map) < M_PI/4 && mem_angle < 9000 ) { 
                            is_valid = false; 
                            break;
                        }
                    }

                    // CHECK table 

                    // Extract the position of the table in the base_link frame
                    double x = table_pose_tiago.pose.position.x;
                    double y = table_pose_tiago.pose.position.y;
                    double table_distance = sqrt(pow(x,2) + pow(y,2));
                    double angle_threshold;
                    // set different thresholds based on table distance from Tiago
                    if(table_distance < 1.5){
                        angle_threshold = M_PI/2+M_PI/8;
                    }
                    else if (table_distance < 2){
                        angle_threshold = M_PI/8;
                    }
                    else{
                        angle_threshold = M_PI/16;
                    }
                    // Compute the angle of the vector
                    double table_angle = std::atan2(y, x);
                    if(std::abs(explore_angle - table_angle)< angle_threshold&& table_distance < ESCAPE_DISTANCE_THRESHOLD+0.5 ){
                        is_valid = false;
                    }

                    // CHECK corridor

                    // Extract the position of the corridor in the base_link frame
                    double corridor_distance = sqrt(pow(corridor_tiago.pose.position.x,2) + pow(corridor_tiago.pose.position.y,2));
                    double corridor_angle = std::atan2(corridor_tiago.pose.position.y, corridor_tiago.pose.position.x);
                    if(corridor_distance < 3 && std::abs(explore_angle - corridor_angle)< M_PI/6){
                        is_valid = false;
                    }

                    // if explore vector candidate succeded every check  select it as valid
                    if(is_valid == true){
                        valid_explore_angles.push_back(explore_angle);
                        valid_explore_modules.push_back(explore_module);
                        valid_explore_angles_map.push_back(explore_angle_map);
                    }
                }
            }

            // if no valid explore vectors set bad view status
            if(valid_explore_angles.empty()){
                bad_view = true;
                return;
            }

            // if not choose randomically one
            // Seed the random number generator (once per program execution)
            static bool is_rand_seeded = false;
            if (!is_rand_seeded) {
                srand(static_cast<unsigned int>(time(0)));
                is_rand_seeded = true;
            }
            // Randomly select one explore angle and store it
            size_t random_index = rand() % valid_explore_angles.size();
            explore_vector_angle = valid_explore_angles[random_index];
            explore_vector_module = ESCAPE_DISTANCE_THRESHOLD; // default distance 

            // avoid going on default distance if obstacles are present within it
            size_t i_explore = scan_ranges.size()+static_cast<size_t>(explore_vector_angle/scan_angle_increment);
            if(scan_ranges[i_explore]<(ESCAPE_DISTANCE_THRESHOLD) && scan_ranges[i_explore]>1){ // also avoid tiago
                explore_vector_module = scan_ranges[i_explore]-0.2;
            }

            bad_view = false; // deactivate bad view status because Tiago found the best exploring vector
            return; 
        }

        // NEXT EXPLORE VECTOR method
        // __________________________________________
        // Tiago calls exploringVector multiple times
        // until one is found, every time an exporing 
        // vector is not found, calls lookAround()
        // to do a random rotation to search for more 
        // directions
        void exploringVectorFinder()
        {
            ros::Rate rate(10);  // 10 Hz
            bool found = false;
            avoid_rotation = false;
            while(ros::ok() && !found && !found_all_aprilTags){
                feedback("Tiago is searching for the next exploration ...");
                // compute next exploring vector
                exploringVector(3.5,1); // 3.5 meters, 1 scan_angle_increment
                
                // if not valid, rotate randomically
                if(bad_view){
                    // rotate
                    feedback("Tiago cannot find the next exploration");
                    lookAround();
                    // 360 already done in that position, Tiago is just looking for an exploring vector
                    avoid_rotation = true; 
                    rate.sleep();
                }
                else{
                    // if valid, go out the while
                    found = true;
                    feedback("Tiago has found the next exploration!");
                    avoid_rotation = false;
                    rate.sleep();
                }
                // clear past path for next path storage
                tiago_path.clear();
            }
            
        }

        // LOOK AROUND method 
        // _____________________________________
        // random rotation of 45 degrees to make
        // Tiago look to other directions to find
        // an exploring vector
        void lookAround()
        {
            feedback("Tiago is turning to find the next exploration ...");
            geometry_msgs::Twist twist;
            double angle = M_PI / 2;
            ros::Rate rate(50);  // 50 Hz
            srand(static_cast<unsigned int>(time(0)));
            // Randomly choose speed as -2.0 or 2.0
            double angular_speed = (rand() % 2 == 0) ? 2.0 : -2.0;
            double rotation_duration = angle / angular_speed;  // Time to complete the rotation
            ros::Time start_time = ros::Time::now();
            while (ros::Time::now() - start_time < ros::Duration(rotation_duration)) {
                twist.angular.z = angular_speed;  // randomly clock-wise or counter clock-wise
                vel_cmd_pub.publish(twist);
                rate.sleep();
            }
            // Stop Tiago after completing the rotation
            twist.angular.z = 0.0;
            vel_cmd_pub.publish(twist);
            // Wait for a short duration after stopping
            ros::Duration(2.0).sleep(); 
        }

        // 360 ROTATION method 
        // _____________________________________
        // complete rotation needed for Tiago to 
        // look for AprilTags around
        void rotation()
        {
            if(avoid_rotation){
                return;
            }
            feedback("Tiago is looking around for AprilTags ...");
            geometry_msgs::Twist twist;
            ros::Rate rate(50);  // 50 Hz
            double angular_speed = 2.0;  // rad/s
            double rotation_duration = 2 * M_PI / angular_speed;  // Time to complete 360° rotation
            ros::Time start_time = ros::Time::now();

            while (ros::Time::now() - start_time < ros::Duration(rotation_duration)) {
                twist.angular.z = angular_speed;  // Rotate counter-clockwise
                vel_cmd_pub.publish(twist);
                rate.sleep();
            }

            // Stop the robot after completing the rotation
            twist.angular.z = 0.0;
            vel_cmd_pub.publish(twist);
            if(!found_all_aprilTags)
                feedback("Tiago is ready to explore!");
            // Wait for a short duration after stopping
            ros::Duration(2.0).sleep();  // Wait for 2 second
        }

        // CORRIDOR MODE method
        // ________________________________________________________
        // motion control law to assign velocity commands from scan 
        // measurements to Tiago in a reactive paradigm fashion,
        // (while being in a corridor) 
        void corridorMode()
        {
            feedback("Tiago has found a corridor");
            feedback("[CORRIDOR MODE] activated");
            // set the rate of commands to Tiago
            ros::Rate r_corridor(10);
            // start the reactive control
            double right_future_side_distance;
            double left_future_side_distance;
            remember_tiago_path = true; // start storing the path 
            while(ros::ok() && corridor) 
            {
                // Create the Twist message for velocity command
                geometry_msgs::Twist next_cmd_vel;

                // Calculate future side distances based on scan ranges
                left_future_side_distance = scan_ranges[(scan_ranges.size()/2) - static_cast<size_t>((M_PI/8)/scan_angle_increment)] * sin(M_PI/4);
                right_future_side_distance = scan_ranges[(scan_ranges.size()/2) + static_cast<size_t>((M_PI/8)/scan_angle_increment)] * sin(M_PI/4);
                
                // Out of corridor condition
                if(right_future_side_distance+left_future_side_distance>3){ 
                    // Stop Tiago when out of the corridor
                    next_cmd_vel.linear.x = 0;
                    vel_cmd_pub.publish(next_cmd_vel);
                    ros::Duration(2.0).sleep();  // Wait for 2 second
                    corridor = false;
                    feedback("Tiago has reached the end of the corridor");
                }
                else{
                    // set reactive control
                    double correction = 0.5 * (right_future_side_distance - left_future_side_distance);
                    // Set forward speed
                    next_cmd_vel.linear.x = 4 - std::abs(correction); 
                    // Set the angular velocity
                    next_cmd_vel.angular.z = correction; // Turn towards the greater distance side
                    // Publish the command
                    vel_cmd_pub.publish(next_cmd_vel);                 
                    // Sleep to maintain the loop rate
                    r_corridor.sleep();
                }
            }
            remember_tiago_path = false; // stop storing the path
            feedback("[CORRIDOR MODE] de-activated");
                
        }

        // EXPLORATION MODE method
        //__________________________________________________
        // this method contains the whole Tiago's journey
        // towards achieving the goal: finding all AprilTags
        // contained in goal_.
        void explorationMode(){
            // Set the rate 
            ros::Rate rate(1);
            // initialize move_base action client
            MoveBaseClient ac_("move_base",true);
            // wait for the action server to come up
            while(!ac_.waitForServer(ros::Duration(5.0))){
                feedback("Waiting for the move_base action server to come up");
            }
            feedback("Tiago is ready to search for the given AprilTags!");
            // EXPLORATION MODE journey
            // ends when all AprilTags are found  
            while(!found_all_aprilTags && ros::ok()){ 
                // 360 ROTATION with move_base
                // "checking around for AprilTags"  
              	rotation(); 
                if(found_all_aprilTags) break; // go out of the while if all apriltags found

                // only when in the corridor -> motion control law 
                if(corridor)
                {
                    corridorMode();
                }

                // find next exploring vector
                exploringVectorFinder();
                if(found_all_aprilTags) break; // go out of the while if all apriltags found

                // tell move_base to go to that position

                // set frame transformer
                tf2_ros::Buffer tfBuffer; // buffer for look up transform
                tf2_ros::TransformListener tfListener(tfBuffer);
                geometry_msgs::TransformStamped transformStamped;
                ros::Duration timeout(2.0); // Adjust as needed
                // Wait for transform to be available
                transformStamped.header.stamp = ros::Time::now();
                // Wait for transform from base_link to map to be available
                for (int i = 0; i < 3; ++i) { // Retry 3 times if fails
                    try {
                        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), timeout);
                        break; // Exit loop on success
                    } 
                    catch (tf2::TransformException& ex) {
                        ros::Duration(0.5).sleep(); // Wait before retrying
                    }
                }
                // Current position of Tiago in the map frame
                double current_x = transformStamped.transform.translation.x;
                double current_y = transformStamped.transform.translation.y;
                // Extract the robot's current yaw from the transform's quaternion
                tf2::Quaternion current_orientation_q(
                    transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w
                );
                double roll, pitch, current_yaw;
                tf2::Matrix3x3(current_orientation_q).getRPY(roll, pitch, current_yaw);
                // Calculate goal orientation by adding explore_vector_angle to current_yaw
                double goal_yaw = current_yaw + explore_vector_angle;
                // Calculate goal position in map frame
                double goal_x = current_x + explore_vector_module * cos(goal_yaw);
                double goal_y = current_y + explore_vector_module * sin(goal_yaw);
                // Create a MoveBaseGoal message
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map"; // Set frame to map
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = goal_x;
                goal.target_pose.pose.position.y = goal_y;
                goal.target_pose.pose.position.z = 0.0; // Ensure it's in the 2D plane
                // Set orientation based on the calculated goal_yaw
                tf2::Quaternion q;
                q.setRPY(0, 0, goal_yaw); // Roll and pitch are 0, yaw is goal_yaw
                goal.target_pose.pose.orientation.x = q.x();
                goal.target_pose.pose.orientation.y = q.y();
                goal.target_pose.pose.orientation.z = q.z();
                goal.target_pose.pose.orientation.w = q.w();

                // Send the goal to move_base
                ac_.sendGoal(goal);
                feedback("Tiago is exploring ...");
                remember_tiago_path = true; // start storing tiago's path
                // wait for 7 s maximum for Tiago to reach goal pose
                while(ac_.waitForResult(ros::Duration(7.0)))
		{
			if(found_all_aprilTags) break; // go out of the while if all apriltags found
		}	
                remember_tiago_path = false; // stop storing tiago's path 
                feedback("Tiago has found a good spot to look for AprilTags!");

                // If the goal is not achieved within 7 seconds, cancel it
                ac_.cancelGoal();
                // Sleep to enforce the rate 
                rate.sleep();
            }
            // exploration mode deactivated: goal achieved!
            feedback("Tiago has found all AprilTags!");
            return;
        }



	public:
		// Constructor 
		TiagoAction(std::string name):as_(nh_, name, boost::bind(&TiagoAction::tiagoActionCB, this, _1), false), action_name_(name)
		{
            // Initialize parameters
            exploration_memory = {9999}; // initialized to a non valid memory value
            explore_vector_angle = 0.0; // initialize explore vector
            explore_vector_module = 0.0; // initialize explore vector
            table_pose_map.pose.position.x = 7.579; // hard-coded x position of the table wrt map
            table_pose_map.pose.position.y = - 1.6921, // hard-coded y position of the table wrt map
            table_pose_map.pose.position.z = 0; // hard-coded z position of the table wrt map
            found_all_aprilTags = false; // no AprilTags found yet, start looking for them
	        corridor = true; // assumption: there's a corridor in front of the initial position
            bad_view = false; // initialize bad view status to false
            remember_tiago_path = false; // initialize remember path status to false
            avoid_rotation = false; // initialize avoid rotation to false
          
            // Subscriber for LaserScan topic /scan (messages rate: 10 Hz)
            laser_scan_sub = nh_.subscribe("/scan", 10, &TiagoAction::laserScanCallback, this);
            // Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_transport::ImageTransport it(nh_); // image transport for the camera topic
            image_sub = it.subscribe("/xtion/rgb/image_color", 100, &TiagoAction::tiagoEyesCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            tag_sub = nh_.subscribe("/tag_detections", 10, &TiagoAction::tagDetectionCallback, this);
            // SUbscriber to odom
            // Subscribe to odometry topic
            path_sub = nh_.subscribe("/mobile_base_controller/odom", 10, &TiagoAction::pathCB, this);
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
