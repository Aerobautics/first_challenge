/*
 * Author: Aerobotics (Stewart Nash)
 * File: first_challenge_integrated.cpp
 * Description: Node for first drone challenge with integrated
 * computer vision and movement functions.
 */
#include "controller.h"

//#include "spacestate.h"

// Take functionality out of main for multiple methods of use.
//void method_search(int argc, char* argv[]);
//void method_waypoint(int argc, char* argv[]);

int main(int argc, char* argv[])
{
	Controller controller;
	//method_search(argc, argv);
	//method_waypoint(argc, argv);
	
	controller.waypointNavigation(argc, argv);

	return 0;
}

/*
void method_search(int argc, char* argv[]) {
	SpaceState spaceState;
	isPoseAcquired = false;
	ros::init(argc, argv, "first_challenge_node");
	ros::NodeHandle nodeHandle;

	ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &local_pos_cb);
	ros::Publisher local_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	image_transport::ImageTransport bottomImageTransport(nodeHandle);
	image_transport::ImageTransport frontImageTransport(nodeHandle);
	image_transport::Subscriber bottomImageSubscriber;
	image_transport::Subscriber frontImageSubscriber;
	image_transport::Publisher imagePublisher;

	bottomImageSubscriber = bottomImageTransport.subscribe("/iris_1/camera_down/image_raw", 1, &bottomImageCallback);
	frontImageSubscriber = frontImageTransport.subscribe("/iris_1/camera_forward/image_raw", 1, &frontImageCallback);
	//imagePublisher = imageTransport.advertise("/image_converter/output_video", 1);
	//subscriber = nodeHandle.subscribe<sensor_msgs::Image>("image_raw", 32, &rawImageCallback);

	cv::VideoCapture capture;
	
	geometry_msgs::PoseStamped pose;

	double pose_x, pose_y;
	double desired_x, desired_y;	
	unsigned long elapsedTime = 0;

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
	
	// Wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	// PX4 Pro Flight Stack operates in aerospacee NED coordinate frame
	// MAVROS translates to standard ENU frame
	while (!isPoseAcquired) {

	}

	//image = cv::imread(FILE_NAME);
	//if (!image.data) {
	//	printf("No image data \n");
	//	return -1;
	//}

	cv::namedWindow(PROCESSING_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(SEARCH_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(BOTTOM_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(FRONT_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::resizeWindow(BOTTOM_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	cv::resizeWindow(FRONT_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	cv::resizeWindow("Image Processing", SD_WIDTH, SD_HEIGHT);
	cv::resizeWindow("Search Space", SD_WIDTH, SD_HEIGHT);
	//cv::resizeWindow(PROCESSING_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	//cv::resizeWindow(SEARCH_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);

	std::cout << "Starting position (" << current_pose.pose.position.x << ", ";
	std::cout << current_pose.pose.position.y << ")" << std::endl;
	
	//pose.pose.position.x = current_pose.pose.position.x; // 0?
	//pose.pose.position.y = current_pose.pose.position.y; // 0?
	//pose.pose.position.z = current_pose.pose.position.z; // 2?
	//pose.pose.orientation.x = current_pose.pose.orientation.x; // 0?
	//pose.pose.orientation.y = current_pose.pose.orientation.y; // 0?
	//pose.pose.orientation.z = current_pose.pose.orientation.z; // -0.99?
	//pose.pose.orientation.w = -current_pose.pose.orientation.w; // -0.04?
	
	pose.pose.position.x = 0; // 0?
	pose.pose.position.y = 0; // 0?
	pose.pose.position.z = 2; // 2?
	pose.pose.orientation.x = 0; // 0?
	pose.pose.orientation.y = 0; // 0?
	pose.pose.orientation.z = -0.99; // -0.99?
	pose.pose.orientation.w = -0.04; // -0.04?
	pose.pose.position.z = 2;		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	// Initialize spacestate values -> move this to header
	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}
	spaceState.initializeSpace(pose.pose.position.x + X_OFFSET, pose.pose.position.y + Y_OFFSET);

	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
	first_challenge_set_mode.request.custom_mode = "OFFBOARD";
	// Switch to Offboard mode, arm quadcopter, send service calls every 5 seconds
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	if (SAVE_VIDEO) {
		video = cv::VideoWriter("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(video_width, video_height)); 
	}

	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if( set_mode_client.call(first_challenge_set_mode) && first_challenge_set_mode.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
				if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		++elapsedTime;
		
		//if (isPoseAcquired) {
		//	pose_x = current_pose.pose.position.x;
		//	pose_y = current_pose.pose.position.y;
		//} else {
		//	pose_x = pose.pose.position.x;
 		//	pose_y = pose.pose.position.y;
		//}
		
		pose_x = current_pose.pose.position.x + X_OFFSET;
		pose_y = current_pose.pose.position.y + Y_OFFSET;		
		spaceState.updateSpace(pose_x, pose_y);	
		if (spaceState.moveTo(pose_x, pose_y, elapsedTime)) {
			pose.pose.position.x = pose_x - X_OFFSET;
			pose.pose.position.y = pose_y - Y_OFFSET;
			std::cout << "moveTo: (" << pose.pose.position.x + X_OFFSET;
			std::cout << ", " << pose.pose.position.y + Y_OFFSET;
			std::cout << ", " << pose.pose.position.z << ")" << std::endl;
			elapsedTime = 0;
		} else {

		}
		spaceState.displaySpace();
		processImage(frame);
		isPoseAcquired = false;
		//if (elapsedTime >= DWELL_TIME) {
		//	elapsedTime = 0;
		//	std::cout << "main: Restarting elapsedTime" << std::endl;
		//}
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	cv::destroyWindow("Image Processing");
	cv::destroyWindow("Search Space");
	//cv::destroyWindow(PROCESSING_WINDOW_NAME);
	//cv::destroyWindow(SEARCH_WINDOW_NAME);
	if (SAVE_VIDEO) {
		video.release();
	}
}
*/

/*
void method_waypoint(int argc, char* argv[]) {
	Waypoint waypoint;
	SpaceState spaceState;
	isPoseAcquired = false;
	ros::init(argc, argv, "first_challenge_node");
	ros::NodeHandle nodeHandle;

	ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_cb);
	ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &local_pos_cb);
	ros::Publisher local_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	image_transport::ImageTransport bottomImageTransport(nodeHandle);
	image_transport::ImageTransport frontImageTransport(nodeHandle);
	image_transport::Subscriber bottomImageSubscriber;
	image_transport::Subscriber frontImageSubscriber;
	image_transport::Publisher imagePublisher;

	bottomImageSubscriber = bottomImageTransport.subscribe("/iris_1/camera_down/image_raw", 1, &bottomImageCallback);
	frontImageSubscriber = frontImageTransport.subscribe("/iris_1/camera_forward/image_raw", 1, &frontImageCallback);

	cv::VideoCapture capture;
	
	geometry_msgs::PoseStamped pose;

	double pose_x, pose_y;
	double desired_x, desired_y;	
	unsigned long elapsedTime = 0;

	// The setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);
	
	// Wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	// PX4 Pro Flight Stack operates in aerospacee NED coordinate frame
	// MAVROS translates to standard ENU frame
	while (!isPoseAcquired) {

	}

	//cv::namedWindow(PROCESSING_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(SEARCH_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::namedWindow(BOTTOM_WINDOW_NAME, cv::WINDOW_NORMAL);
	//cv::namedWindow(FRONT_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::resizeWindow(BOTTOM_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	//cv::resizeWindow(FRONT_WINDOW_NAME, SD_WIDTH, SD_HEIGHT);
	//cv::resizeWindow("Image Processing", SD_WIDTH, SD_HEIGHT);
	cv::resizeWindow("Search Space", SD_WIDTH, SD_HEIGHT);

	std::cout << "Starting position (" << current_pose.pose.position.x << ", ";
	std::cout << current_pose.pose.position.y << ")" << std::endl;

	pose.pose.position.x = 0; // 0?
	pose.pose.position.y = 0; // 0?
	pose.pose.position.z = 2; // 2?
	pose.pose.orientation.x = 0; // 0?
	pose.pose.orientation.y = 0; // 0?
	pose.pose.orientation.z = -0.99; // -0.99?
	pose.pose.orientation.w = -0.04; // -0.04?
	pose.pose.position.z = 2;		
	// Send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	// Initialize spacestate values -> move this to header
	for (int i = 0; i < NUMBER_OF_WALLS; i++) {
		forbiddenPoints[i] = getPoints(i);
	}
	spaceState.initializeSpace(pose.pose.position.x / X_SCALE + X_OFFSET, pose.pose.position.y / Y_SCALE + Y_OFFSET);

	// Set custom mode to OFFBOARD
	mavros_msgs::SetMode first_challenge_set_mode;
	first_challenge_set_mode.request.custom_mode = "OFFBOARD";
	// Switch to Offboard mode, arm quadcopter, send service calls every 5 seconds
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	ros::Time last_request = ros::Time::now();

	if (SAVE_VIDEO) {
		video = cv::VideoWriter("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(video_width, video_height));
	}
 
	while(ros::ok()){
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if( set_mode_client.call(first_challenge_set_mode) && first_challenge_set_mode.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
				if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		pose_x = current_pose.pose.position.x / X_SCALE + X_OFFSET;
		pose_y = current_pose.pose.position.y / Y_SCALE + Y_OFFSET;		
		spaceState.updateSpace(pose_x, pose_y);	
		if (waypoint.moveToWaypoint(pose_x, pose_y, elapsedTime)) {
			pose.pose.position.x = pose_x - X_OFFSET;
			pose.pose.position.y = pose_y - Y_OFFSET;
			pose.pose.position.x = X_SCALE * (pose_x - X_OFFSET);
			pose.pose.position.y = Y_SCALE * (pose_y - Y_OFFSET);			
			std::cout << "moveTo: (" << pose.pose.position.x / X_SCALE + X_OFFSET;
			std::cout << ", " << pose.pose.position.y / Y_SCALE + Y_OFFSET;
			std::cout << ", " << pose.pose.position.z << ")" << std::endl;
			elapsedTime = 0;
		} else {
			++elapsedTime;
		}
		spaceState.displaySpace();
		//processImage(frame);
		isPoseAcquired = false;

		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	//cv::destroyWindow("Image Processing");
	cv::destroyWindow("Search Space");

	if (SAVE_VIDEO) {
		video.release();
	}
}
*/
