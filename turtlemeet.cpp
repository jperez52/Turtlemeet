// This program steers a turtlesim turtle1 toward a randomly spawned turtle named "Tim" and utilized the same commands to have them meet.
// Control considerations are based on Bretl, T.(2007) Control of Many Agents Using Few Instructions, Robotics: Science and Systems, June 27-30 2007.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Kill.h>
#include <iomanip> // for std: setprecision and std::fixed
#include <stdlib.h> // For rand() and RAND_MAX
#include <math.h>       /* sqrt */
#include <std_srvs/Empty.h> // for clearing the screen

//Javier Perez Jr. - 1595083

turtlesim::Pose turtlePose;  // global variable for the turtle's pose.  
turtlesim::Pose timPose;  // global variable for the second turtle's pose (Tim). 
// Note: global variables is generally bad programming practice.
int poseInitialized = 0;

// A callback function. Executed each time a new pose message arrives.
void poseMessageReceived1(const turtlesim::Pose& msg1) {
    // TODO 2b:  copy the msg pose to your global variable
    poseInitialized = 1;
	turtlePose.x = msg1.x;
	turtlePose.y = msg1.y;
	turtlePose.theta = msg1.theta;
	turtlePose.linear_velocity = msg1.linear_velocity;
	turtlePose.angular_velocity = msg1.angular_velocity;
}

void poseMessageReceived2(const turtlesim::Pose& msg2) {
    // TODO 2b:  copy the msg pose to your global variable
    poseInitialized = 2;
	timPose.x = msg2.x;
	timPose.y = msg2.y;
	timPose.theta = msg2.theta;
	timPose.linear_velocity = msg2.linear_velocity;
	timPose.angular_velocity = msg2.angular_velocity;
}

int main(int argc, char ** argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "turtlemeet");
    ros::NodeHandle nh;
    
    // goal location
    float goalx, goaly;
    
    // clear the screen
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clearClient.call(srv); 
	
	// Create a client object for the Spawn service.
  ros::ServiceClient spawnClient
    = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	// Create a client object for the set_pen service.  This
	// needs to know the data type of the service and its
	// name.
	ros::ServiceClient setpenClient
		= nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");	
	
	// Set rate to 10 Hz
	ros::Rate rate(10);
	
	// Create the request and response objects.
	turtlesim::SetPen::Request preq;
	turtlesim::SetPen::Response presp;
	
	rate.sleep();	
		
	// Fill in the request data members.
	preq.r = 200;
	preq.width = 3;
	ROS_INFO_STREAM("Req:"
					<< "r=" << preq.r
					<< "width=" << preq.width);		
	rate.sleep();	
	
	// Call the setpen service
	bool psuccess = setpenClient.call(preq, presp);
	
	// Check for success
	if(psuccess) {
	ROS_INFO_STREAM("Pen is red");
		}
	else {
	ROS_ERROR_STREAM("Failed to set pen");
	}
	
	// Create the request and response objects.
  turtlesim::Spawn::Request sreq;
  turtlesim::Spawn::Response sresp;

  // Fill in the request data members.
  srand(time(NULL));
  sreq.x = rand()%11+1;
  sreq.y = rand()%11+1;
  sreq.theta = 6.47*double(rand())/double(RAND_MAX);
  sreq.name = "Tim";

  // Actually call the service.  This won't return until
  // the service is complete.
  bool ssuccess = spawnClient.call(sreq, sresp);

  // Check for success and use the response.
  if(ssuccess) {
    ROS_INFO_STREAM("Spawned a turtle named "
      << sresp.name);
  } else {
    ROS_ERROR_STREAM("Failed to spawn.");
  }
    // Create a publisher object
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("Tim/cmd_vel", 1000);
	
    // Create a subscriber object
    //TODO 1: setup the callback 
	ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 1000,
    &poseMessageReceived1);
	
	ros::Subscriber sub2 = nh.subscribe("Tim/pose", 1000,
    &poseMessageReceived2);
    
    // Loop at 10 Hz until the node is shut down.
    //ros::Rate rate(10);
	
    // control variables
    float dx, dy, dcos, dsin, angErr, distErr;
    //message to be sent to turtle.  The other four fields, which are ignored by turtlesim, default to zero.
    geometry_msgs::Twist msg1, msg2;
	
    while(ros::ok()) {
		//TODO 3: you need to hand control over to ROS or your callbacks will never be checked (let ROS spin)
		ros::spinOnce();
        //compute control parameters
        dx = timPose.x-turtlePose.x;
        dy = timPose.y-turtlePose.y;
		
		// Calculating the distance error
		distErr = sqrt(pow((timPose.x-turtlePose.x),2)+pow((timPose.y-turtlePose.y),2)); //TODO 4: calculate the distance
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Distance=" << distErr);
		
		// Calculating angle error for u vector based on Bretl 2007
        dcos = cos(turtlePose.theta) - cos(timPose.theta);
        dsin = sin(turtlePose.theta) - sin(timPose.theta);
        angErr = atan2(  (dy*dcos-dx*dsin) , (dx*dcos+dy*dsin));
        
        // control law
        if( fabsf(angErr) > 1.0){
            msg1.linear.x = 0; // if angular error is large, turn in place. 
			msg2.linear.x = msg1.linear.x;
        }else{
            msg1.linear.x = fmin(distErr,1.0);
			msg2.linear.x = msg1.linear.x;
        }
        msg1.angular.z = .3*angErr;  //TODO 6: what happens if you change this control gain from 1/2?
        msg2.angular.z = msg1.angular.z; 
        
        // Publish the message.
        pub1.publish(msg1);
		pub2.publish(msg2);
        
        // Send a message to rosout with the details.
        ROS_INFO_STREAM("vel command:"
            << " linear=" << msg1.linear.x << " angular=" << msg1.angular.z 
            << " pose=("<< turtlePose.x<<","<<turtlePose.y<<","<<turtlePose.theta<<"), angErr="<<angErr);
        
        if(distErr < 0.2 && poseInitialized ){ // return if at the goal.
            return 0;
        }    
        
        // Wait until it's time for another interaction
        rate.sleep();// 
    }
} 








