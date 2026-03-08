// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// include math 
#include <math.h>

using namespace std;

float theta_error;

// turtle pose
turtlesim::Pose leader;
turtlesim::Pose follower1;
turtlesim::Pose follower2;

// goal points
geometry_msgs::Point leader_goal;
geometry_msgs::Point follower1_goal;
geometry_msgs::Point follower2_goal;

// turtle twist
geometry_msgs::Twist follower1_twist;
geometry_msgs::Twist follower2_twist;

// turtle publisher
ros::Publisher follower1_pub;
ros::Publisher follower2_pub;


bool reset;

struct XY{
    float x;
	float y;
};

struct XY pos_err_I;



// declare call back function
void leader_cb(const turtlesim::Pose::ConstPtr& msg)
{
	leader = *msg;
}

void follower_cb1(const turtlesim::Pose::ConstPtr& msg)
{
	follower1 = *msg;
}

void follower_cb2(const turtlesim::Pose::ConstPtr& msg)
{
	follower2 = *msg;
}


// Transform a relative offset given in leader's body frame → world frame,
// then shift by leader's world-frame position to get the absolute goal.
//
// Body → World rotation (rotate by +θ around z-axis):
//   x_w =  x_b * cos(θ) - y_b * sin(θ)
//   y_w =  x_b * sin(θ) + y_b * cos(θ)
// Then add leader world position:
//   goal_world = R(θ) * offset_body + leader_position
void leadertoworld2D(geometry_msgs::Point &follower_goal, turtlesim::Pose &leader)
{
	float temp_x = follower_goal.x;
	float temp_y = follower_goal.y;

	// Rotate offset from leader body frame to world frame
	follower_goal.x = cos(leader.theta) * temp_x - sin(leader.theta) * temp_y;
	follower_goal.y = sin(leader.theta) * temp_x + cos(leader.theta) * temp_y;

	// Shift by leader's current world-frame position
	follower_goal.x += leader.x;
	follower_goal.y += leader.y;
} 


// Rotate a world-frame vector into follower's body frame.
// World → Body rotation (rotate by -θ around z-axis):
//   x_b =  x_w * cos(θ) + y_w * sin(θ)
//   y_b = -x_w * sin(θ) + y_w * cos(θ)
void worldtobody2D(float &x, float &y, float theta)
{
	float x_body =  x * cos(theta) + y * sin(theta);
	float y_body = -x * sin(theta) + y * cos(theta);
	x = x_body;
	y = y_body;
} 


// P control for goal position in world frame 
void Positioncontrol(geometry_msgs::Point &goal, turtlesim::Pose &follower, geometry_msgs::Twist &vel_msg) {

	// P gain tuning parameters
	const float Kp_linear  = 1.5f;
	const float Kp_angular = 4.0f;

	// Error in world (inertia) frame
	pos_err_I.x = goal.x - follower.x;
	pos_err_I.y = goal.y - follower.y;

	// Transform error into follower's body frame
	worldtobody2D(pos_err_I.x, pos_err_I.y, follower.theta);

	// Distance to goal (frame-invariant magnitude)
	float error_norm = sqrt(pow(pos_err_I.x, 2) + pow(pos_err_I.y, 2));

	// Heading error: angle from body x-axis to goal direction
	float error_theta = atan2(pos_err_I.y, pos_err_I.x);

	// Output boundary: clamp max linear speed to 2
	if (error_norm > 2) error_norm = 2;

	// P controller:
	//   linear.x  → drive forward proportional to distance
	//   angular.z → turn proportional to heading error
	vel_msg.linear.x  = Kp_linear  * error_norm;
	vel_msg.angular.z = Kp_angular * error_theta;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_formation");
  	ros::NodeHandle n;

  	// declare publisher & subscriber

	// turtle subscriber
  	ros::Subscriber leader_sub = n.subscribe<turtlesim::Pose>("/turtlesim/leader/pose", 1, leader_cb); 
  	ros::Subscriber follower_sub1 = n.subscribe<turtlesim::Pose>("/turtlesim/follower1/pose", 1, follower_cb1);
	ros::Subscriber follower_sub2 = n.subscribe<turtlesim::Pose>("/turtlesim/follower2/pose", 1, follower_cb2);

	follower1_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/follower1/cmd_vel", 1);
	follower2_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/follower2/cmd_vel", 1);
		


	// setting frequency as 10 Hz
  	ros::Rate loop_rate(10);
	
  	while (ros::ok()){

		/*     define formation of turtle 

				follower2 >
						       leader >
				follower1 >    
		*/

        follower1_goal.x = -1;
        follower1_goal.y = -1;

		follower2_goal.x = -1;
		follower2_goal.y = 1;
		
        // rotate from leader turtle frame to world frame
        leadertoworld2D( follower1_goal, leader);
		leadertoworld2D( follower2_goal, leader);

		//Input your goal_point to your controller
    	Positioncontrol(follower1_goal, follower1, follower1_twist);
    	Positioncontrol(follower2_goal, follower2, follower2_twist);

		//Input your control input(from Pcontrol) to your plant
    	follower1_pub.publish(follower1_twist);
		follower2_pub.publish(follower2_twist);

    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
