// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

// include math 
#include <math.h>

using namespace std;

float theta_error;

// turtle pose
turtlesim::Pose turtle;

// goal points
geometry_msgs::Point turtle_goal;

// turtle twist
geometry_msgs::Twist turtle_twist;

// turtle publisher
ros::Publisher turtle_pub;


bool reset;

struct XY{
    float x;
	float y;
};

struct XY pos_err_I;



// declare call back function
void turtle_cb(const turtlesim::Pose::ConstPtr& msg)
{
	turtle = *msg;
}


// rotate the world frame coordinate to body frame
// Using 2D rotation matrix: R(-θ) transforms world → body
//   x_b =  x_w * cos(θ) + y_w * sin(θ)
//   y_b = -x_w * sin(θ) + y_w * cos(θ)
void worldtobody2D(float &x, float &y, float theta)
{
	float x_body =  x * cos(theta) + y * sin(theta);
	float y_body = -x * sin(theta) + y * cos(theta);
	x = x_body;
	y = y_body;
	std::cout << "x rot: " << x << std::endl;
} 

// rotate the world frame coordinate to body frame using Quaternion
// For a 2D planar rotation by θ around z-axis:
//   q  = (cos(θ/2), 0, 0, sin(θ/2))   ← body w.r.t. world
//   q* = (cos(θ/2), 0, 0, -sin(θ/2))  ← inverse / conjugate
//   v_body = q* ⊗ v_world ⊗ q
void worldtobodyQuat(float x, float y, float theta)
{
	// Body-to-world quaternion: rotation by theta around z-axis
	double w_q = cos(theta / 2.0);
	double z_q = sin(theta / 2.0);

	// Construct the quaternion representing turtle orientation
	Eigen::Quaterniond q(w_q, 0.0, 0.0, z_q);
	Eigen::Quaterniond q_normalized(q.w()/q.norm(), q.x()/q.norm(), q.y()/q.norm(), q.z()/q.norm());

	// Represent the 2D error vector as a pure quaternion (w=0)
	Eigen::Quaterniond v(0.0, (double)x, (double)y, 0.0);

	// World → body: v_body = q_inv ⊗ v ⊗ q
	Eigen::Quaterniond v_new = q_normalized.conjugate() * v * q_normalized;

	x = (float)v_new.x();
	y = (float)v_new.y();

	std::cout << "x quat: " << x << std::endl;
}

// P control for goal position in world frame 
void Positioncontrol(geometry_msgs::Point &goal, turtlesim::Pose &turtle_pose, geometry_msgs::Twist &turtle_vel_msg) {

	// P gain tuning parameters
	const float Kp_linear  = 1.5f;  // gain for forward velocity
	const float Kp_angular = 4.0f;  // gain for angular velocity

	// error in inertia frame
	pos_err_I.x = goal.x - turtle_pose.x;
	pos_err_I.y = goal.y - turtle_pose.y;

	// Find the goal_point position in Body(turtlesim) frame
	// Note: worldtobodyQuat takes by value → used for verification/debug only
	worldtobodyQuat(pos_err_I.x, pos_err_I.y, turtle_pose.theta);
	// worldtobody2D takes by reference → actually updates pos_err_I
	worldtobody2D(pos_err_I.x, pos_err_I.y, turtle_pose.theta);

	// Distance to goal in body frame
	float error_norm = sqrt(pow(pos_err_I.x, 2) + pow(pos_err_I.y, 2));

	// Heading error: angle from body x-axis to the goal direction
	float error_theta = atan2(pos_err_I.y, pos_err_I.x);

	// Output boundary: clamp max linear speed to 2
	if (error_norm > 2) error_norm = 2;

	// P controller:
	//   linear.x  → drive forward proportional to distance
	//   angular.z → turn proportional to heading error
	turtle_vel_msg.linear.x  = Kp_linear  * error_norm;
	turtle_vel_msg.angular.z = Kp_angular * error_theta;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_Pcontrol");
  	ros::NodeHandle n;

  	// declare publisher & subscriber

	// turtle subscriber
  	ros::Subscriber turtle_sub = n.subscribe<turtlesim::Pose>("/turtlesim/turtle1/pose", 1, turtle_cb); 

	turtle_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/turtle1/cmd_vel",1);
		
	// define turtle goal point

	ROS_INFO("Please input (x,y). x>0,y>0");
	cout<<"desired_X:";
	cin>>turtle_goal.x;
	cout<<"desired_Y:";
	cin>>turtle_goal.y;	


	// setting frequency as 10 Hz
  	ros::Rate loop_rate(10);
	
  	while (ros::ok()){
		
		ROS_INFO("goal x : %f \t y : %f\n",turtle_goal.x,turtle_goal.y);
    	ROS_INFO("pose x : %f \t y : %f\n",turtle.x,turtle.y);
    	ROS_INFO("pose theta: %f \n",turtle.theta);

		//Input your goal_point to your controller
		Positioncontrol(turtle_goal, turtle, turtle_twist);

		turtle_pub.publish(turtle_twist);

    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
