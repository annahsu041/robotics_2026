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

// World frame → Body frame  (rotate by -θ around z-axis)
// q  = body-to-world = (cos(θ/2), 0, 0, sin(θ/2))
// v_body = q* ⊗ v_world ⊗ q
void worldtobodyQuat(float &x, float &y, float theta)
{
	// quaternion representing turtle orientation (body w.r.t. world)
	double half = theta / 2.0;
	Eigen::Quaterniond q(cos(half), 0.0, 0.0, sin(half));
	q.normalize();

	// wrap the 2D vector as a pure quaternion (w=0)
	Eigen::Quaterniond v(0.0, (double)x, (double)y, 0.0);

	// world → body:  q_conj * v * q
	Eigen::Quaterniond v_body = q.conjugate() * v * q;

	x = (float)v_body.x();
	y = (float)v_body.y();
}

// Body frame → World frame  (rotate by +θ around z-axis)
// q  = body-to-world = (cos(θ/2), 0, 0, sin(θ/2))
// v_world = q ⊗ v_body ⊗ q*
void body2WorldQuat(float &x, float &y, float theta)
{
	// quaternion representing turtle orientation (body w.r.t. world)
	double half = theta / 2.0;
	Eigen::Quaterniond q(cos(half), 0.0, 0.0, sin(half));
	q.normalize();

	// wrap the 2D vector as a pure quaternion (w=0)
	Eigen::Quaterniond v(0.0, (double)x, (double)y, 0.0);

	// body → world:  q * v * q_conj
	Eigen::Quaterniond v_world = q * v * q.conjugate();

	x = (float)v_world.x();
	y = (float)v_world.y();
}

// P control for goal position in world frame 
void Positioncontrol(geometry_msgs::Point &goal, turtlesim::Pose &turtle_pose, geometry_msgs::Twist &turtle_vel_msg) {

	// P gain tuning parameters
	const float Kp_linear  = 1.5f;
	const float Kp_angular = 4.0f;

	// Compute error in world frame
	pos_err_I.x = goal.x - turtle_pose.x;
	pos_err_I.y = goal.y - turtle_pose.y;

	// Transform error from world frame → body frame
	worldtobodyQuat(pos_err_I.x, pos_err_I.y, turtle_pose.theta);

	// Distance to goal (in body frame, magnitude is frame-invariant)
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
	ros::init(argc, argv, "turtle_Pcontrol_B");
  	ros::NodeHandle n;

  	// declare publisher & subscriber

	// turtle subscriber
  	ros::Subscriber turtle_sub = n.subscribe<turtlesim::Pose>("/turtlesim/turtle1/pose", 1, turtle_cb); 

	turtle_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/turtle1/cmd_vel",1);
		
	// define turtle goal point (given in body frame)

	float gx, gy;

	ROS_INFO("Please input (x,y) in BODY frame. x>0,y>0");
	cout<<"desired_X:";
	cin>>gx;
	cout<<"desired_Y:";
	cin>>gy;	

	// spin once to get the latest turtle pose before doing the conversion
	ros::spinOnce();

	/*-----------------------------------------------------------------------
	 * Convert the user-given goal from body frame → world frame,
	 * then offset by the turtle's current world-frame position.
	 *
	 * Steps:
	 *   1. Rotate (gx, gy) from body → world using current turtle theta
	 *   2. Add current turtle position to get absolute world coordinates
	 *
	 * Why:  The user specified a relative offset in body frame (e.g. "go
	 *       1 m ahead of me").  We need the absolute world-frame target so
	 *       that Positioncontrol() can compute world-frame errors correctly.
	 *-----------------------------------------------------------------------*/
	body2WorldQuat(gx, gy, turtle.theta);   // rotate body offset → world direction
	turtle_goal.x = turtle.x + gx;          // absolute world x
	turtle_goal.y = turtle.y + gy;          // absolute world y

	ROS_INFO("Converted goal in world frame → x: %f  y: %f", turtle_goal.x, turtle_goal.y);

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
