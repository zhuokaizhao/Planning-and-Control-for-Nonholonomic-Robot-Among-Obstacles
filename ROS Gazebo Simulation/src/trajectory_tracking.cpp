#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <robot/DesiredStates.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


typedef message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
typedef message_filters::Subscriber<robot::DesiredStates> desired_state_sub;
typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, robot::DesiredStates> syncPolicy;

const float PI = 3.1415926;

class TrajecTracking{
public:
	TrajecTracking();
private:
	void syncCB(const nav_msgs::Odometry::ConstPtr& odome, const robot::DesiredStates::ConstPtr& desiredSt);
	ros::NodeHandle nh;
	ros::Publisher vel_pub;
	float pre_dx1, pre_dx2;
	float kp, kd;
	float u1;
	float last_time;
	float current_time;
	bool finished;
	odom_sub odom_sub_;
	desired_state_sub desired_state_sub_;
	message_filters::Synchronizer<syncPolicy> sync;
	tf::Transform transform;
};

TrajecTracking::TrajecTracking():
pre_dx1(0),
pre_dx2(0),
kp(1.0),
kd(1.0),
u1(0.01),
finished(false),
last_time(0),
current_time(0),
odom_sub_(nh, "/ground_truth/state", 1),
desired_state_sub_(nh, "/desired_states", 1),
sync(syncPolicy(10),odom_sub_,desired_state_sub_)
{
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	sync.registerCallback(boost::bind(&TrajecTracking::syncCB,this,_1,_2));
	nh.getParam("/KP", kp);
	nh.getParam("/KD", kd);
}

void TrajecTracking::syncCB(const nav_msgs::Odometry::ConstPtr& odome, const robot::DesiredStates::ConstPtr& desiredSt){
	current_time = odome->header.stamp.toSec();
	nh.getParam("/Finished", finished);
	geometry_msgs::Twist cmd;
	if(finished){
		cmd.linear.x = 0;
		cmd.linear.z = 0;
		vel_pub.publish(cmd);
	}
	else{
		if(last_time>0){
			float delta_t = current_time - last_time;
			// get desired states
			float x1_d, x2_d, dx1_d, dx2_d, ddx1_d, ddx2_d;
			float pre_dx1, pre_dx2;
			x1_d = desiredSt->x1;
			x2_d = desiredSt->x2;
			dx1_d = desiredSt->dx1;
			dx2_d = desiredSt->dx2;
			ddx1_d = desiredSt->ddx1;
			ddx2_d = desiredSt->ddx2;
			// get current states
			float x1, x2, dx1, dx2, ddx1, ddx2;
			x1 = odome->pose.pose.position.x;
			x2 = odome->pose.pose.position.y;
			dx1 = odome->twist.twist.linear.x;
			dx2 = odome->twist.twist.linear.y;
			ddx1 = (dx1 - pre_dx1) / delta_t;
			ddx2 = (dx2 - pre_dx2) / delta_t;
			pre_dx1 = dx1;
			pre_dx2 = dx2;
			transform.setOrigin(tf::Vector3(x1, x2, 0.0));
			tf::Quaternion q;
			tf::quaternionMsgToTF(odome->pose.pose.orientation, q);
			transform.setRotation(q);
			static tf::TransformBroadcaster br;
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "car_body"));
			float v1 = ddx1_d - kp*(x1 - x1_d) - kd*(dx1 - dx1_d);
			float v2 = ddx2_d - kp*(x2 - x2_d) - kd*(dx2 - dx2_d);
			float theta = atan2(dx2, dx1);
			if(theta>0)
				theta = theta - floor(theta/PI)*PI;
			else
				theta = theta - ceil(theta/PI)*PI;
			std::cout << "theta " << theta << std::endl;
			float v_input1 = cos(theta) * v1 + sin(theta) * v2;
			float v_input2 = cos(theta) * v2 - sin(theta) * v1;
			float dummy_u1 = u1;
			dummy_u1 = (dummy_u1>0 && dummy_u1<1 ? 1.0 : dummy_u1);
			dummy_u1 = (dummy_u1<0 && dummy_u1>-1 ? -1.0 : dummy_u1);
			float u2 = v_input2 / dummy_u1;
			u1 += v_input1 * delta_t;
			if(u2>0.5)
				u2 = 0.5;
			else if(u2<-0.5)
				u2 = -0.5;
			if(u1>0.8)
				u1 = 0.8;
			else if(u1<0)
				u1 = 0.05;
			cmd.linear.x = u1;
			cmd.angular.z = u2;
			vel_pub.publish(cmd);
			ROS_INFO("x1 %f x2 %f desired x1 %f desired x2 %f", x1, x2, x1_d, x2_d);
			ROS_INFO("linear velocity %f angular velocity %f", u1, u2);
		}
	}
	last_time = current_time;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "trajec_tracking");
	TrajecTracking tt;
	ros::spin();
	return 0;
}
