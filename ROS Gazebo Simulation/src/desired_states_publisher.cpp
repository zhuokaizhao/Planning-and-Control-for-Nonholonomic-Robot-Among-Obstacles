#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <robot/DesiredStates.h>

typedef std::vector<float> one_state;

class DState{
public:
	DState();
	~DState();
	void DSPublish();
private:
	float t, x1, x2, dx1, dx2, ddx1, ddx2, theta;
	float t1, t2;
	std::vector<one_state> states;
	ros::NodeHandle nh;
	ros::Publisher state_pub;
	std::vector<one_state>::size_type i;
	robot::DesiredStates xd;
	void readfile();
};

DState::DState()
{
	state_pub = nh.advertise<robot::DesiredStates>("/desired_states", 100);
}

DState::~DState(){
	states.clear();
}

void DState::DSPublish(){
	DState::readfile();
	i = 1;
	t1 = states[i-1][0];
	t2 = states[i][0];
	ros::Rate r(100);
	ROS_INFO("start publishing desired states");
	while(i<states.size()-1){
		one_state v;
		v = states[i-1];
		xd.t = v[0];
		xd.x1 = v[1] * 0.01;
		xd.x2 = v[2] * 0.01;
		xd.dx1 = v[3] * 0.01;
		xd.dx2 = v[4] * 0.01;
		xd.ddx1 = v[5] * 0.01;
		xd.ddx2 = v[6] * 0.01;
		xd.theta = v[7];
		ros::Time begin = ros::Time::now();
		ros::Time end = ros::Time::now();
		ros::Duration diff = end - begin;
		while(ros::ok() && diff.toSec()<=t2-t1){
			end = ros::Time::now();
			xd.header.stamp = end;
			state_pub.publish(xd);
			diff = end - begin;
			r.sleep();
		}
		++i;
		t1 = states[i-1][0];
		t2 = states[i][0];
	}
	nh.setParam("/Finished", true);
	ROS_INFO("Finishing publishing all desired states");
	ros::Rate r2(100);
	ros::Time end;
	while(ros::ok()){
		end = ros::Time::now();
		xd.header.stamp = end;
		state_pub.publish(xd);
		r2.sleep();
	}
	return;
}

void DState::readfile(){
	one_state v;
	std::ifstream infile("/home/changxin/nonlinear_ws/src/robot/src/final_trajectory.txt");
	if(!infile){
		ROS_INFO("data not read");
	}
	while(infile >> t >> x1 >> x2 >> dx1 >> dx2 >> ddx1 >> ddx2 >> theta){
		v.push_back(t);
		v.push_back(x1);
		v.push_back(x2);
		v.push_back(dx1);
		v.push_back(dx2);
		v.push_back(ddx1);
		v.push_back(ddx2);
		v.push_back(theta);
		states.push_back(v);
		v.clear();
	}
	infile.close();
	ROS_INFO("finished reading the trajectory");
	return;
}

int main(int argc, char**argv){
	// vector<vector<float>> a;
	ros::init(argc, argv, "xd_publisher");
	DState ddstate;
	ddstate.DSPublish();
	ros::spin();
	return 0;
}	