#include <ros/ros.h>

#include "ros_keigan/KeiganMotor.h"
#include <geometry_msgs/Twist.h>

class DGControl
{
	public:
		DGControl(ros::NodeHandle _nh) : nh(_nh)
		{
			r_motor = new KeiganMotor("/dev/rightmotor", 115200);
			l_motor = new KeiganMotor("/dev/leftmotor", 115200);

			r_motor->enable();
			l_motor->enable();

			r_motor->speed(10);
			l_motor->speed(10);

			sub_cmd_vel = nh.subscribe("cmd_vel", 10, &DGControl::callback_cmd_vel, this);
		}
		~DGControl()
		{
			r_motor->disable();
			l_motor->disable();
			delete r_motor;
			delete l_motor;
		}
		void spin()
		{
			ros::Rate rate(10);
			while(ros::ok())
			{
				ros::spinOnce();
				rate.sleep();
			}
		}
		void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& vel)
		{
			geometry_msgs::Twist velocity = *vel;
			float forward = velocity.linear.x / 0.05;
			float rotation = velocity.angular.z * 0.1 / (2*0.05);

			r_motor->speed(std::fabs(forward-rotation));
			l_motor->speed(std::fabs(forward+rotation));
			
			if(0 < (forward-rotation)) r_motor->runBackward();
			else if((forward-rotation) < 0) r_motor->runForward();

			if(0 < (forward+rotation)) l_motor->runForward();
			else if((forward+rotation) < 0) l_motor->runBackward();

			if((forward+rotation) == 0 && (forward-rotation) == 0)
			{
				r_motor->stop();
				l_motor->stop();
			}
		}
		ros::Time getTime() const {return ros::Time::now();}
		ros::Duration getPeriod() const {return ros::Duration(0.01);}
	private:
		ros::Subscriber sub_cmd_vel;
		KeiganMotor *r_motor, *l_motor;
		ros::NodeHandle nh;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dg_sbr_driver");
	ros::NodeHandle nh;

	DGControl motor(nh);
	
	motor.spin();	
}
