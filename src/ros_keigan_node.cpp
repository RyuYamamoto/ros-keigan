#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ros_keigan/KeiganMotor.h"

class DGControl
{
	public:
		DGControl(ros::NodeHandle _nh) : nh(_nh), cmd_flag(false)
		{
			r_motor = new KeiganMotor("/dev/rightmotor", 115200);
			l_motor = new KeiganMotor("/dev/leftmotor", 115200);

			r_motor->enable();
			l_motor->enable();

			r_motor->speed(10);
			l_motor->speed(10);

			odom_x = odom_y = odom_th = 0.f;

			send_time = ros::Time::now();

			pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
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
				if(cmd_flag && getTime().toSec() - last_cmdvel.toSec() >= 1.0)
				{
					//r_motor->stop();
					//l_motor->stop();
				}
				pub_odom.publish(publish_odometry());
				ros::spinOnce();
				rate.sleep();
			}
		}
		void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& vel)
		{
			velocity = *vel;
			float forward = velocity.linear.x / 0.05;
			float rotation = velocity.angular.z * 0.1 / (2*0.05);

			r_motor->speed(std::fabs(forward-rotation));
			l_motor->speed(std::fabs(forward+rotation));
			
			if(0 < (forward-rotation)) r_motor->runBackward();
			else if((forward-rotation) < 0) r_motor->runForward();

			if(0 < (forward+rotation)) l_motor->runForward();
			else if((forward+rotation) < 0) l_motor->runBackward();

#if 1
			if((forward+rotation) == 0 && (forward-rotation) == 0)
			{
				r_motor->stop();
				l_motor->stop();
			}
#endif
			cmd_flag = true;
		//	last_cmdvel = ros::Time::now();
		}
		nav_msgs::Odometry publish_odometry()
		{
			cur_time = getTime();

			double dt = cur_time.toSec() - last_cmdvel.toSec();
			odom_x += velocity.linear.x * cos(odom_th) * dt;
			odom_y += velocity.linear.x * sin(odom_th) * dt;
			odom_th += velocity.angular.z * dt;

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th);

			static tf2_ros::TransformBroadcaster br;

			geometry_msgs::TransformStamped odom_tf;
			odom_tf.header.stamp = cur_time;
			odom_tf.header.frame_id = "odom";
			odom_tf.child_frame_id = "base_link";

			odom_tf.transform.translation.x = odom_x;
			odom_tf.transform.translation.y = odom_y;
			odom_tf.transform.translation.z = 0.0;

			tf2::Quaternion q;
			q.setRPY(0,0,odom_th);
			odom_tf.transform.rotation.x = q.x();
			odom_tf.transform.rotation.y = q.y();
			odom_tf.transform.rotation.z = q.z();
			odom_tf.transform.rotation.w = q.w();

			br.sendTransform(odom_tf);

			nav_msgs::Odometry odom;
			odom.header.stamp = cur_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = odom_x;
			odom.pose.pose.position.y = odom_y;
			odom.pose.pose.position.z = 0.0;
			odom_tf.transform.rotation = odom_quat;
			odom.pose.pose.orientation = odom_quat;

			odom.twist.twist.linear.x = velocity.linear.x;
			odom.twist.twist.linear.y = 0.0;
			odom.twist.twist.angular.z = velocity.angular.z;

			send_time = cur_time;

			last_cmdvel = cur_time;

			return odom;
		}
		ros::Time getTime() const {return ros::Time::now();}
		ros::Duration getPeriod() const {return ros::Duration(0.01);}
	private:
		ros::Subscriber sub_cmd_vel;
		ros::Publisher pub_odom;
		ros::NodeHandle nh;
		geometry_msgs::Twist velocity;
		ros::Time last_cmdvel, cur_time, send_time;
		
		KeiganMotor *r_motor, *l_motor;
		double odom_x, odom_y, odom_th;
		bool cmd_flag;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "dg_sbr_driver");
	ros::NodeHandle nh;

	DGControl motor(nh);
	
	motor.spin();	
}
