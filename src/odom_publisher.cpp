#include <ros/ros.h>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

double vx = 0.0;
double vy = 0.0;//Always 0 robot assumed to be non holonomic
double vth = 0.0;

double pose_x = 0.0, pose_y = 0.0, pose_theta = 0.0;

ros::Time pose_current_time, pose_last_time;
ros::Publisher pose_from_odom_pub;
uint64_t seq_id = 0;


void calc_pose_from_odom(const ros::TimerEvent& event)
{
     seq_id++;
     pose_current_time = ros::Time::now();
     double dt = (pose_current_time - pose_last_time).toSec();
     double delta_x = (vx * cos(pose_theta) - vy * sin(pose_theta)) * dt;
     double delta_y = (vx * sin(pose_theta) + vy * cos(pose_theta)) * dt;
     double delta_th = vth * dt;

     pose_x += delta_x;
     pose_y += delta_y;
     pose_theta += delta_th;
     geometry_msgs::PoseStamped pose_stamped;
     tf2::Quaternion q;
     q.setRPY(0, 0, pose_theta);
     geometry_msgs::Quaternion pose_quat = tf2::toMsg(q);
     pose_stamped.header.seq = seq_id;
     pose_stamped.header.stamp = ros::Time::now();
     pose_stamped.header.frame_id = "map";
     pose_stamped.pose.position.x = pose_x;
     pose_stamped.pose.position.y = pose_y;
     pose_stamped.pose.position.z = 0.0;
     pose_stamped.pose.orientation = pose_quat;
     pose_from_odom_pub.publish(pose_stamped);
     pose_last_time = pose_current_time;



}
void pose_update_callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
     pose_x = msg.pose.pose.position.x;
     pose_y = msg.pose.pose.position.y;
     pose_theta = tf2::getYaw(msg.pose.pose.orientation);

}

void cmd_vel_callback(const geometry_msgs::Twist &msg )
{
     vx = msg.linear.x;
     vth = msg.angular.z;
}

int main(int argc, char** argv)
{
     ros::init(argc, argv, "odometry_publisher");

     ros::NodeHandle n;
     ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",5); 
     pose_from_odom_pub = n.advertise<geometry_msgs::PoseStamped>("/pose_from_odom",5);
     ros::Subscriber pose_update_sub = n.subscribe("/pose_update", 2, &pose_update_callback);
     tf2_ros::TransformBroadcaster  odom_broadcaster;
     ros::Subscriber cmd_vel_sub = n.subscribe("/key_vel",10, cmd_vel_callback);
     double x = 0.0;
     double y = 0.0;
     double th = 0.0;


     ros::Time current_time, last_time;
     current_time = ros::Time::now();
     last_time = ros::Time::now();
     pose_current_time = current_time;
     pose_last_time = pose_current_time;
     ros::Timer timer = n.createTimer(ros::Duration(0.1), calc_pose_from_odom);


     ros::Rate r(30.0);
     while(n.ok())
     {

          ros::spinOnce();               // check for incoming messages
          current_time = ros::Time::now();

          //compute odometry in a typical way given the velocities of the robot
          double dt = (current_time - last_time).toSec();
          double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
          double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
          double delta_th = vth * dt;

          x += delta_x;
          y += delta_y;
          th += delta_th;

          //since all odometry is 6DOF we'll need a quaternion created from yaw
          tf2::Quaternion q;
          q.setRPY(0, 0, th);
          geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

          //first, we'll publish the transform over tf
          geometry_msgs::TransformStamped odom_trans;
          odom_trans.header.stamp = current_time;
          odom_trans.header.frame_id = "odom";
          odom_trans.child_frame_id = "base_link";


          odom_trans.transform.translation.x = x;
          odom_trans.transform.translation.y = y;
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;

          //send the transform
          odom_broadcaster.sendTransform(odom_trans);

          //next, we'll publish the odometry message over ROS
          nav_msgs::Odometry odom;
          odom.header.stamp = current_time;
          odom.header.frame_id = "odom";

          //set the position
          odom.pose.pose.position.x = x;
          odom.pose.pose.position.y = y;
          odom.pose.pose.position.z = 0.0;
          odom.pose.pose.orientation = odom_quat;

          //set the velocity
          odom.child_frame_id = "base_link";
          odom.twist.twist.linear.x = vx;
          odom.twist.twist.linear.y = vy;
          odom.twist.twist.angular.z = vth;

          //publish the message
          odom_pub.publish(odom);

          last_time = current_time;
          r.sleep();
     }
}
