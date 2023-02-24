#include "quadrotor_sim/quadrotor.hpp"

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<quadrotor_sim/thrust_rates.h>
#include<tf/transform_broadcaster.h>

Quadrotor quad;
ros::Publisher odom_pub;
ros::Subscriber thrust_rates_sub;

tf::TransformBroadcaster* tf_br;

void rcv_thrust_rates_cb(const quadrotor_sim::thrust_rates& msg)
{
    double c[4];
    c[0] = msg.thrust;
    c[1] = msg.wx;
    c[2] = msg.wy;
    c[3] = msg.wz;

    quad.send_ctrl(c);
}

void quadrotor_states_cb(double states[13])
{
    // std::cout << states[0] << std::endl;
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";

    odom.pose.pose.position.x = states[0];
    odom.pose.pose.position.y = states[1];
    odom.pose.pose.position.z = states[2];
    odom.twist.twist.linear.x = states[3];
    odom.twist.twist.linear.y = states[4];
    odom.twist.twist.linear.z = states[5];
    odom.pose.pose.orientation.w = states[6];
    odom.pose.pose.orientation.x = states[7];
    odom.pose.pose.orientation.y = states[8];
    odom.pose.pose.orientation.z = states[9];
    odom.twist.twist.angular.x = states[10];
    odom.twist.twist.angular.y = states[11];
    odom.twist.twist.angular.z = states[12];

    odom_pub.publish(odom);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(states[1], states[0], -states[2]) );
    transform.setRotation( tf::Quaternion( states[8], states[7], -states[9], states[6] ));
    tf_br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad_body"));
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "quadrotor_sim");
    ros::NodeHandle nh("~");

    tf_br = new tf::TransformBroadcaster();

    double pos0[3] = {0,0, -2};
    quad.set_position(pos0);

    odom_pub = nh.advertise<nav_msgs::Odometry>("Odometry", 1);

    quad.set_states_cb(quadrotor_states_cb);
    quad.start();
    
    thrust_rates_sub = nh.subscribe("thrust_rates", 1, rcv_thrust_rates_cb);

    ros::spin();
    return 0;
}
