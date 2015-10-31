#include <string>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <urdf/model.h>

#include <iostream>
#include <fstream>

double angle [6] = { 0, 0, 0, 0, 0, 0, };
double joint_limit [6] [2];

double map (double value, double from_min, double from_max, double to_min, double to_max)
{
    if (value < from_min)
    {
        value = from_min;
    }
    if (value > from_max)
    {
        value = from_max;
    }

    if (from_max - from_min == 0)
    {
        ROS_INFO ("Preventing divide by zero");
        return to_max;
    }

    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

void callback (const std_msgs::UInt8::ConstPtr& msg)
{
    static int c;
    static int servo = -1;

    c = msg->data;
    ROS_DEBUG ("Byte is %d", c);
    if (c == 255 || c == 253)
    {
        c = -1;
    }
    if (servo == -1)
    {
        servo = c;
        ROS_DEBUG ("Setting servo to %d", servo);
    }
    else if (servo > 0 && servo < 6)
    {
        ROS_DEBUG ("Setting angle to %g", angle);
        angle [servo] = map
            (c, 0, 180, joint_limit [servo] [1], joint_limit [servo] [0]);
        servo = -1;
    }
    else
    {
        servo = -1;
    }
}

int main (int argc, char** argv)
{
    boost::shared_ptr<const urdf::Joint> joint;

    ros::init (argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub =
        n.advertise<sensor_msgs::JointState> ("joint_states", 1);
    ros::Subscriber inmoov_sub =
        n.subscribe ("inmoov_command", 500, callback);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate (500);

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;

    const char* joint_name [6] =
    {
        "right_arm_to_wrist", "right_palm_to_thumb", "right_palm_to_index",
        "right_palm_to_medius", "right_palm_to_ring", "right_palm_to_pinky",
    };

    std::string urdf_file;
    urdf::Model model;

    if (ros::param::get ("/state_publisher/urdf_file", urdf_file))
    {
        ROS_INFO ("URDF file location: %s", urdf_file.c_str ());
    }
    else
    {
        ROS_ERROR ("Could not retrieve model file");
        return 1;
    }

    if (!model.initFile (urdf_file))
    {
        ROS_ERROR ("Failed to parse urdf file");
        return 1;
    }
    for (int i = 0; i < 6; i++)
    {
        joint = model.getJoint (joint_name [i]);
        joint_limit [i] [0] = joint->limits->lower;
        joint_limit [i] [1] = joint->limits->upper;
    }

    ROS_INFO ("Initializing geometry message");
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw (0);

    ROS_INFO ("Listening");
    while (ros::ok ())
    {
        joint_state.header.stamp = ros::Time::now ();
        odom_trans.header.stamp = ros::Time::now ();

        joint_state.name.resize (6);
        joint_state.position.resize (6);
        for (int i = 0; i < 6; i++)
        {
            joint_state.name [i] = joint_name [i];
            joint_state.position [i] = angle [i];
        }

        joint_pub.publish (joint_state);
        broadcaster.sendTransform (odom_trans);

        ros::spinOnce ();
        loop_rate.sleep ();
    }

    return 0;
}
