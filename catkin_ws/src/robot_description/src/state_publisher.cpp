#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <urdf/model.h>

#include <SerialStream.h>

using namespace LibSerial;

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

    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

int main(int argc, char** argv)
{
    SerialStream serial_stream;

    double joint_limit [6] [2];
    boost::shared_ptr<const urdf::Joint> joint;

    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(50);

    double angle [6] = { 0, 0, 0, 0, 0, 0, };
    const char* joint_name [6] =
    {
        "right_arm_to_wrist", "right_palm_to_thumb", "right_palm_to_index",
        "right_palm_to_medius", "right_palm_to_ring", "right_palm_to_pinky",
    };

    std::string urdf_file = "/home/dork/programs/android/catkin_ws/src/robot_description/model.xml";
    urdf::Model model;

    if (!model.initFile(urdf_file))
    {
        ROS_ERROR("Failed to parse urdf file");
        return -1;
    }
    for (int i = 0; i < 6; i++)
    {
        joint = model.getJoint (joint_name [i]);
        joint_limit [i] [0] = joint->limits->lower;
        joint_limit [i] [1] = joint->limits->upper;
    }

    ROS_INFO ("connecting to serial port");
    serial_stream.Open ("/dev/ttyACM0");
    serial_stream.SetBaudRate (SerialStreamBuf::BAUD_9600);
    serial_stream.SetCharSize (SerialStreamBuf::CHAR_SIZE_8);
    serial_stream.SetNumOfStopBits (1);
    serial_stream.SetParity (SerialStreamBuf::PARITY_NONE);

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    while (ros::ok())
    {
        uint8_t c;
        int servo;

        serial_stream >> c;
        if (c == 255 || c == 253)
        {
            c = -1;
        }
        servo = c;
        serial_stream >> c;
        if (c == 255 || c == 253)
        {
            servo = -1;
        }
        if (servo > 0 && servo < 6)
        {
            angle [servo] = map
                (c, 0, 180, joint_limit [servo] [1], joint_limit [servo] [0]);
        }

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);

        for (int i = 0; i < 6; i++)
        {
            joint_state.name[i] = joint_name [i];
            joint_state.position[i] = angle [i];
        }

        odom_trans.header.stamp = ros::Time::now();

        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);
    }

    return 0;
}
