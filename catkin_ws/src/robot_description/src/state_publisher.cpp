#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <SerialStream.h>

using namespace LibSerial;

int main(int argc, char** argv) {
    SerialStream serial_stream;

    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(50);

    const double degree = M_PI/180;
    double angle [6] = { 0, 0, 0, 0, 0, 0, };
    const char* joint_name [6] =
    {
        "right_arm_to_wrist", "right_palm_to_thumb", "right_palm_to_index",
        "right_palm_to_medius", "right_palm_to_ring", "right_palm_to_pinky",
    };

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

    while (ros::ok()) {
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
            angle [servo] = -c * degree/2;
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

        /* loop_rate.sleep(); */
    }


    return 0;
}
