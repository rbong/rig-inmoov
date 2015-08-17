#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <urdf/model.h>

#include <SerialPort.h>
#define URDF_PATH "/home/dork/programs/android/catkin_ws/src/robot_description/urdf/"

/* using namespace LibSerial; */

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
    int c;
    int servo = -1;
    SerialPort serial_port ("/dev/ttyACM1");

    double joint_limit [6] [2];
    boost::shared_ptr<const urdf::Joint> joint;

    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate (500);

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
    serial_port.Open ();
    serial_port.SetBaudRate (SerialPort::BAUD_9600);
    serial_port.SetCharSize (SerialPort::CHAR_SIZE_8);
    serial_port.SetNumOfStopBits (SerialPort::STOP_BITS_1);
    serial_port.SetParity (SerialPort::PARITY_NONE);

    ROS_INFO ("initializing geometry message");
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    /* serial_port.flush (); */

    while (ros::ok())
    {
        joint_state.header.stamp = ros::Time::now();
        odom_trans.header.stamp = ros::Time::now();

        if (serial_port.IsDataAvailable ())
        {
            ROS_INFO ("serial data is available");
            c = serial_port.ReadByte ();
            ROS_INFO ("byte is %d", c);
            if (c == 255 || c == 253)
            {
                c = -1;
            }
            if (servo == -1)
            {
                servo = c;
                ROS_INFO ("setting servo to %d", servo);
            }
            else if (servo > 0 && servo < 6)
            {
                angle [servo] = map
                    (c, 0, 180, joint_limit [servo] [1], joint_limit [servo] [0]);
                ROS_INFO ("setting angle to %g", angle);
                servo = -1;
            }
        }

        joint_state.name.resize(6);
        joint_state.position.resize(6);
        for (int i = 0; i < 6; i++)
        {
            joint_state.name[i] = joint_name [i];
            joint_state.position[i] = angle [i];
        }

        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        loop_rate.sleep();
    }

    serial_port.Close ();

    return 0;
}
