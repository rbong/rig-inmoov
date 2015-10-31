#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <SerialPort.h>

int main(int argc, char** argv)
{
    SerialPort* serial_port;
    std::string port_name;

    ros::init(argc, argv, "serial_publisher");
    ros::NodeHandle node;
    ros::Publisher serial_publisher =
        node.advertise<std_msgs::UInt8>("inmoov_command", 500);
    ros::Rate loop_rate(500);

    if (ros::param::get ("/serial_publisher/port_name", port_name))
    {
        ROS_INFO ("Port name: %s", port_name.c_str ());
    }
    else
    {
        ROS_ERROR ("Could not retrieve port name");
        return 1;
    }

    ROS_INFO ("Connecting to serial port");
    serial_port = new SerialPort (port_name);

    ROS_INFO ("connecting to serial port");
    serial_port->Open ();
    serial_port->SetBaudRate (SerialPort::BAUD_9600);
    serial_port->SetCharSize (SerialPort::CHAR_SIZE_8);
    serial_port->SetNumOfStopBits (SerialPort::STOP_BITS_1);
    serial_port->SetParity (SerialPort::PARITY_NONE);

    while (serial_port->IsDataAvailable ())
    {
        serial_port->ReadByte ();
    }

    while (ros::ok ())
    {
        std_msgs::UInt8 msg;

        if (! serial_port->IsOpen ())
        {
            ROS_ERROR ("Connection to serial port lost");
            return 1;
        }
        if (serial_port->IsDataAvailable ())
        {
            msg.data = serial_port->ReadByte ();
            serial_publisher.publish (msg);
            ROS_DEBUG ("Recieved byte %u", msg.data);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    serial_port->Close ();

    return 0;
}
