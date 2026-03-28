#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>

class AckermannToTwistConverter
{
public:
    AckermannToTwistConverter()
    {
        ros::NodeHandle nh;
        twist_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ackermann_sub_ = nh.subscribe("/drive", 10,
            &AckermannToTwistConverter::ackermannCallback, this);
    }

private:
    static constexpr float wheelbase = 0.4;

    void ackermannCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        geometry_msgs::Twist twist_msg;
        twist_msg.angular.z = (msg->drive.speed * std::tan(msg->drive.steering_angle)) / wheelbase;
        twist_msg.linear.x = msg->drive.speed;
        twist_pub_.publish(twist_msg);
    }

    ros::Publisher twist_pub_;
    ros::Subscriber ackermann_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ackermann_to_twist_converter_node");
    AckermannToTwistConverter converter;
    ros::spin();
    return 0;
}
