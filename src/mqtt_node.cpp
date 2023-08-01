#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <cmath>

class Pose2mqtt{
private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub;
    ros::Publisher mqtt_pub;
public:
    Pose2mqtt(){};
    Pose2mqtt(const std::string& input, const std::string& output){
        mqtt_pub = n.advertise<std_msgs::String>(output, 10);
        pose_sub = n.subscribe<geometry_msgs::PoseStamped>(input, 10, &Pose2mqtt::pose_cb, this);    
    };
    
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        std_msgs::String output_string;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        float px, py, pz, ox, oy, oz, ow;
        px = msg->pose.position.x;
        py = msg->pose.position.y;
        pz = msg->pose.position.z;
        ox = msg->pose.orientation.x;
        oy = msg->pose.orientation.y;
        oz = msg->pose.orientation.z;
        ow = msg->pose.orientation.w;        
        ss << "x: " << (px >= 0 ? "+" : "") << round(100*px)/100 << ", ";
        ss << "y: " << (py >= 0 ? "+" : "") << round(100*py)/100 << ", ";
        ss << "z: " << (pz >= 0 ? "+" : "") << round(100*pz)/100 ;
        output_string.data = ss.str();
        mqtt_pub.publish(output_string);
        ros::Rate rate(1.0);
        rate.sleep();
    }
    
};

class vel2mqtt{
private:
    ros::NodeHandle n;
    ros::Subscriber vel_sub;
    ros::Publisher mqtt_pub;
public:
    vel2mqtt(){};
    vel2mqtt(const std::string& input, const std::string& output){
        mqtt_pub = n.advertise<std_msgs::String>(output, 10);
        vel_sub = n.subscribe<geometry_msgs::TwistStamped>(input, 10, &vel2mqtt::vel_cb, this);   
    };
    
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
        std_msgs::String output_string;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        float x, y, z;
        x = msg->twist.linear.x;
        y = msg->twist.linear.y;
        z = msg->twist.linear.z;
        ss << "x: " << (x >= 0 ? "+" : "") << round(100*x)/100 << ", ";
        ss << "y: " << (y >= 0 ? "+" : "") << round(100*y)/100 << ", ";
        ss << "z: " << (z >= 0 ? "+" : "") << round(100*z)/100;
        output_string.data = ss.str();
        mqtt_pub.publish(output_string);
        ros::Rate rate(1.0);
        rate.sleep();
    }
};

class battery2mqtt{
private:
    ros::NodeHandle n;
    ros::Subscriber battery_sub;
    ros::Publisher mqtt_pub;
public:
    battery2mqtt(){};
    battery2mqtt(const std::string& input, const std::string& output){
        mqtt_pub = n.advertise<std_msgs::String>(output, 10);
        battery_sub = n.subscribe<sensor_msgs::BatteryState>(input, 10, &battery2mqtt::battery_cb, this);    
    };
    
    void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
        std_msgs::String string;
        string.data = std::to_string(msg->percentage);
        mqtt_pub.publish(string);
	ros::Rate rate(1.0);
        rate.sleep();
    }
};


int main (int argc, char** argv){
    ros::init(argc, argv, "mqtt_node");
    Pose2mqtt pose("/mavros/local_position/pose", "/pose_o/primitive");
    vel2mqtt vel("/mavros/local_position/velocity_body", "/vel_o/primitive");
    battery2mqtt battery("/mavros/battery", "/battery_o/primitive");
    ros::spin();
}
