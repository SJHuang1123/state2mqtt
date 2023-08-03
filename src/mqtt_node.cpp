#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

class IMU2mqtt{
private:
    ros::NodeHandle n;
    ros::Subscriber IMU_sub;
    ros::Publisher mqtt_pub;
public:
    IMU2mqtt(){};
    IMU2mqtt(const std::string& input, const std::string& output){
        mqtt_pub = n.advertise<std_msgs::String>(output, 10);
        IMU_sub = n.subscribe<sensor_msgs::Imu>(input, 10, &IMU2mqtt::IMU_cb, this);   
    };

    void IMU_cb(const sensor_msgs::Imu::ConstPtr& msg){
        std_msgs::String output_string;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        float x, y, z, w;
        x = msg->orientation.x;
        y = msg->orientation.y;
        z = msg->orientation.z;
        w = msg->orientation.w;        
        ss << "x: " << (x >= 0 ? "+" : "") << round(100*x)/100 << ", ";
        ss << "y: " << (y >= 0 ? "+" : "") << round(100*y)/100 << ", ";
        ss << "z: " << (z >= 0 ? "+" : "") << round(100*z)/100 << ", ";
        ss << "w: " << (w >= 0 ? "+" : "") << round(100*w)/100 ;
        output_string.data = ss.str();
        mqtt_pub.publish(output_string);
        ros::Rate rate(1.0);
        rate.sleep();
    }
};

class GPS2mqtt{
private:
    ros::NodeHandle n;
    ros::Subscriber GPS_sub;
    ros::Publisher mqtt_pub;
public:
    GPS2mqtt(){};
    GPS2mqtt(const std::string& input, const std::string& output){
        mqtt_pub = n.advertise<std_msgs::String>(output, 10);
        GPS_sub = n.subscribe<sensor_msgs::NavSatFix>(input, 10, &GPS2mqtt::GPS_cb, this);   
    };
    
    void GPS_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
        std_msgs::String output_string;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        float altitude, longitude, latitude;
        altitude = msg->altitude;
        longitude = msg->longitude;
        latitude = msg->latitude;
        ss << "altitude: " << (altitude >= 0 ? "+" : "") << round(10000*altitude)/10000 << ", ";
        ss << "longitude: " << (longitude >= 0 ? "+" : "") << round(10000*longitude)/10000 << ", ";
        ss << "latitude: " << (latitude >= 0 ? "+" : "") << round(10000*latitude)/10000;
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
    IMU2mqtt imu("/mavros/imu/data", "/IMU_o/primitive");
    vel2mqtt vel("/mavros/local_position/velocity_body", "/vel_o/primitive");
    battery2mqtt battery("/mavros/battery", "/battery_o/primitive");
    GPS2mqtt position("/mavros/global_position/global", "/GPS_o/primitive");    
    ros::spin();
}
