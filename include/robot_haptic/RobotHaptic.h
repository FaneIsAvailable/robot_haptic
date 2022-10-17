#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/CartesianPose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>


class RobotHaptic
{
    public:

        RobotHaptic(ros::NodeHandle node, float loopRate, 
                    std::string robotPositionTopic, std::string hapticPositionTopic,
                    std::string switchPositionTopic, std::string robotStateTopic,
                    std::string interfStartTopic, std::string interfScaleTopic,
                    std::string interfAxisLockTopic);
        ~RobotHaptic();

        void publishRobotData();       

        void processPos();

        void toEuler(double w, double x, double y, double z);

        void toQuat(double x, double y, double z);


    private:

        ros::NodeHandle node;
        ros::Rate loopRate;

        ros::Publisher rob_pos_pub;
        ros::Subscriber rob_pos_sub;
        ros::Subscriber haptic_pos_sub;
        ros::Subscriber haptic_switch_sub;

        ros::Subscriber interf_start_sub;
        ros::Subscriber interf_scale_sub;
        ros::Subscriber interf_axis_sub;

        std::string robotPositionTopic;
        std::string hapticPositionTopic;
        std::string switchPositionTopic;
        std::string robotStateTopic;

        std::string interfStartTopic;
        std::string interfScaleTopic;
        std::string interfAxisLockTopic;

        //iiwa_msgs::CartesianPose robotPosition;
        //iiwa_msgs::CartesianPose oldRobotPosition;
        geometry_msgs::PoseStamped robotPosition;
        iiwa_msgs::CartesianPose oldRobotPosition;
        geometry_msgs::Twist hapticPosition;
        geometry_msgs::Twist hapticOldPosition;
        geometry_msgs::Twist hapticSwitch;

        double robotEuler[3];
        double robotQuat[4];

        bool teleopStarted;

        int axis_lock[3];

        double motionScale;

        void RobotPositionCallBack(const iiwa_msgs::CartesianPose::ConstPtr &data);

        void HapticPositionCallBack(const geometry_msgs::Twist::ConstPtr &data); 

        void HapticSwitchCallBack(const geometry_msgs::Twist::ConstPtr &data);

        void InterfStartCallBack(const std_msgs::Float64::ConstPtr &data);

        void InterfScaleCallBack(const std_msgs::Float64::ConstPtr &data);

        void InterfAxisLockCallBack(const geometry_msgs::Vector3::ConstPtr &data);
};



