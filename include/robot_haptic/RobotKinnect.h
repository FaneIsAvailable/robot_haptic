
//#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <iiwa_msgs/CartesianPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>
#include <iiwa_msgs/JointPosition.h>
#include <math.h>

typedef actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> ClientJoint;
typedef actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> ClientCartesian;

struct RobotTarget{
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quat;
    iiwa_msgs::RedundancyInformation redundancy;
};

class RobotKinnect
{
    public:

        RobotKinnect(ros::NodeHandle node, float loopRate, std::string ObjectPositionTopic,
                                                           std::string gripperModeTopic, 
                                                           std::string gripperSpeedPositionTopic,
                                                           std::string gripperForceTopic,
                                                           std::string gripperObjectTopic, 
                                                           std::string gripperStatusTopic,
                                                           std::string gripperMotionTopic,
                                                           std::string handPositionTopic);
        ~RobotKinnect();

        void run();

        geometry_msgs::Point cameraToRobot(const geometry_msgs::Vector3 &message);

        bool moveRobot(geometry_msgs::Point position, geometry_msgs::Quaternion quaternion);

        bool moveGripper(geometry_msgs::TwistStamped gripperPoseAndSpeed,std_msgs::Int8 gripperMode);

        geometry_msgs::Quaternion euler2Quaternion(double x, double y, double z);

        geometry_msgs::TwistStamped toggleGripper(bool gripper);

        geometry_msgs::Point cast2Point(double x, double y, double z);

        bool moveRobot(RobotTarget target);

        void sendHome();
        void sendStarting();
        bool checkHandPosition();

        ClientCartesian ac;
        ClientJoint acj;

        int order;
        ros::Rate loopRate;

        geometry_msgs::Vector3 objectPosition;
        unsigned int gripperMoved;

    private:

       

        ros::NodeHandle node;
        

        ros::Subscriber object_position_sub;
        ros::Publisher gripper_mode_pub;
        ros::Publisher gripper_speed_position_pub;
        ros::Publisher gripper_force_pub;
        ros::Publisher robot_home_pub;

        ros::Subscriber gripper_object_sub;
        ros::Subscriber gripper_status_sub;
        ros::Subscriber gripper_motion_sub;
        ros::Subscriber robot_position_sub;
        ros::Subscriber hand_position_sub;
        
        std::string objectPositionTopic;
        std::string gripperModeTopic;
        std::string gripperSpeedPositionTopic;
        std::string gripperForceTopic;
        std::string robotPositionTopic;

        std::string gripperObjectTopic;
        std::string gripperStatusTopic;
        std::string gripperMotionTopic;
        std::string handPositionTopic;

        
        std_msgs::Int8 gripperMode;
        geometry_msgs::TwistStamped gripperSpeedPosition;
        geometry_msgs::Vector3Stamped gripperForce;
        iiwa_msgs::CartesianPose robotPosition;

        geometry_msgs::Vector3 gripperObject;
        std_msgs::Int8 gripperStatus;
        std_msgs::Int8 motionStatus;
        tf::Point error;
        geometry_msgs::Point handPosition;

        

        void GripperObjectCallback(const geometry_msgs::Vector3::ConstPtr &data);
        void GripperStatusCallback(const std_msgs::Int8::ConstPtr &data);
        void MotionStatusCallback(const std_msgs::Int8::ConstPtr &data);
        void ObjectPositionCallback(const geometry_msgs::Vector3::ConstPtr &data);
        void robotPositionCallback(const iiwa_msgs::CartesianPose &data);
        void handPositionCallback(const geometry_msgs::Point::ConstPtr &data);
       bool gripperInMotion;

        
};

