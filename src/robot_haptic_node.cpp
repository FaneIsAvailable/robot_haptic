#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <memory>
#include <robot_haptic/RobotHaptic.h>

int main( int argc, char** argv){

    ros::init(argc,argv,"robot_haptic_node");

    ros::NodeHandle node;

    RobotHaptic robot(node, 1000, "/iiwa/command/CartesianPose", "/chai3d/position", "/gripper_topic","/iiwa/state/CartesianPose",
    "/interface/start", "/interface/scale", "/interface/lock_axis");

    robot.publishRobotData();

    ros::shutdown();

}