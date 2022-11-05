


#include "robot_haptic/RobotKinnect.h"

RobotTarget target1;


int main( int argc, char **argv)
{std::cout<<"started";
    ros::init(argc, argv, "action_client");
    ros::NodeHandle node;
    ROS_INFO("started");
    std::cout<<"started";
    RobotKinnect robot(node,100,"/kinect/found_pos",
                                "/iiwa/command/GripperMode",
                                "/iiwa/command/FingerPositionAndSpeed",
                                "/iiwa/command/GriperForce",
                                "/iiwa/robotiq/state/FingerObject",
                                "/iiwa/robotiq/state/GripperStatus",
                                "/iiwa/robotiq/state/MotionStatus");

   // ClientCartesian ac1("/iiwa/action/move_to_cartesian_pose",true);
    ROS_INFO("waiting for server");
    robot.RobotKinnect::ac.waitForServer();
    robot.RobotKinnect::acj.waitForServer();
    ROS_INFO("connected");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    geometry_msgs::Point point ;
    std_msgs::Int8 mode;
    mode.data = 1;
    while(ros::ok()){
        point = robot.cameraToRobot(robot.objectPosition);

        ROS_INFO_STREAM("\n---------------------------------------------\nball "<<point.x<<" "<<point.y<<" "<<point.z<<'\n');
        ROS_INFO_STREAM("order  ==== "<<robot.order);
        
        switch(robot.RobotKinnect::order){
            case 0:
            {   
                bool next = robot.moveGripper(robot.toggleGripper(false),mode);
                robot.RobotKinnect::sendStarting();
                if(next){
                robot.RobotKinnect::order++;
                break;
                }
                else break;
                
            }
            case 1:{
                ROS_INFO("im case 1");
                if(point.x < 0.8 && point.y < 0.8 && point.z < 0.8) {robot.RobotKinnect::order++;
                break;}
                else break;

            }
            case 2:
            {   
                ROS_INFO("im case 2");
                if(point.x < 0.8 && point.y < 0.8 && point.z < 0.8){
                bool next = robot.RobotKinnect::moveRobot(  robot.RobotKinnect::cast2Point (point.x,//x
                                                                                            point.y,//y
                                                                                            0.15),//z
                                                            robot.RobotKinnect::euler2Quaternion (-171,//roll
                                                                                                   1,  //yaw
                                                                                                  -164));//pitch
                if (next){ robot.RobotKinnect::order++;
                break;}
                else break;
                }
            }
            case 3:
            {   
                ROS_INFO("im case 3");
                if(point.x < 0.8 && point.y < 0.8 && point.z < 0.8){
                bool next = robot.RobotKinnect::moveRobot(  robot.RobotKinnect::cast2Point (point.x,
                                                                                            point.y,
                                                                                            point.z),
                                                            robot.RobotKinnect::euler2Quaternion (-171,
                                                                                                   1,
                                                                                                  -164));
                if (next){ robot.RobotKinnect::order++;
                break;}
                else break;
                }
            }
            case 4:
            {
                ROS_INFO("im case 4");
                //robot.sendHome();
                //robot.RobotKinnect::order ++;

                bool next = robot.moveGripper(robot.toggleGripper(true),mode);
                if(next){
                    robot.RobotKinnect::order++;
                    ROS_INFO_STREAM("gipper closeted ");
                    break;
                }
                else break;

                
            }
            case 5:
            {

                robot.sendStarting();
                robot.RobotKinnect::order++;
                break;
            }
            default:
            {
                ros::shutdown();
                break;
            }
        } 

      robot.loopRate.sleep();
    }
    spinner.stop();

    return 0;
}