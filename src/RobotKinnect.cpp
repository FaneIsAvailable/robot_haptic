#include <robot_haptic/RobotKinnect.h>

RobotKinnect::RobotKinnect( ros::NodeHandle node, float loopRate, 
                            std::string objectPositionTopic, 
                            std::string gripperModeTopic, 
                            std::string gripperSpeedPositionTopic,
                            std::string gripperForceTopic,
                            std::string gripperObjectTopic, 
                            std::string gripperStatusTopic,
                            std::string gripperMotionTopic):
                            node(node), 
                            loopRate(loopRate), 
                            objectPositionTopic(objectPositionTopic), 
                            gripperModeTopic(gripperModeTopic),
                            gripperSpeedPositionTopic(gripperSpeedPositionTopic), 
                            gripperForceTopic(gripperForceTopic), 
                            gripperObjectTopic(gripperObjectTopic),
                            gripperStatusTopic(gripperStatusTopic), 
                            gripperMotionTopic(gripperMotionTopic),
                            order(0),
                            ac("/iiwa/action/move_to_cartesian_pose",true),
                            acj("/iiwa/action/move_to_joint_position",true)
                            {
                                
                                this->object_position_sub= this->node.subscribe<geometry_msgs::Vector3>(this->objectPositionTopic.c_str(),1,&RobotKinnect::ObjectPositionCallback, this);
                                this->gripper_object_sub = this->node.subscribe<geometry_msgs::Vector3>(this->gripperObjectTopic.c_str(),1, &RobotKinnect::GripperObjectCallback, this);
                                this->gripper_status_sub = this->node.subscribe<std_msgs::Int8>(this->gripperStatusTopic.c_str(), 1, &RobotKinnect::GripperStatusCallback,this);
                                this->gripper_motion_sub = this->node.subscribe<std_msgs::Int8>(this->gripperMotionTopic.c_str(), 1, &RobotKinnect::MotionStatusCallback, this);

                                this->gripper_mode_pub = this->node.advertise<std_msgs::Int8>(this->gripperModeTopic.c_str(),1);
                                this->gripper_speed_position_pub = this->node.advertise<geometry_msgs::TwistStamped>(this->gripperSpeedPositionTopic.c_str(),1);
                                this->gripper_force_pub = this->node.advertise<geometry_msgs::Vector3Stamped>(this->gripperForceTopic.c_str(),1);
                                this->robot_home_pub = this->node.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition",1);
                                
                                error.setX(0.0);
                                error.setY(0.085);
                                error.setZ(0.0);
                            }
RobotKinnect::~RobotKinnect(){}

void RobotKinnect::sendHome(){
    iiwa_msgs::JointPosition pos;
    pos.header.stamp = ros::Time::now();
    pos.position.a1 = 0;
    pos.position.a2 = 0;
    pos.position.a3 = 0;
    pos.position.a4 = 0;
    pos.position.a5 = 0;
    pos.position.a6 = 0;
    pos.position.a7 = 0;
    robot_home_pub.publish(pos);
}
void RobotKinnect::sendStarting(){
    iiwa_msgs::MoveToJointPositionGoal pos;
    pos.joint_position.header.stamp = ros::Time::now();
    pos.joint_position.position.a1 = 0;
    pos.joint_position.position.a2 = 0;
    pos.joint_position.position.a3 = 0;
    pos.joint_position.position.a4 = -1.57;
    pos.joint_position.position.a5 = 0;
    pos.joint_position.position.a6 = 1.57;
    pos.joint_position.position.a7 = 0;
    pos.joint_position.header.frame_id="iiwa_link_0";
    //robot_home_pub.publish(pos);
    acj.sendGoal(pos);
    ROS_INFO("sending to start pos");
    acj.waitForResult();
    ROS_INFO("i can start now");

}

geometry_msgs::Point RobotKinnect::cameraToRobot(const geometry_msgs::Vector3 &message){
    geometry_msgs::Point output;
    tf::Point point;
    double alpha = 17.5;
    alpha = (alpha* 355/113)/180;
  /**  output.x =   sin(0.31)*message.x - 6.1232e-17*message.y - cos(0.31)*message.z + 1.2500;
    output.y =   1.8840e-17*message.x + message.y + 8.4521e-17*message.z ;
    output.z =   cos(0.31)*message.x + 6.1232e-17*message.y - sin(0.31)*message.z - 0.3200;*/
   output.y = message.y + 0.085;
   output.x = 1.27 - double(cos(alpha)*message.z) + double(sin(alpha)*message.x);
   output.z = 0.34 - double(sin(alpha)*message.z + cos(alpha)*message.x); 
   ROS_INFO_STREAM("OUTPUT VALUS: sin alpha z = "<<sin(alpha)*message.z <<
                   "cos alpha x =  "<< cos(alpha)*message.x<<
                   "sin alpha z =  "<<sin(alpha)*message.z<<
                   "cos alpha x "<<   cos(alpha)*message.x);
   /* tf::pointMsgToTF(output,point);
    point = point + this->error;
    tf::pointTFToMsg(point,output);*/
    return output;
}
bool RobotKinnect::moveRobot(geometry_msgs::Point position, geometry_msgs::Quaternion quaternion)
{
    iiwa_msgs::MoveToCartesianPoseGoal goal;
    goal.cartesian_pose.poseStamped.header.frame_id="iiwa_link_0";
    goal.cartesian_pose.poseStamped.header.stamp = ros::Time::now();
    goal.cartesian_pose.poseStamped.pose.position = position;
    goal.cartesian_pose.poseStamped.pose.orientation = quaternion;
    goal.cartesian_pose.redundancy.status = 2;
    goal.cartesian_pose.redundancy.turn = 92;

    ac.sendGoal(goal);
    ROS_INFO("goal sent");
    ac.waitForResult();

    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)return true;
    else return false;
    
}

 bool RobotKinnect::moveRobot(RobotTarget target)
 {
    iiwa_msgs::MoveToCartesianPoseGoal goal;
    goal.cartesian_pose.poseStamped.header.frame_id="iiwa_link_0";
    goal.cartesian_pose.poseStamped.header.stamp = ros::Time::now();
    goal.cartesian_pose.poseStamped.pose.position = target.point;
    goal.cartesian_pose.poseStamped.pose.orientation = target.quat;
    goal.cartesian_pose.redundancy = target.redundancy;

    ac.sendGoal(goal);
    ROS_INFO("goal sent");
    ac.waitForResult();
 
    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED) return true;
    else return false;

 }

bool RobotKinnect::moveGripper(geometry_msgs::TwistStamped position, std_msgs::Int8 mode){

    if(this->motionStatus.data != 0 && this->gripperStatus.data != 1 && this->gripperStatus.data == 3)
        gripper_mode_pub.publish(mode);// 1 pinch, 0 basic
    if(this->gripperStatus.data == 3)    
        gripper_speed_position_pub.publish(position);
    
    ROS_INFO("im controlling gripper\n");

    if(this->motionStatus.data != 0 )return true;
    else return false;

}

geometry_msgs::TwistStamped RobotKinnect::toggleGripper(bool gripper){

    geometry_msgs::TwistStamped output;

    output.header.stamp = ros::Time::now();

    output.twist.angular.x = 255;
    output.twist.angular.y = 255;
    output.twist.angular.z = 255;

    output.twist.linear.x =gripper * 255;
    output.twist.linear.y =gripper * 255;
    output.twist.linear.z =gripper * 255;


    
    return output;

}
geometry_msgs::Point RobotKinnect::cast2Point(double x, double y, double z){
    geometry_msgs::Point output;
    output.x = x;
    output.y = y;
    output.z = z;
    return output;
}

geometry_msgs::Quaternion RobotKinnect::euler2Quaternion(double x, double y, double z){

   geometry_msgs::Quaternion output;
    x=x*3.1415/180.;
    y=y*3.1415/180.;
    z=z*3.1415/180.;
    tf::Transform transform;
    /* double cr = cos(x * 0.5);
    double sr = sin(x * 0.5);
    double cp = cos(y * 0.5);
    double sp = sin(y * 0.5);
    double cy = cos(z * 0.5);
    double sy = sin(z * 0.5);*/
    tf::Quaternion quat;
    //output.x = //cr * cp * cy + sr * sp * sy;
    //output.w = //(sr * cp * cy - cr * sp * sy);
    //output.y = //(cr * sp * cy + sr * cp * sy);
    //output.z = //(cr * cp * sy - sr * sp * cy);
    quat.setEulerZYX(x,y,z);
    output.w = -quat.getW();
    output.x = -quat.getX();
    output.y = -quat.getY();
    output.z = -quat.getZ();
    
    return output;
};

void RobotKinnect::GripperObjectCallback(const geometry_msgs::Vector3::ConstPtr &data){
    this->gripperObject.x = data->x;
    this->gripperObject.y = data->y;
    this->gripperObject.z = data->z;

};
void RobotKinnect::GripperStatusCallback(const std_msgs::Int8::ConstPtr &data){
    this->gripperStatus.data = data->data;
};
void RobotKinnect::MotionStatusCallback(const std_msgs::Int8::ConstPtr &data){
    this->motionStatus.data = data->data;

};
void RobotKinnect::ObjectPositionCallback(const geometry_msgs::Vector3::ConstPtr &data){
    this->objectPosition.x = data->x;
    this->objectPosition.y = data->y;
    this->objectPosition.z = data->z;

};
                     