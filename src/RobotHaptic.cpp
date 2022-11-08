#include <robot_haptic/RobotHaptic.h>

RobotHaptic::RobotHaptic(ros::NodeHandle node, float loopRate, std::string robotPositionTopic, std::string hapticPositionTopic, 
                         std::string switchPositionTopic, std::string robotStateTopic, std::string interfAxisLockTopic, 
                         std::string interfCommandsTopic, std::string interfOrientationTopic):
                         node(node), loopRate(loopRate), robotPositionTopic(robotPositionTopic), hapticPositionTopic(hapticPositionTopic), 
                         switchPositionTopic(switchPositionTopic), robotStateTopic(robotStateTopic), interfAxisLockTopic(interfAxisLockTopic), 
                         interfCommandsTopic(interfCommandsTopic), interfOrientationTopic(interfOrientationTopic){

                            this->teleopStarted = false;
                            this->orientMode = false;

                            this->axis_lock[3] = {0};
                            this->motionScale = 2.0;
                            this->interfOrient[3] = {0};

                            this->rob_pos_pub = this->node.advertise<geometry_msgs::PoseStamped>(this->robotPositionTopic.c_str(),1);  
                            this->rob_pos_sub = this->node.subscribe<iiwa_msgs::CartesianPose>(this->robotStateTopic.c_str(),1 ,&RobotHaptic::RobotPositionCallBack,this);

                            this->haptic_pos_sub = this->node.subscribe<geometry_msgs::Twist>(this->hapticPositionTopic.c_str(),1, &RobotHaptic::HapticPositionCallBack, this);
                            this->haptic_switch_sub = this->node.subscribe<geometry_msgs::Twist>(this->switchPositionTopic.c_str(),1, &RobotHaptic::HapticSwitchCallBack, this);

                            //this->interf_start_sub = this->node.subscribe<std_msgs::Float64>(this->interfStartTopic.c_str(),1, &RobotHaptic::InterfStartCallBack, this);
                            //this->interf_scale_sub = this->node.subscribe<std_msgs::Float64>(this->interfScaleTopic.c_str(),1, &RobotHaptic::InterfScaleCallBack, this);
                            this->interf_axis_sub = this->node.subscribe<geometry_msgs::Vector3>(this->interfAxisLockTopic.c_str(),1, &RobotHaptic::InterfAxisLockCallBack, this);
                            this->interf_commands_sub = this->node.subscribe<std_msgs::Float64MultiArray>(this->interfCommandsTopic.c_str(),1, &RobotHaptic::InterfCommandsCallBack, this);
                            this->interf_orient_sub = this->node.subscribe<geometry_msgs::Vector3>(this->interfOrientationTopic.c_str(),1, &RobotHaptic::InterfOrientationCallBack, this);
                         }

RobotHaptic::~RobotHaptic(){
    //nada boss
}

void RobotHaptic::publishRobotData(){
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    robotPosition = oldRobotPosition.poseStamped;
    hapticOldPosition = hapticPosition;

    while(ros::ok()){
        if(teleopStarted){
            geometry_msgs::Vector3 hapticDisplacement;
            geometry_msgs::Vector3 hapticAngularDisplacement;
            geometry_msgs::PoseStamped robotDisplacement;

            robotPosition.header.stamp = ros::Time::now();
        
            if(hapticSwitch.linear.x < 0.06){

                hapticDisplacement.x = this->hapticOldPosition.linear.x - this->hapticPosition.linear.x;
                hapticDisplacement.y = this->hapticOldPosition.linear.y - this->hapticPosition.linear.y;
                hapticDisplacement.z = this->hapticOldPosition.linear.z - this->hapticPosition.linear.z; 

                //hapticAngularDisplacement.x = this->hapticOldPosition.angular.x - this->hapticPosition.angular.x;
                //hapticAngularDisplacement.y = this->hapticOldPosition.angular.y - this->hapticPosition.angular.y;
                //hapticAngularDisplacement.z = this->hapticOldPosition.angular.z - this->hapticPosition.angular.z;

                robotDisplacement.header.stamp = ros::Time::now();
                robotDisplacement.pose.position.x = hapticDisplacement.x;
                robotDisplacement.pose.position.y = hapticDisplacement.y;
                robotDisplacement.pose.position.z = hapticDisplacement.z;

                if(orientMode){
                    toEuler(robotPosition.pose.orientation.w,robotPosition.pose.orientation.x,
                    robotPosition.pose.orientation.y,robotPosition.pose.orientation.z);

                    robotEuler[0] = (robotEuler[0] + interfOrient[0]) * 3.1415/180; //hapticAngularDisplacement.x/10;
                    robotEuler[1] = (robotEuler[1] + interfOrient[1]) * 3.1415/180; //hapticAngularDisplacement.y/10;
                    robotEuler[2] = (robotEuler[2] + interfOrient[2]) * 3.1415/180; //hapticAngularDisplacement.z/10;

                    toQuat(robotEuler[0],robotEuler[1],robotEuler[2]);

                    robotPosition.pose.orientation.w = this->robotQuat[0];
                    robotPosition.pose.orientation.x = this->robotQuat[1];
                    robotPosition.pose.orientation.y = this->robotQuat[2];
                    robotPosition.pose.orientation.z = this->robotQuat[3];
                }else{
                    robotPosition.pose.orientation = oldRobotPosition.poseStamped.pose.orientation;
                    if(axis_lock[0]==1)
                        robotPosition.pose.position.x = robotPosition.pose.position.x;
                    else if(axis_lock[0]==0)
                        robotPosition.pose.position.x = robotPosition.pose.position.x - hapticDisplacement.x/1000 *motionScale;
                    if(axis_lock[1]==1)
                        robotPosition.pose.position.y = robotPosition.pose.position.y;
                    else if(axis_lock[1]==0)
                        robotPosition.pose.position.y = robotPosition.pose.position.y + hapticDisplacement.y/1000 *motionScale;
                    if(axis_lock[2]==1)
                        robotPosition.pose.position.z = robotPosition.pose.position.z;
                    else if(axis_lock[2]==0)
                        robotPosition.pose.position.z = robotPosition.pose.position.z - hapticDisplacement.z/1000 *motionScale;
                    std::cout<<axis_lock[0]<<" "<<axis_lock[1]<<" "<<axis_lock[2]<<std::endl;
                }

                //std::cout<<"haptic disp: "<<hapticDisplacement.x<<" "<<hapticDisplacement.y<<" "<<hapticDisplacement.z<<"\n";
                //std::cout<<"haptic now position: "<<hapticPosition.linear.x<< " "<<hapticPosition.linear.y<< " "<<hapticPosition.linear.z<<'\n';
                //std::cout<<robotEuler[0]<<" "<<robotEuler[1]<<" "<<robotEuler[2]<<'\n';
                //std::cout<<"robot pos"<<oldRobotPosition.pose.position.x<< " "<<oldRobotPosition.pose.position.y<<" "<<oldRobotPosition.pose.position.z<<"\n";
                
            }
            else robotPosition = oldRobotPosition.poseStamped;

            hapticOldPosition = hapticPosition;

            this->rob_pos_pub.publish(robotPosition);
        }   
        this->loopRate.sleep();
        //robotPosition = oldRobotPosition.poseStamped;
    }
    spinner.stop();
}

void RobotHaptic::toEuler(double w, double x, double y, double z){
    
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    this->robotEuler[0] = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        this->robotEuler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        this->robotEuler[1] = std::asin(sinp);
    
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    this->robotEuler[2] = std::atan2(siny_cosp, cosy_cosp);

}

void RobotHaptic::toQuat(double x, double y, double z){

    double cr = cos(x * 0.5);
    double sr = sin(x * 0.5);
    double cp = cos(y * 0.5);
    double sp = sin(y * 0.5);
    double cy = cos(z * 0.5);
    double sy = sin(z * 0.5);

    robotQuat[0] = cr * cp * cy + sr * sp * sy;
    robotQuat[1] = sr * cp * cy - cr * sp * sy;
    robotQuat[2] = cr * sp * cy + sr * cp * sy;
    robotQuat[3] = cr * cp * sy - sr * sp * cy;

}

void RobotHaptic::RobotPositionCallBack(const iiwa_msgs::CartesianPose::ConstPtr &data){
    
    this->oldRobotPosition.poseStamped = data->poseStamped;
    this->oldRobotPosition.redundancy = data->redundancy;
} 

void RobotHaptic::HapticPositionCallBack(const geometry_msgs::Twist::ConstPtr &data){
    this->hapticPosition.angular = data->angular;
    this->hapticPosition.linear = data->linear;
}

void RobotHaptic::HapticSwitchCallBack(const geometry_msgs::Twist::ConstPtr &data){
    this->hapticSwitch.angular = data->angular;
    this->hapticSwitch.linear = data->linear;
}

void RobotHaptic::InterfAxisLockCallBack(const geometry_msgs::Vector3::ConstPtr &data){
    this->axis_lock[0] = data->x;
    this->axis_lock[1] = data->y;
    this->axis_lock[2] = data->z;
}

void RobotHaptic::InterfCommandsCallBack(const std_msgs::Float64MultiArray::ConstPtr &data){
    
    /*
    *  Float64MultiArray command message structure
    *  x -> TELEOP START/STOP
    *  y -> ORIENTATION MODE START/STOP
    *  z -> MOTION SCALE FACTOR
    */
    
    if(data->data[0] == 1.0)              
        this->teleopStarted = true;
    else
        this->teleopStarted = false;

    if(data->data[1] == 1.0)
        this->orientMode = true;
    else
        this->orientMode = false;

    this->motionScale = data->data[2];
}

void RobotHaptic::InterfOrientationCallBack(const geometry_msgs::Vector3::ConstPtr &data){
    this->interfOrient[0] = data->x;
    this->interfOrient[1] = data->y;
    this->interfOrient[2] = data->z;
}
