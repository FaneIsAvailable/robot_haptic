#include <robot_haptic/RobotHaptic.h>

RobotHaptic::RobotHaptic(ros::NodeHandle node, float loopRate, std::string robotPositionTopic,
                         std::string hapticPositionTopic, std::string switchPositionTopic, std::string robotStateTopic):
                         node(node), loopRate(loopRate), robotPositionTopic(robotPositionTopic),
                         hapticPositionTopic(hapticPositionTopic), switchPositionTopic(switchPositionTopic),robotStateTopic(robotStateTopic){

                            this->rob_pos_pub = this->node.advertise<geometry_msgs::PoseStamped>(this->robotPositionTopic.c_str(),1);  
                            this->rob_pos_sub = this->node.subscribe<iiwa_msgs::CartesianPose>(this->robotStateTopic.c_str(),1 ,&RobotHaptic::RobotPositionCallBack,this);

                            this->haptic_pos_sub = this->node.subscribe<geometry_msgs::Twist>(this->hapticPositionTopic.c_str(),1, &RobotHaptic::HapticPositionCallBack, this);
                            this->haptic_switch_sub = this->node.subscribe<geometry_msgs::Twist>(this->switchPositionTopic.c_str(),1, &RobotHaptic::HapticSwitchCallBack, this);
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

        geometry_msgs::Vector3 hapticDisplacement;
        geometry_msgs::Vector3 hapticAngularDisplacement;
        geometry_msgs::PoseStamped robotDisplacement;

        robotPosition.header.stamp = ros::Time::now();
       
        if(hapticSwitch.linear.x < 0.06){

            hapticDisplacement.x = this->hapticOldPosition.linear.x - this->hapticPosition.linear.x;
            hapticDisplacement.y = this->hapticOldPosition.linear.y - this->hapticPosition.linear.y;
            hapticDisplacement.z = this->hapticOldPosition.linear.z - this->hapticPosition.linear.z; 

            hapticAngularDisplacement.x = this->hapticOldPosition.angular.x - this->hapticPosition.angular.x;
            hapticAngularDisplacement.y = this->hapticOldPosition.angular.y - this->hapticPosition.angular.y;
            hapticAngularDisplacement.z = this->hapticOldPosition.angular.z - this->hapticPosition.angular.z;

            robotDisplacement.header.stamp = ros::Time::now();
            robotDisplacement.pose.position.x = hapticDisplacement.x;
            robotDisplacement.pose.position.y = hapticDisplacement.y;
            robotDisplacement.pose.position.z = hapticDisplacement.z;

            toEuler(robotPosition.pose.orientation.w,robotPosition.pose.orientation.x,
                    robotPosition.pose.orientation.y,robotPosition.pose.orientation.z);

            robotEuler[0] = robotEuler[0] + hapticAngularDisplacement.x/10;
            robotEuler[1] = robotEuler[1] + hapticAngularDisplacement.y/10;
            robotEuler[2] = robotEuler[2] + hapticAngularDisplacement.z/10;

            toQuat(robotEuler[0],robotEuler[1],robotEuler[2]);

           // std::cout<<"haptic disp: "<<hapticDisplacement.x<<" "<<hapticDisplacement.y<<" "<<hapticDisplacement.z<<"\n";
            //std::cout<<"haptic now position: "<<hapticPosition.linear.x<< " "<<hapticPosition.linear.y<< " "<<hapticPosition.linear.z<<'\n';
            //std::cout<<robotEuler[0]<<" "<<robotEuler[1]<<" "<<robotEuler[2]<<'\n';
            //std::cout<<"robot pos"<<oldRobotPosition.pose.position.x<< " "<<oldRobotPosition.pose.position.y<<" "<<oldRobotPosition.pose.position.z<<"\n";
            robotPosition.pose.position.x = robotPosition.pose.position.x + hapticDisplacement.x/1000 *2;
            robotPosition.pose.position.y = robotPosition.pose.position.y + hapticDisplacement.y/1000 *2;
            robotPosition.pose.position.z = robotPosition.pose.position.z + hapticDisplacement.z/1000 *2;
            
            

           // robotPosition.pose.orientation.w = this->robotQuat[0];
           // robotPosition.pose.orientation.x = this->robotQuat[1];
           // robotPosition.pose.orientation.y = this->robotQuat[2];
           // robotPosition.pose.orientation.z = this->robotQuat[3];
            
        }
        else robotPosition = oldRobotPosition.poseStamped;


        
        hapticOldPosition = hapticPosition;


        this->rob_pos_pub.publish(robotPosition);
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