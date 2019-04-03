/**  
 * @file offb_node.cpp  
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight  
 * Stack and tested in Gazebo SITL  
 */  
#include <ros/ros.h> 

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>  
#include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/SetMode.h>  
#include <mavros_msgs/State.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/CommandTOL.h>

/*** px4flow data read ***/
// px4flow_ground_distance
sensor_msgs::Range current_distance; 
void distance_callback(const sensor_msgs::Range::ConstPtr& msg){  
    current_distance = *msg; 
}
// px4flow_optical_flow_rad
mavros_msgs::OpticalFlowRad px4flow_rad;
void px4flow_rad_callback(const mavros_msgs::OpticalFlowRad::ConstPtr& msg){
    px4flow_rad = *msg;
}
sensor_msgs::Imu imu_data;
void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_data = *msg;
}
geometry_msgs::PoseStamped current_pose;  
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){  
    current_pose = *msg;  
}
/*
//pxhawk
//current_pose
geometry_msgs::PoseStamped current_pose;  
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){  
    current_pose = *msg;  
}

mavros_msgs::State current_state;  
void state_cb(const mavros_msgs::State::ConstPtr& msg){  
    current_state = *msg;  
}
//current_velocity
geometry_msgs::TwistStamped current_velocity;
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){  
    current_velocity = *msg;  
}
*/
int main(int argc, char **argv)  
{  
    ros::init(argc, argv, "data_read");  
    ros::NodeHandle nh;  

    //Subscriber
    ros::Subscriber pxflow_distance_sub = nh.subscribe<sensor_msgs::Range>("mavros/px4flow/ground_distance", 1000, distance_callback);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 1000, imu_data_callback);
    ros::Subscriber pxflow_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("mavros/px4flow/raw/optical_flow_rad", 1000, px4flow_rad_callback);         
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1000, pose_callback); 
    // ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1000, pose_cb);                  //订阅飞控的定位（没有gps飞控的xy坐标不准确，高度是气压计测量，相当不准确）
    // ros::Subscriber local_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1000, velocity_cb);     //订阅飞控的速度
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);                                              //这是啥？好像是模式的说明
    
    //Publisher
    // ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1000);                    //发布速度消息
    // ros::Publisher local_accel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("mavros/setpoint_accel/accel", 1000);                        //发布加速度消息
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);                          //发布位置消息

    //ServiceClient
    // ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");                                            //着陆服务端
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");                                       //好像是安全开关服务端？
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");                                           //模式设定服务端
     
    //the setpoint publishing rate MUST be faster than 2Hz  
    ros::Rate rate(20.0);  

    while(ros::ok()){
        // float ttt = current_distance.range;
        float distance = px4flow_rad.distance;
        float integrated_x = px4flow_rad.integrated_x;
        float integrated_y = px4flow_rad.integrated_y;
        float integrated_xgyro = px4flow_rad.integrated_xgyro;
        float integrated_ygyro = px4flow_rad.integrated_ygyro;
        float integrated_zgyro = px4flow_rad.integrated_zgyro;


        float read_local_pose_positionx = current_pose.pose.position.x;
        float read_local_pose_positiony = current_pose.pose.position.y;
        float read_local_pose_positionz = current_pose.pose.position.z;
        float read_local_pose_orientationx = current_pose.pose.orientation.x;
        float read_local_pose_orientationy = current_pose.pose.orientation.y;
        float read_local_pose_orientationz = current_pose.pose.orientation.z;
        float read_local_pose_orientationw = current_pose.pose.orientation.w;
        ROS_INFO("--------local_pose_position------------");
        ROS_INFO("read_local_pose_position_x = %.6f", read_local_pose_positionx);    
        ROS_INFO("read_local_pose_position_y = %.6f", read_local_pose_positiony);    
        ROS_INFO("read_local_pose_position_z = %.6f", read_local_pose_positionz);    
        ROS_INFO("---------------------------------------");
        
        ROS_INFO("-----local_pose_orientation------------");
        ROS_INFO("read_local_pose_orientation_x = %.6f", read_local_pose_orientationx);    
        ROS_INFO("read_local_pose_orientation_y = %.6f", read_local_pose_orientationy);    
        ROS_INFO("read_local_pose_orientation_z = %.6f", read_local_pose_orientationz);    
        ROS_INFO("read_local_pose_orientation_w = %.6f", read_local_pose_orientationw);
        ROS_INFO("---------------------------------------");   


	    // ROS_INFO("%f",ttt);

        float read_imu_data_x = imu_data.orientation.x;
        float read_imu_data_y = imu_data.orientation.y;
        float read_imu_data_z = imu_data.orientation.z;
        float read_imu_data_w = imu_data.orientation.w;
        ROS_INFO("------------imu_data-------------------");
        ROS_INFO("read_imu_data_x = %.6f", read_imu_data_x);
        ROS_INFO("read_imu_data_y = %.6f", read_imu_data_y);
        ROS_INFO("read_imu_data_z = %.6f", read_imu_data_z);
        ROS_INFO("read_imu_data_w = %.6f", read_imu_data_w);
        ROS_INFO("---------------------------------------");
        
        ROS_INFO("%f",distance);
        ROS_INFO("integrated_x = %.6f", integrated_x);
        ROS_INFO("integrated_y = %.6f", integrated_y);
        ROS_INFO("integrated_xgyro = %.6f", integrated_xgyro);
        ROS_INFO("integrated_ygyro = %.6f", integrated_ygyro);
        ROS_INFO("integrated_zgyro = %.6f", integrated_zgyro);

        ROS_INFO("ok");
        ros::spinOnce();  
        rate.sleep(); 
    }
    return 0;  
} 
