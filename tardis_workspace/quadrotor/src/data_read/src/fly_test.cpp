/**  
 * @ Author: Yao Chengtao
 * @ Email: 3160102438@zju.edu.cn
 * @ Time: 11:30 PM 2019/4/3
 * @ Brief: fly test node, written with MAVROS  
 */ 

#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h> 
#include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/SetMode.h>  
#include <mavros_msgs/State.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>

// state of the drone
mavros_msgs::State current_state;  
void state_callback(const mavros_msgs::State::ConstPtr& msg){  
    current_state = *msg;  
}

// imu data, we use the quaternion to discribe the orientation
sensor_msgs::Imu imu_data;
void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_data = *msg;
}

// imu_pose
geometry_msgs::PoseStamped current_pose;  
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){  
    current_pose = *msg;  
}

// px4flow distance
sensor_msgs::Range current_distance; 
void distance_callback(const sensor_msgs::Range::ConstPtr& msg){  
    current_distance = *msg;
}

int main(int argc, char **argv)  
{  
    ros::init(argc, argv, "fly_test");  
    ros::NodeHandle nh;  

    float set_hight = 1.5;
    //Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback); 
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 1000, imu_data_callback);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1000, pose_callback); 
    ros::Subscriber pxflow_distance_sub = nh.subscribe<sensor_msgs::Range>("mavros/px4flow/ground_distance", 1000, distance_callback);

    //Publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000); 

    //ServiceClient
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // Set publishing rate must be faster than 2Hz
    ros::Rate rate(20.0);  

    // Wait the FCU connected
    while(ros::ok() && !current_state.connected){  
        ros::spinOnce(); 
        rate.sleep();
    }

    /*** Init some parameters **/
    // Read local pose
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

    // Read the data from the imu
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

    // Read the hight info
    float read_distance = current_distance.range;

    ROS_INFO("------------distance-------------------");
    ROS_INFO("read_distance = %.6f", read_distance);
    ROS_INFO("---------------------------------------");

    // set local pose ( position (x, y, z) orientation(x, y, z, w) )
    geometry_msgs::PoseStamped local_pose;
    local_pose.pose.position.x = 0;
    local_pose.pose.position.y = 0;
    local_pose.pose.position.z = set_hight;
    local_pose.pose.orientation.x = read_imu_data_x;    
    local_pose.pose.orientation.y = read_imu_data_y;
    local_pose.pose.orientation.z = read_imu_data_z;
    local_pose.pose.orientation.w = read_imu_data_w;

    // Send a few setposes before starting  
    for(int i = 100; ros::ok() && i > 0; --i){  
        local_pos_pub.publish(local_pose);
        ros::spinOnce();  
        rate.sleep();
    }
    
    // Set mode
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    // Unlock the drone
    mavros_msgs::CommandBool arm_cmd;  
    arm_cmd.request.value = true; 

    // Set timer start
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" && (ros::Time::now()-last_request > ros::Duration(5.0))){

            if(set_mode_client.call(set_mode) && set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled!"); 
            }

            last_request = ros::Time::now();

        }else{
            
            if(!current_state.armed && (ros::Time::now()-last_request > ros::Duration(5.0))){
                if(arm_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Drone Armed!");
                }
                last_request = ros::Time::now();
            }

            read_distance = current_distance.range;
            ROS_INFO("------------distance-------------------");
            ROS_INFO("read_distance = %.6f", read_distance);
            ROS_INFO("---------------------------------------");
            read_local_pose_positionz = current_pose.pose.position.z;
            ROS_INFO("--------local_pose_position------------");
            ROS_INFO("read_local_pose_position_x = %.6f", read_local_pose_positionx);    
            ROS_INFO("---------------------------------------");
            local_pose.pose.position.z = set_hight - read_distance + read_local_pose_positionz;
            local_pos_pub.publish(local_pose);

            ros::spinOnce();  
            rate.sleep(); 

        }
    }
    return 0;  
} 



