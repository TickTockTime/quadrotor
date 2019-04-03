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

#include <sensor_msgs/Range.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <mavros_msgs/CommandTOL.h>

//px4flow
//distance
sensor_msgs::Range current_distance; 
void range_cb(const sensor_msgs::Range::ConstPtr& msg){  
    current_distance = *msg; 
}
//local_xy
mavros_msgs::OpticalFlowRad current_xy;
void xy_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& msg){
    current_xy = *msg;
}

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

int main(int argc, char **argv)  
{  
    ros::init(argc, argv, "offb_node");  
    ros::NodeHandle nh;  

    //Subscriber
    ros::Subscriber pxflow_distance_sub = nh.subscribe<sensor_msgs::Range>("mavros/px4flow/ground_distance", 1000, range_cb);                //订阅光流传感器的高度
    ros::Subscriber pxflow_xy_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("mavros/px4flow/raw/optical_flow_rad", 1000, xy_cb);           //订阅光流传感器的原始数据           
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1000, pose_cb);                  //订阅飞控的定位（没有gps飞控的xy坐标不准确，高度是气压计测量，相当不准确）
    ros::Subscriber local_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1000, velocity_cb);     //订阅飞控的速度
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);                                              //这是啥？好像是模式的说明
    
    //Publisher
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1000);                    //发布速度消息
    ros::Publisher local_accel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("mavros/setpoint_accel/accel", 1000);                        //发布加速度消息
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);                          //发布位置消息

    //ServiceClient
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");                                            //着陆服务端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");                                       //好像是安全开关服务端？
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");                                           //模式设定服务端
     
    //the setpoint publishing rate MUST be faster than 2Hz  
    ros::Rate rate(20.0);  
  
    // wait for FCU connection  
    while(ros::ok() && !current_state.connected){  
        ros::spinOnce();  
        rate.sleep();  
    }  

    int flag=1;                                                                                                                                //判断起飞的起始时间
    float x=0;                                                                                                                                 //相对于起飞点的位置的x分量
    float y=0;                                                                                                                                 //相对于起飞点的位置的y分量


    geometry_msgs::Twist vel;                                                                                                                  //发布的速度消息（线速度+角速度）
    vel.linear.x=0;
    vel.linear.y=0;
    vel.linear.z=0;
    //vel.angular.x=0;
    //vel.angular.y=0;
    //vel.angular.z=0;

    geometry_msgs::Vector3Stamped accel;                                                                                                        //发布的加速度消息
    accel.vector.x=0;                
    accel.vector.y=0;
    accel.vector.z=0;

	float temp = current_pose.pose.position.z;
    geometry_msgs::PoseStamped pose;                                                                                                            //发布的位置消息
    pose.pose.position.x = 0;  
    pose.pose.position.y = 0;
    pose.pose.position.z = temp;                                                                                     
    
    float first_distance = current_pose.pose.position.z;                                                                                        //确定起飞点的初始绝对高度


    //send a few setpoints before starting  
    for(int i = 100; ros::ok() && i > 0; --i){  
        local_vel_pub.publish(vel);
        local_pos_pub.publish(pose); 
        //accel_pub.publish(accel);  
        ros::spinOnce();  
        rate.sleep();  
    }  
  
    mavros_msgs::SetMode offb_set_mode;  
    offb_set_mode.request.custom_mode = "OFFBOARD";  
    mavros_msgs::CommandTOL land_cmd;
    //land_cmd.request.= true;
    mavros_msgs::CommandBool arm_cmd;  
    arm_cmd.request.value = true;  
  
    ros::Time last_request = ros::Time::now();  
    ros::Time land_request = ros::Time::now();
    float ttt = current_distance.range;
	ROS_INFO("%f",ttt);
    ROS_INFO("ok");

    while(ros::ok()){  
        if( current_state.mode != "OFFBOARD" &&  
            (ros::Time::now() - last_request > ros::Duration(5.0))){  
            if( set_mode_client.call(offb_set_mode) &&  
                offb_set_mode.response.mode_sent){  
                ROS_INFO("Offboard enabled");  
            }  
            last_request = ros::Time::now();  
        } else {  
            if( !current_state.armed &&  
                (ros::Time::now() - last_request > ros::Duration(5.0))){  
                if( arming_client.call(arm_cmd) &&  
                    arm_cmd.response.success){  
                    ROS_INFO("Vehicle armed");  
                    }
                if(flag){
                    ros::Time land_request = ros::Time::now();
                    ROS_INFO("ok");
                    flag=0;
                }
				last_request = ros::Time::now(); 
				}
                
                 
        }  
        if(ros::Time::now()-land_request > ros::Duration(15.0)){
			ROS_INFO("gjhkjkklklljhgvbbbnnmnmghgg");
            if(land_client.call(land_cmd)){
                ROS_INFO("555");
                break;
            }
        }
        else{
            /*
            if(put){
                pose.pose.position.x = -2-x;
                pose.pose.position.y = -y;
                pose.pose.position.z = 1-current_distance.range+current_pose.pose.position.z; 
                if(detect){
                    system(python "/home/tardis/catkin_ws/put.py");
                    put = 0;
                }
            }else{
                pose.pose.position.x = -x;
                pose.pose.position.y = -y;
                pose.pose.position.z = 1-current_distance.range+current_pose.pose.position.z;
                if(back){
                    system(python "/home/tardis/catkin_ws/catch.py");
                    back = 0;
                } 
            }
            */
			if(ros::Time::now()-land_request > ros::Duration(10.0)){
				system("python /home/tardis/catkin_ws/put.py");
			}
			float a = current_pose.pose.position.z;
			float p = current_distance.range;
			ROS_INFO("%f",a);
			ROS_INFO("%f",p);
		 
			
            //pose.pose.position.x = -0.3*x;
            //pose.pose.position.y = -0.3*y;
            pose.pose.position.z=1.5-p+a;
			//ROS_INFO("%f",x); 
			//ROS_INFO("%f",y); 

            //vel.linear.x=1;
			float v1=current_velocity.twist.linear.x;
			ROS_INFO("x %f",v1);
;
            /*if(current_distance.range<0.7){
                vel.linear.z=10;

            //accel.vector.z=0.5;
            }
            else{
                vel.linear.z=8;
                vel.linear.x=5;
            //accel.vector.z=0.5;
            }*/
            //local_vel_pub.publish(vel);
            local_pos_pub.publish(pose); 
        }
        //accel_pub.publish(accel);
        x = x + current_xy.integrated_x;
        y = y + current_xy.integrated_y;
        ros::spinOnce();  
        rate.sleep();  
    }  
  
    return 0;  
} 
