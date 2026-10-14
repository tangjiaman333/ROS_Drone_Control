#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
     current_state = *msg;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    ros::Publisher local_pos_pub=nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client=nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client=nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 等连上飞控
    while(ros::ok() && !current_state.connected){
         ros::spinOnce();
         rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x=0;
    pose.pose.position.y=0;
    pose.pose.position.z=2;

    // 预发100次目标点（热身）
    for(int i=100;ros::ok() && i>0;--i){
       local_pos_pub.publish(pose);
       ros::spinOnce();
       rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode="OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value=true;
    ros::Time last_request=ros::Time::now();

    // 切OFFBOARD + 解锁
    while(ros::ok() && !current_state.armed){
         if(current_state.mode!="OFFBOARD" && (ros::Time::now()-last_request>ros::Duration(5.0))){
              if(set_mode_client.call(offb_set_mode)&&offb_set_mode.response.mode_sent){
                 ROS_INFO("Offboard enabled");
              }
              last_request=ros::Time::now();
         }else{
              if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0))){
                   if(arming_client.call(arm_cmd)&&arm_cmd.response.success){
                      ROS_INFO("Vehicle armed");
                   }
                   last_request=ros::Time::now();
              }
         }
         local_pos_pub.publish(pose);
         ros::spinOnce();
         rate.sleep();
    }

    // 走航点（正方形），每个点停8秒，看得清
    float waypoints[4][3]={
       {2,0,2},
       {2,2,2},
       {0,2,2},
       {0,0,2}
    };
    for(int j=0;j<4;j++){
       pose.pose.position.x=waypoints[j][0];
       pose.pose.position.y=waypoints[j][1];
       pose.pose.position.z=waypoints[j][2];
       ROS_INFO("Fly to waypoint %d", j+1);
       for(int i=160;ros::ok() && i>0;--i){
          local_pos_pub.publish(pose);
          ros::spinOnce();
          rate.sleep();
       }
    }

    // 降落：回到原点，慢慢降到地面
    pose.pose.position.x=0;
    pose.pose.position.y=0;
    pose.pose.position.z=0.1;
    ROS_INFO("Landing...");
    for(int i=200;ros::ok() && i>0;--i){
       local_pos_pub.publish(pose);
       ros::spinOnce();
       rate.sleep();
    }

    // 关电机
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value=false;
    if(arming_client.call(disarm_cmd)&&disarm_cmd.response.success){
       ROS_INFO("Disarmed");
    }

    ROS_INFO("Mission complete");
    return 0;
}
