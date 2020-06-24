#include "ros/ros.h"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <unistd.h>


#define YAW 1
#define PITCH 2
#define ROLL 3

//Midpoints
// int yaw_pos = 500;
// int pitch_pos = 502;
// int roll_pos = 520;

ros::ServiceClient neck_controller_client;

dynamixel_workbench_msgs::DynamixelCommand yaw_cmd; //id 1
dynamixel_workbench_msgs::DynamixelCommand pitch_cmd; //id 2
dynamixel_workbench_msgs::DynamixelCommand roll_cmd; //id 3

//Control
int yaw_pos = 500;
int pitch_pos = 502;
int roll_pos = 520;


dynamixel_workbench_msgs::DynamixelCommand create_dymx_msg(uint id, int position){
    dynamixel_workbench_msgs::DynamixelCommand cmd_msg;
    cmd_msg.request.addr_name = "Goal_Position";
    cmd_msg.request.id = id;
    cmd_msg.request.value = position;

    return cmd_msg;
}


void send3Axis_cmd(dynamixel_workbench_msgs::DynamixelCommand yaw, dynamixel_workbench_msgs::DynamixelCommand pitch, dynamixel_workbench_msgs::DynamixelCommand roll){
       if(neck_controller_client.call(yaw_cmd)){
        ROS_INFO("Yaw: %d", yaw_cmd.response.comm_result);
    }
    else{
        ROS_ERROR("Yaw Error");
    }

    if(neck_controller_client.call(pitch_cmd)){
        ROS_INFO("Pitch: %d", pitch_cmd.response.comm_result);
    }
    else{
        ROS_ERROR("Pitch Error");
    }
    if(neck_controller_client.call(roll_cmd)){
        ROS_INFO("Roll: %d", roll_cmd.response.comm_result);
    }
    else{
        ROS_ERROR("Roll Error");
    }

}



int main(int argc, char** argv){
    ros::init(argc, argv, "dynamixel_neck_client"); // Initialise a ROS node with the name service_client
    ros::NodeHandle nh;

    neck_controller_client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand> ("neck_controller/dynamixel_command");
    //Separate client for each axis?


    yaw_cmd = create_dymx_msg(YAW, yaw_pos);
    pitch_cmd = create_dymx_msg(PITCH, pitch_pos);
    roll_cmd = create_dymx_msg(ROLL, roll_pos);
    send3Axis_cmd(yaw_cmd, pitch_cmd, roll_cmd);
    ROS_INFO("Homed");
    sleep(2);

    

    for(int i = 0; i < 21; i++){
        yaw_cmd = create_dymx_msg(YAW, yaw_pos + i*25);
        pitch_cmd = create_dymx_msg(PITCH, pitch_pos + i*5);
        roll_cmd = create_dymx_msg(ROLL, roll_pos + i*5);
        send3Axis_cmd(yaw_cmd, pitch_cmd, roll_cmd);
        //usleep(50);
    }

    for(int i = 0; i < 21; i++){
        yaw_cmd = create_dymx_msg(YAW, yaw_pos + 500 - i*25);
        pitch_cmd = create_dymx_msg(PITCH, pitch_pos + 100 - i*5);
        roll_cmd = create_dymx_msg(ROLL, roll_pos + + 100 - i*5);
        send3Axis_cmd(yaw_cmd, pitch_cmd, roll_cmd);
        //usleep(50);
    }

    yaw_cmd = create_dymx_msg(YAW, yaw_pos);
    pitch_cmd = create_dymx_msg(PITCH, pitch_pos);
    roll_cmd = create_dymx_msg(ROLL, roll_pos);
    send3Axis_cmd(yaw_cmd, pitch_cmd, roll_cmd);
    ROS_INFO("back home");
    sleep(2);


    ros::shutdown();

    return 0;
}