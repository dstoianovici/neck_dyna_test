#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>


trajectory_msgs::JointTrajectory create_nyku_neck_joint_traj_msg(std::vector<double>& joint_angles, double sec_dur){

    //0: yaw
    //1: pitch
    //2: roll

    trajectory_msgs::JointTrajectory traj;

    int num_joints = 3; //Must be 3
    int num_points = 2; //Trajectory points 

    // traj.header.frame_id = "neck_frame"; //Non-essential
    traj.header.stamp = ros::Time::now(); // Can be added to to execute in the future
    // traj.header.seq = seq; //set automatically through ros

    traj.joint_names.resize(num_joints);
    traj.joint_names[0] = "yaw";
    traj.joint_names[1] = "pitch";
    traj.joint_names[2] = "roll";


    traj.points.resize(num_points); //Resize points array to number of trajectory points

    for(std::size_t i = 0; i < num_points; i++){
        traj.points[i].positions.resize(num_joints); //Resize pos and vel to number of joints
        traj.points[i].velocities.resize(num_joints);
        traj.points[i].effort.resize(num_joints);
        traj.points[i].accelerations.resize(num_joints);

        for(int j =0; j < num_joints; j++){
            traj.points[i].velocities[j] = 0;
            traj.points[i].accelerations[j] = 0;
            traj.points[i].effort[j] = 5;
        }
    }

    traj.points[0].positions[0] = 0;
    traj.points[0].positions[1] = 0;
    traj.points[0].positions[2] = 0;
    traj.points[0].time_from_start = ros::Duration(0);

    traj.points[1].positions[0] = joint_angles[0];
    traj.points[1].positions[1] = joint_angles[1];
    traj.points[1].positions[2] = joint_angles[2];
    traj.points[1].time_from_start = ros::Duration(sec_dur);

    return traj;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "dyna_control_test"); // Initialise a ROS node with the name service_client
    ros::NodeHandle nh;
    ros::Publisher dyna_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_controllers/nyku_neck_controller/command", 1000);
    ROS_INFO("dyna_control_test node initialized");

    std::vector<double> nyku_joint_vals(3);

    nyku_joint_vals[0] = 0.0;
    nyku_joint_vals[1] = 0.0;
    nyku_joint_vals[2] = 0.0;

    ROS_INFO("Joint Vals Var Created");

    trajectory_msgs::JointTrajectory traj;

    ROS_INFO("vars");

    traj = create_nyku_neck_joint_traj_msg(nyku_joint_vals,6);

    dyna_pub.publish(traj);
    sleep(5);

    // while(ros::ok()){
    //     sleep(1.0);
    //     dyna_pub.publish(traj);
    //     ros::spinOnce();
    // }

    ROS_INFO("Completed");

    ros::spinOnce();

    ros::shutdown();

    return 0;
}

