#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
// #include "control_msgs/FollowJointTrajectoryActionResult.h"
// #include "control_msgs/FollowJointTrajectoryActionFeedback.h"
// #include "actionlib_msgs/GoalStatusArray.h"
#include "unistd.h"
#include "actionlib/client/simple_action_client.h"
#include <vector>


#define YAW 0
#define PITCH 1
#define ROLL 2



control_msgs::FollowJointTrajectoryGoal create_neck_action_goal(std::vector<double>& joint_angles_start, std::vector<double>& joint_angles_end, double t1, double t2){
    control_msgs::FollowJointTrajectoryGoal  goal;
    

    int num_joints = joint_angles_start.size();
    int num_points = 2; //Start and End Trajectorys

    goal.goal_tolerance.resize(3); //3joints
    goal.path_tolerance.resize(3);

    for(int i = 0; i<num_joints; i++){ //Set tolerances to 0 makes them default (as specified in yaml?)
        
        goal.goal_tolerance[i].position = 0;
        goal.goal_tolerance[i].velocity = 0;
        goal.goal_tolerance[i].acceleration = 0;

        goal.path_tolerance[i].position = 0;
        goal.path_tolerance[i].velocity = 0;
        goal.path_tolerance[i].acceleration = 0;
    }

    goal.goal_time_tolerance.fromSec(0.25);//= ros::Duration(0.25);

    trajectory_msgs::JointTrajectory traj;

    traj.header.frame_id = "neck_frame";
    traj.header.stamp = ros::Time::now();

    traj.joint_names.resize(3);
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
            traj.points[i].effort[j] = 0;
        }
    }

    traj.points[0].positions[0] = joint_angles_start[0];
    traj.points[0].positions[1] = joint_angles_start[1];
    traj.points[0].positions[2] = joint_angles_start[2];
    traj.points[0].time_from_start = ros::Duration(t1);

    traj.points[1].positions[0] = joint_angles_end[0];
    traj.points[1].positions[1] = joint_angles_end[1];
    traj.points[1].positions[2] = joint_angles_end[2];
    traj.points[1].time_from_start = ros::Duration(t2);

    goal.trajectory = traj;

    return goal;
}

std::vector<double>  create_joint_vec(double yaw, double pitch, double roll){
    std::vector<double> joint_vec;
    joint_vec.resize(3);

    joint_vec[0] = yaw;
    joint_vec[1] = pitch;
    joint_vec[2] = roll;

    return joint_vec;
}

// void doneCb(const actionlib::SimpleClientGoalState &state, control_msgs::FollowJointTrajectoryResultConstPtr result) { //both variable were const 
//   ROS_INFO("The Action has been completed");
//   ros::shutdown();
// }

// // Definition of the active callback. It is called once when the goal becomes
// // active
// void activeCb() { ROS_INFO("Goal just went active"); }

// // Definition of the feedback callback. This will be called when feedback is
// // received from the action server. It just // prints a message indicating a new
// // message has been received
// void feedbackCb( control_msgs::FollowJointTrajectoryFeedbackConstPtr feedback) {
//   ROS_INFO("Feedback Recieved");
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "neck_action_client_test"); // Initialise a ROS node with the name service_client
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> neck_client("/dynamixel_controllers/nyku_neck_controller/follow_joint_trajectory",true); //connect node to action server

    neck_client.waitForServer(ros::Duration(2.0)); // Waits for connection with Server

    ROS_INFO("Server Reached");

    control_msgs::FollowJointTrajectoryGoal goal_pos; //Create Goal 

    std::vector<double> nyku_joint_vals_prev(3);
    std::vector<double> nyku_joint_vals_next(3);


    nyku_joint_vals_prev = create_joint_vec(0,0,0);
    nyku_joint_vals_next = create_joint_vec(1.0,1.0,1.0);
    goal_pos = create_neck_action_goal(nyku_joint_vals_prev,nyku_joint_vals_next, 2, 5);
    neck_client.sendGoal(goal_pos);
    ROS_INFO("Goal 1 Sent");
    neck_client.waitForResult();

    nyku_joint_vals_prev = create_joint_vec(1.0,1.0,1.0);
    nyku_joint_vals_next = create_joint_vec(1,1,0);
    goal_pos = create_neck_action_goal(nyku_joint_vals_prev,nyku_joint_vals_next,8,11);
    neck_client.sendGoal(goal_pos);
    ROS_INFO("Goal 2 Sent");
    neck_client.waitForResult();
  

    neck_client.waitForResult();
    
    
    ros::shutdown();

  
    return 0;
}